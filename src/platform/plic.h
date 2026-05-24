//
// Created by liujilan on 2026/5/23.
// PLIC (Platform-Level Interrupt Controller) 模块对外接口。
//
// 职责: per-source 中断状态维护 + per-ctx enable/threshold/priority 仲裁 + bus MMIO
//        暴露给 guest + 给 hart 侧 csr_mip_read 合成 MEIP/SEIP 用的 pending 查询。
//
// 地址布局 / 寄存器形态 见 config.h PLIC_* 宏 (跟 QEMU virt + RV PLIC spec v1.0.0
// 一致)。bus 接口形态 (read/write 返 cause / 0=成功) 见 platform/bus.h + dummy.txt §9。
//
// ----------------------------------------------------------------------------
// monitor 模型 (dummy.txt §7) — T6.2 升级成"完整 monitor + refresh 辅助线程"
// ----------------------------------------------------------------------------
//
// T6.2 前: PLIC 是 "monitor 但无线程" — 所有路径都是外设 device_set/clear_pending
// 或 hart MMIO 触发的同步调用, 无后台推进需求。但 a_03 末性能实测发现 dispatcher
// 主帧 csr_mip_read 调 is_plic_*_pending 走 rdlock + scan, 中断检查路径拖累
// 03_csr_heavy 等 fixture ~10x。
//
// T6.2 后: PLIC 升级成"完整 monitor + refresh 辅助线程":
//   - device_set/clear_pending 不再同步改 sources[].device_line, 改成入队
//     PLIC_REFRESH_OP_SET/CLEAR event 到 plic_refresh_queue (file-static ring,
//     16 slot; producer 满则 cond_timedwait not_full 100ms 心跳检 SDS)
//   - plic_pending_refresh_thread (consumer, single) 异步消费 event, 持 wrlock
//     改 sources[].device_line + 重算 plic_ctx_eip atomic 子集
//   - is_plic_meip/seip_pending 改 atomic_load(plic_ctx_eip[ctx_id]) 直返,
//     零 lock + 零 scan (signs 透明, csr_mip_read 调用点零侵入)
//   - claim/complete 慢路径不动: 仍 wrlock 改 sources[].claimed; 释放锁前顺手
//     调 plic_recompute_ctx_eip_locked(ctx_id) 保持 plic_ctx_eip 反映最新
//
// 三 monitor 范式 (a_03 末集齐):
//   CLINT = "monitor + timer 辅助线程"   (mtime 由 host wall clock 推进, 后台线程必须)
//   UART  = "monitor + reader 辅助线程"  (RX 源 = host stdin, blocking read 必须后台)
//   PLIC  = "monitor + refresh 辅助线程" (T6.2 后; 中断检查热路径要 atomic_load 直返)
//
// dummy.txt §12 "谁 spawn 谁 join" 协议适用 — plic_start_pending_refresh_thread /
// plic_join_pending_refresh_thread 暴露给 main, 跟 clint/uart 同形态; 受 SDS 控制
// 跨 system reset 一直跑。
//
// ----------------------------------------------------------------------------
// 读写抽象 (monitor RW; pthread_rwlock_t 实装)
// ----------------------------------------------------------------------------
//
// PLIC 内部用 pthread_rwlock_t 单锁; file-static helper plic_rdlock / plic_wrlock /
// plic_unlock 包装, 不对外暴露 (调用方都是 plic.c 内部函数)。
//
// 路径分配 (按字段访问是读是写, 跟 MMIO 入口形态无关 — claim "读寄存器" 但 set
// claimed 字段, 走 wrlock):
//   - rdlock: is_plic_*_pending / plic_read 命中纯读寄存器 (priority/threshold/enable/pending)
//   - wrlock: device_set/clear_pending / plic_read 命中 claim / plic_write 命中
//             complete/priority/threshold/enable
//
// SMP 单 hart 下 contention=0; 多 hart 真起来时 N reader + 1 writer 是 pthread_rwlock_t
// 的语义本职。公平性走默认 (reader 优先), 真撞 writer starvation 再调
// PTHREAD_RWLOCK_PREFER_WRITER_NP attr。
//
// ----------------------------------------------------------------------------
// 字段模型 vs RV PLIC spec — per-source 简化 + per-ctx 全实装
// ----------------------------------------------------------------------------
//
// per-source 两字段简化 (RV PLIC spec v1.0.0 §5 Interrupt Gateways 三态机的两字段化):
//
//   spec device line          — 设备到 PLIC 的物理信号 (level 或 edge), 设备侧拥有
//   spec IP bit               — gateway 输出, "该 source 有未处理 interrupt"; claim 清
//   spec gateway forward latch — gateway 内部锁存, claim set, complete clear; 锁住
//                                期间 IP 不被新 forward (spec 通用规则, 任一 source
//                                任一时刻 PLIC core 最多 1 pending request)
//
// 我们简化 (per source):
//
//   device_line  ≈ spec device line 镜像 (设备调 device_set/clear_pending 直接拉)
//   claimed      ≈ spec gateway forward latch (0=未 claim; 非 0 = ctx_id + 1)
//   priority     = spec source priority (RV PLIC spec: priority = 0 永不触发中断)
//   不存 IP bit, 用 (enable[ctx][src] && device_line && !claimed && priority>threshold)
//   合成 "送 hart" view
//
// per-ctx 完整实装 (跟 spec 字段一对一映射):
//
//   threshold              = spec context priority threshold (priority > threshold 才送)
//   enable[(N_SOURCES+31)/32] = spec context interrupt enable bitmap, 1 bit/source
//
// 命名理由: per-source 简化是为了 level-triggered 正确 driver 序列下零差异 (forward
// latch 是 spec 通用规则, 我们的 claimed 等价它); per-ctx 字段直接 spec 命名 (priority/
// threshold/enable), 跟外部 is_plic_*_pending 接口语义不冲突。pending 留给外部接口名,
// 跟内部 source 级字段名错开。一字段 (claimed) 顶 spec 里 "IP bit + gateway latch"
// 两态; edge-triggered 未来支持时, 加 source-side latch 字段单独存即可不冲突。
//
// ----------------------------------------------------------------------------
// plic_ctx_map (hart_priv → ctx_id 反向映射)
// ----------------------------------------------------------------------------
//
// index = (hartid << 2) + priv (复用 cpu_t.tlb_table 的 priv encoding: U=0/S=1/VS=2/M=3);
// 元素 = 0..PLIC_N_CONTEXTS-1; -1 = 没连线 (跟 dtb interrupts-extended 没列出来的
// hart×priv 对应)。
//
// 填充时机: v1 简化 — plic_init 内 hardcoded, 假定每 hart 都是 MSU, 按 QEMU virt
// sifive_plic 惯例 ctx_id 顺序 M 小 S 大 (M ctx = 2h, S ctx = 2h+1)。
//
// 未来 dtb 接入后: 改成 plic_set_ctx_map(hartid, priv, ctx_id) 接口, main 在 dtb 解析
// 后显式调; 不影响 PLIC 内部状态机, 只换填表入口。
//
// ----------------------------------------------------------------------------
// 三函数 lifecycle
// ----------------------------------------------------------------------------
//
// plic_init    — POR 一次性 (main 入口, clint_init 之后): 字段 0 init + plic_ctx_map
//                hardcoded 填充 (v1 全 MSU) + plic_ctx_eip atomic 清 0 +
//                plic_refresh_queue mutex/双 cond init + pthread_rwlock_init +
//                bus 注册。失败 -1 (跟 ram_init / clint_init 同形态; dummy.txt §5)。
//                **不发线程** (跟 clint_init 同体例 — spawn 由专门函数)。
//
// plic_reset   — system reset 每 iter (main while 顶段): 清 device_line / claimed /
//                priority / threshold / enable bitmap + plic_refresh_queue (head=tail=0
//                + broadcast cond) + plic_ctx_eip 全 atomic_store 0; plic_ctx_map
//                不动 (硬件 wired 状态不掉电不停, 跟 mtime 不动同性质); 锁不动
//                (基础设施); refresh_thread 不动 (受 SDS 跨 reset 一直跑, 跟 CLINT
//                timer / UART reader 同形态)。锁顺序 (避 deadlock): 先 mutex 清
//                queue 释放, 再 wrlock 清 plic 字段释放; 从不同时持两个锁。
//
// plic_destroy — POR 收尾 (main 末段): pthread_rwlock_destroy +
//                pthread_cond_destroy(both) + pthread_mutex_destroy; refresh thread
//                已由 plic_join_pending_refresh_thread 收回, 这里只清同步原语。
//
// ----------------------------------------------------------------------------
// thread lifecycle (dummy.txt §12 谁 spawn 谁 join; T6.2 新加, 跟 clint/uart 同体例)
// ----------------------------------------------------------------------------
//
// plic_start_pending_refresh_thread — main POR spawn (clint_start_timer_thread /
//   uart_start_reader_thread 同位置, 进 while 之前): pthread_create refresh thread,
//   routine 跨 system reset 一直跑 (受 SDS 控制起停)。错误走 runtime signal 通道
//   (fprintf + set SRS=0+SDS=0; 跟 clint_start_timer_thread 同形态), 不分 error path
//   destroy chain 只在 while 外写一次。
//
// plic_join_pending_refresh_thread  — POR 退出段 (main while 外, 跟 clint_join_timer_
//   thread / uart_join_reader_thread 同位置): SDS=0 之后调; refresh loop 自然退出 →
//   pthread_join 不永远 block。spawn fail path 防御 (refresh_thread 字段 BSS 0,
//   pthread_join(0) glibc 下返 ESRCH 一行 fprintf 不 fatal)。
//
// ----------------------------------------------------------------------------
// 外部接口 (T2' 暴露 + T6.2 改 atomic 路径)
// ----------------------------------------------------------------------------
//
// is_plic_meip_pending / is_plic_seip_pending — hart 侧合成 mip 用 (csr_mip_read +
//   trap_check_interrupt 入口); 内部查 plic_ctx_map 拿 ctx_id, 然后
//   atomic_load_explicit(plic_ctx_eip[ctx_id], acquire) 直返 (T6.2 改; 之前是
//   rdlock + scan)。priv 编码进函数名, 跟 is_clint_msip_pending 同体例; 函数签名
//   透明, csr_mip_read 调用点零侵入。
//
// device_set_pending / device_clear_pending — 外设侧通知 PLIC 拉高/拉低 device line
//   (UART / virtio-blk 等); source_id 0 / 越界 silent ignore。T6.2 后内部改异步
//   入队 plic_refresh_queue + cond_signal not_empty, plic_pending_refresh_thread
//   异步消费 (接口签名不变, 调用方零侵入)。fixture 通过 test_dev MMIO 写 (sw
//   TEST_DEV_SET_OFF / CLEAR_OFF) 触发, 详 src/device/test_dev.{h,c}。
//

#ifndef PLATFORM_PLIC_H
#define PLATFORM_PLIC_H

#include <stdint.h>

int  plic_init(void);
int  plic_reset(void);
void plic_destroy(void);

void plic_start_pending_refresh_thread(void);
void plic_join_pending_refresh_thread (void);

int  is_plic_meip_pending(uint32_t hartid);
int  is_plic_seip_pending(uint32_t hartid);

void device_set_pending  (uint32_t source_id);
void device_clear_pending(uint32_t source_id);

#endif //PLATFORM_PLIC_H
