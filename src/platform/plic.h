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
// monitor 模型 (dummy.txt §7) — "monitor 但无辅助线程"
// ----------------------------------------------------------------------------
//
// 所有路径都是外设 device_set/clear_pending 或 hart MMIO 触发的同步调用, 无后台
// 推进需求。hot path 优化通过 atomic 字段 (plic_ctx_eip + plic_pending_bitmap_cache)
// 实现 — wrlock 路径内 producer 同步设置 atomic, hart 主帧 csr_mip_read 走
// atomic_load 直返, 零 lock 零 scan.
//
// 演进 trail (a_03 milestone): T6.2 (a_03_007 末 ~ a_03_009) 曾引入 refresh queue
// + refresh thread 把 device_line 改异步, 但在 a_03_009 撞到 handler 同步 ACK +
// 异步 CLEAR race 引起的 spurious re-fire, 最终回退同步形态. hot path 优化的本质
// 是 atomic 字段而非异步 queue. 详 notes/context/plic_evolution_report.md.
//
// monitor 范式三态 (a_03 末):
//   CLINT      = "monitor + timer 辅助线程"     (mtime 由 host wall clock 推进)
//   UART       = "monitor + reader 辅助线程"    (RX 源 = host stdin, blocking read)
//   virtio-blk = "monitor + io_worker 辅助线程" (异步 pread/pwrite + IRQ)
//   PLIC       = "monitor 但无辅助线程"         (atomic 字段直接做 hot path 优化)
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
//   - 不持锁: is_plic_*_pending (atomic_load plic_ctx_eip) /
//             plic_read pending 区 (atomic_load plic_pending_bitmap_cache)
//   - rdlock: plic_read 命中纯读寄存器 (priority/threshold/enable)
//   - wrlock: device_set/clear_pending (跨线程同步) / plic_read 命中 claim /
//             plic_write 命中 complete/priority/threshold/enable
//
// wrlock 路径都 "进锁 → 改字段 → recompute_ctx_eip atomic_store(eip) →
// recompute_pending_bitmap atomic_store(cache) → 出锁" 模式, 保 atomic cache 反映
// 最新有效 view.
//
// SMP 单 hart 下 contention=0; 多 hart 真起来时 N reader + 1 writer 是
// pthread_rwlock_t 的语义本职。公平性走默认 (reader 优先), 真撞 writer starvation
// 再调 PTHREAD_RWLOCK_PREFER_WRITER_NP attr。
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
//                hardcoded 填充 (v1 全 MSU) + plic_ctx_eip / plic_pending_bitmap_cache
//                atomic 清 0 + pthread_rwlock_init + bus 注册。失败 -1 (跟
//                ram_init / clint_init 同形态; dummy.txt §5)。
//
// plic_reset   — system reset 每 iter (main while 顶段): wrlock 清 sources / contexts
//                字段 + plic_ctx_eip atomic 清 0 + plic_pending_bitmap_cache atomic
//                清 0; plic_ctx_map 不动 (硬件 wired 状态不掉电不停)。
//
// plic_destroy — POR 收尾 (main 末段): pthread_rwlock_destroy. PLIC 无辅助线程,
//                无 cond/mutex 清理.
//
// ----------------------------------------------------------------------------
// 外部接口
// ----------------------------------------------------------------------------
//
// is_plic_meip_pending / is_plic_seip_pending — hart 侧合成 mip 用 (csr_mip_read +
//   trap_check_interrupt 入口); 内部查 plic_ctx_map 拿 ctx_id, 然后
//   atomic_load_explicit(plic_ctx_eip[ctx_id], acquire) 直返, 零 lock 零 scan.
//   priv 编码进函数名, 跟 is_clint_msip_pending 同体例; 函数签名透明, csr_mip_read
//   调用点零侵入。
//
// device_set_pending / device_clear_pending — 外设侧通知 PLIC 拉高/拉低 device line
//   (UART / virtio-blk 等); source_id 0 / 越界 silent ignore。内部 wrlock + 改
//   device_line + recompute_ctx_eip atomic_store + recompute_pending_bitmap
//   atomic_store + unlock, 同步完成. 跨线程调用 (worker / reader thread) 跟
//   hart 主帧调用走同一把 wrlock, 短临界区 (~几百 ns), 跟真硬件 wire 信号传播
//   量级一致. fixture 通过 test_dev MMIO 写 (sw TEST_DEV_SET_OFF / CLEAR_OFF) 触发,
//   详 src/device/test_dev.{h,c}。
//

#ifndef PLATFORM_PLIC_H
#define PLATFORM_PLIC_H

#include <stdint.h>

int  plic_init(void);
int  plic_reset(void);
void plic_destroy(void);

int  is_plic_meip_pending(uint32_t hartid);
int  is_plic_seip_pending(uint32_t hartid);

void device_set_pending  (uint32_t source_id);
void device_clear_pending(uint32_t source_id);

#endif //PLATFORM_PLIC_H
