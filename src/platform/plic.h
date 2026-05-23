//
// Created by liujilan on 2026/5/23.
// PLIC (Platform-Level Interrupt Controller) 模块对外接口。
//
// 职责: per-source 中断状态维护 + ctx-级仲裁 + bus MMIO 暴露给 guest。
//        T1 阶段只完成骨架 (MMIO 地址解码 + lifecycle), 真仲裁 / 外设接通 / hart 侧
//        合成读 (csr_mip_read MEIP/SEIP 两支) 留 T2。
//
// 地址布局 / 寄存器形态 见 config.h PLIC_* 宏 (跟 QEMU virt + RV PLIC spec v1.0.0
// 一致)。接口形态 (read/write 返 cause / 0=成功) 见 platform/bus.h + dummy.txt §9。
//
// ----------------------------------------------------------------------------
// monitor 模型 (dummy.txt §7) — PLIC 是 "monitor 但无线程" 区分的第一个完整范例
// ----------------------------------------------------------------------------
//
// CLINT = "monitor + timer 辅助线程" (mtime 由 host wall clock 推进, 需要后台线程);
// PLIC  = "monitor 但无线程"        (所有路径都是外设 device_set/clear_pending 或
//                                    hart MMIO 触发的同步调用, 无后台推进需求)。
//
// 所以 dummy.txt §12 "谁 spawn 谁 join" 协议**不适用**于 PLIC — 无 plic_start_* /
// plic_join_* 对偶函数。三函数 lifecycle (init/reset/destroy) 完整。
//
// ----------------------------------------------------------------------------
// 字段模型 vs RV PLIC spec 字段 — 命名选择说明
// ----------------------------------------------------------------------------
//
// RV PLIC spec (v1.0.0 §5 Interrupt Gateways) 是三态机:
//
//   spec device line          — 设备到 PLIC 的物理信号 (level 或 edge), 设备侧拥有
//   spec IP bit               — gateway 输出, "该 source 有未处理 interrupt"; claim 时 clear
//   spec gateway forward latch — gateway 内部锁存, claim 时 set, complete 时 clear;
//                                锁住期间 IP 不被新 forward 设置 (spec 通用规则:
//                                任一 source 任一时刻 PLIC core 最多 1 pending request)
//
// 我们简化建模 (per source 两字段, 不存 IP bit):
//
//   device_line  ≈ spec device line 镜像 (设备调 device_set/clear_pending 直接拉高/拉低)
//   claimed      ≈ spec gateway forward latch (0=未 claim; 非 0 = ctx_id + 1)
//   "送 hart 与否" = device_line && !claimed 合成 view (不进字段, 跟 csr_mip_read 同哲学)
//
// 一字段 (claimed) 顶 spec 里 "IP bit + gateway latch" 两态; 在 level-triggered
// 正确 driver 序列下 (claim → 处理设备 → 设备拉低 device line → complete) 跟 spec
// 行为零差异 (forward latch 是 spec 通用规则, 不分 level/edge — 我们的 claimed
// 等价它)。edge-triggered 未来要支持时, 加 source-side latch 字段单独存即可,
// 不冲突现模型。
//
// 命名理由: spec IP bit 是"是否送 hart 的合成结果", 跟原始线电平不同义, 故不取
// `ip` 字段名避免误导。`device_line` 突出"设备侧线电平"物理语义; `claimed` 突出
// "已被某 ctx 取走等 complete"状态语义。`pending` 留给外部 is_plic_*_pending(hartid)
// 接口 (ctx 级合成 view), 跟内部 source 级字段名错开。
//
// ----------------------------------------------------------------------------
// 三函数 lifecycle
// ----------------------------------------------------------------------------
//
// plic_init    — POR 一次性 (main 入口, clint_init 之后): 字段 0 init + plic_ctx_map
//                全 -1 + bus 注册。失败 -1 (跟 ram_init / clint_init 同形态; dummy.txt §5)。
//                **不发线程** (跟 clint_init 区分)。
//
// plic_reset   — system reset 每 iter (main while 顶段): 清 device_line / claimed;
//                plic_ctx_map 不动 (跟 mtime 不动同性质 — 未来设备树 init 阶段填一次后
//                不动)。
//
// plic_destroy — POR 收尾 (main 末段): 纯模块 cleanup, 跟 clint_destroy / cpu_destroy
//                同 — 函数留作 lifecycle 对称, 真工作量极小 (bus 未来加 unregister 时
//                这里调)。
//

#ifndef PLATFORM_PLIC_H
#define PLATFORM_PLIC_H

int  plic_init(void);
int  plic_reset(void);
void plic_destroy(void);

// ----------------------------------------------------------------------------
// T2 占位 — 不在 T1 暴露, 留 T2 加 (此处仅作 forward reference, 实际声明 T2 加):
//
//   int  is_plic_meip_pending(uint32_t hartid);
//   int  is_plic_seip_pending(uint32_t hartid);
//   void device_set_pending  (uint32_t source_id);
//   void device_clear_pending(uint32_t source_id);
//
// T2 接通后: csr.c csr_mip_read 把 L285-287 占位转实装调用 is_plic_meip/seip_pending;
// UART 用 device_set/clear_pending 通知 PLIC 自己内部状态机决定 device_line。
// ----------------------------------------------------------------------------

#endif //PLATFORM_PLIC_H
