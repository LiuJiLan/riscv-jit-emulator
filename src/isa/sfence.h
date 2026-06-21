//
// isa/sfence —— RV32 SFENCE.VMA 指令实现 (TLB 失效)。
//
// helper 风格 (跟 isa/lsu.{c,h} 一致):
//   - lsu_load_helper / lsu_store_helper: inline 顶层在 lsu.h (BARE/SV32 分流)
//   - store_helper: extern 在 lsu.c (HVA-based, RAM 写 + LR/SC + SMC 副作用入口)
//   - sfence_vma_helper: extern 在本文件 sfence.c — slow path (改 TLB 状态, 块边界,
//                          频次低 OS 级 us 量级, 不需 inline)
//
// trap 协议 (sfence.vma 当前不抛 trap):
//   未来 mstatus.TVM=1 + S-mode csrw satp / sfence.vma → trap cause 2; 真做 OS 隔离时
//   在本 helper 入口 (或 csr_op / interpreter case 入口) 加检查, 直接调 trap_raise_exception
//   长跳 — 不改 helper 签名。当前不实现该检查。
//
// sfence_vma_helper 不调 jit_invalidate_block — 为什么 (b_02 T5 重审, 推翻
// start_plan_b_02 §[2] T5 第 5 点):
//
//   start_plan_b_02 原拍 "sfence_vma_helper 调 jit_invalidate_block" 是过时拍法,
//   基于"JIT 块 baked GVA → PA 映射 / PTE 数值"假设. 实际项目设计:
//
//   1. jit_cache key=(PA, regime) — VMA 改了 PA 不变, JIT 块本身不动
//   2. JIT 块内 perm 检查走 inline TLB compare + walker_helper, PTE perm 从 TLB
//      entry runtime 拿, **不 baked 数值** (T6 SV32 baked 的是"S/U regime 检查逻辑
//      序列", 不是某个 PA 的 PTE_U 值; SUM/MXR 也是 runtime 读 mstatus)
//   3. sfence.vma 清 TLB 后, JIT 块下次 load/store 走 walker_helper 自然适配新页表
//
//   JIT cache invalidate 的真正语义边界是 SMC handler (page_dirty bitmap), 不是
//   sfence.vma. fence.h 顶段注释明确"fence.i 跟 sfence.vma 语义不同族, 三者无共享
//   helper". SMC 路径 b_03 T1.a 已落地 (SIGSEGV + mprotect + page_dirty bitmap;
//   详 jit/smc.h + plan §1.17); fence.i 由 audit Q4.2.1+ 拍 fence_i_helper 不主动
//   invalidate, 走块边界 + SMC chain 间接 (详 fence.h 顶段 doc).
//
//   推翻 trail: b_02_session_006 chat 重审 + memory `feedback_overturn_plan_leave_trail`
//   (推翻 plan 留代码注释 trail). 未来开发若再质疑"sfence.vma 是否该调 jit invalidate",
//   先读本段.
//

#ifndef ISA_SFENCE_H
#define ISA_SFENCE_H

#include <stdint.h>

#include "core/cpu.h"        // cpu_t
#include "riscv.h"           // uxlen_t (typedef family; dummy.txt §13)


// ----------------------------------------------------------------------------
// sfence_vma_helper —— 实现 RV spec sfence.vma 的简化方案 4.a
//   (sfence 不是 hot path, 过度刷新不亏)
//
// RV spec 四组合 (RV Privileged Spec Vol II §10.6.1) vs 简化方案对照:
//   (a) rs1=x0, rs2=x0   → 全清所有 ASID 全所有 vaddr   [简化方案: 全清 ✓ 精确等同]
//   (b) rs1!=x0, rs2=x0  → 清指定 vaddr 跨所有 ASID    [简化方案: 全清 = 过度刷新]
//   (c) rs1=x0, rs2!=x0  → 清指定 ASID 全所有 vaddr    [简化方案: 单 ASID 清 ✓ 精确等同]
//   (d) rs1!=x0, rs2!=x0 → 清指定 vaddr + 指定 ASID    [简化方案: 全清 = 过度刷新]
//
// 各组合在新 TLB 设计 (tlb_table[priv][asid] 二级索引, asid 是外层下标) 下的成本:
//   (a) 全清:           ASID_MAX (16) 次 tlb_clear (memset 1KB 每槽) — 1 行循环
//   (b) 单 vaddr 跨 ASID: 16 次 cache line 写 (vpn → index, 各 ASID 槽 e[index] 看 tag
//                          命中就清); 中等成本
//   (c) 单 ASID 全清:    1 次 tlb_clear (memset 1KB) — 最便宜
//   (d) 单 vaddr + ASID: 1 次 cache line 写 — 最精细但场景特化
//
// 简化方案 4.a 把 (b)/(d) 退化为 (a) 全清 — RV spec 允许过度刷新 (over-flush is correct,
// just slower); sfence 不是 hot path (OS 级操作 us 量级, < 1us); 实现成本最低不亏。
// 未来真上 OS 实测发现过度刷新是瓶颈再细化; 不入 plan §2 改进列表 (本模块自描述)。
//
// 涉及哪些 tlb_table[] 槽:
//   [0] U   = 副本于 [1] S, 不重复清 (避免 double clear 同一份)
//   [1] S   = sfence 主目标
//   [2] H/VS = 当前永远 NULL, 留循环框架方便未来 H 扩展激活 (tlb_clear(NULL) no-op)
//   [3] M   = 永远 NULL (Trust regime bypass TLB), RV spec sfence.vma 也不影响 M-mode
//             TLB, 不动
//
// 入参分两组, 语义独立, **不能互相推导**:
//
//   ----- 组 1: 寄存器值 (32-bit, READ_REG 处理 x0 编码后的真值) -----
//   vaddr_val  - = READ_REG(rs1); 真要清的 vaddr 数值。4.a 简化下 helper 不读 (b/d 都
//                  退化为全清, 不需要 vaddr 维度); 接口保留参数为未来精确实现 (b)/(d) 时
//                  启用 (helper 内 (void)vaddr_val 抑制 unused)。
//   asid_val   - = READ_REG(rs2); 真要清的 asid 数值。rs2!=0 时是 asid 寄存器值, rs2=0
//                  时 caller 用 READ_REG(x0)=0 传 0; helper 内 & ASID_MASK 截断 (跟 satp 路径
//                  同, dummy.txt §3 风格)。
//
//   ----- 组 2: 寄存器编号 (5-bit 0..31, 即 d.rs1 / d.rs2 字段值) -----
//   rs1, rs2   - 寄存器编号本身; helper 用 (rs1==0, rs2==0) 路由 RV spec 4 组合。
//                  RV spec sfence.vma 用 "rs1=x0" 这个 **编码** 标记 "忽略 vaddr, 覆盖所有
//                  vaddr"; rs2=x0 同理 "忽略 asid"。这是 magic encoding, **不是值的语义**。
//
//   ----- 为什么必须同时传 (vaddr_val, rs1) 两件信息, 不能单传 vaddr_val -----
//   vaddr_val=0 跟 rs1=x0 不是同一件事:
//     - d.rs1 = x0 (寄存器号 0) → vaddr_val=READ_REG(x0)=0; 含义 = "忽略 vaddr, 清所有"
//     - d.rs1 = x5 (寄存器号 5) + x5 寄存器值=0 → vaddr_val=0; 含义 = "清 vaddr=0 这页 (合法非忽略)"
//   helper 只看 vaddr_val 看不出 (1)/(2) 区别, 必须看 rs1 寄存器编号才能判 x0 编码。
//   asid 路径同理 (asid_val=0 ≠ rs2=x0)。
//
//   hart       - 调用 hart
//
// trap 协议: 当前不抛 trap (TVM 检查未实现, 见文件顶部); 接口签名不预留 trap 出参。
// ----------------------------------------------------------------------------
void sfence_vma_helper(cpu_t *hart,
                       uxlen_t  vaddr_val, uint32_t asid_val,
                       uint32_t rs1,       uint32_t rs2);

#endif //ISA_SFENCE_H
