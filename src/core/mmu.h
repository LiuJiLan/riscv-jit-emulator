//
// Created by liujilan on 2026/4/29.
// a01_2 mmu 模块对外接口。
//
// 职责: GVA → 终结点的整体翻译层。包括:
//   - 走页表 (GVA → GPA / PA) (Sv32 模式; bare 模式恒等)
//   - 检查权限 (PTE 权限位 / PMP / PMA)
//   - 判断 RAM / ROM / MMIO
//   - LSA (load/store/AMO) 路径还要"真做访问" (RAM 用 host_ptr / MMIO 调 bus_dispatch)
//
// 当前 a01_2 范围: 只实现 mmu_translate_pc; 其它 mmu_walker_load/store/amo 留给 a01_3+ 解释器
// 接入时填。
//
// ============================================================================
// 执行 regime —— 项目内部的"两套硬件逻辑"分类 (D23 路线)
// ============================================================================
//
// 我们项目把所有运行时上下文按"权限规则是否 inline 检查 PTE 位"二分:
//
//   REGIME_BARE (Trust):
//      触发条件: priv == M, 或 任何 priv 带 satp.MODE == bare
//      含义: 没有 PTE, 没有页级权限语义; M-mode 也不查 PTE
//      实现: bypass TLB 完全 (real CPU 在 bare 下也 bypass MMU/TLB)
//            mmu_translate_pc: pa = gva (identity) + IS_GPA_RAM 检查 + pa_to_fetch_hva
//            JIT block: 取指 / load / store fast path 都直接 offset + IS_GPA_RAM 检查,
//                       不走 TLB, 不查权限位
//            interpreter: 同上
//
//   REGIME_SV32 (Checked):
//      触发条件: priv != M 且 satp.MODE == Sv32
//      含义: 有页表, 有 PTE 权限位; 必须按 priv + SUM/MXR 检查
//      实现: 走 hart->tlb_table[priv][asid] 的叶 TLB
//            mmu_translate_pc: TLB hit → 检查 PTE_X (S/U 还要 PTE_U + SUM/MXR);
//                              miss → mmu_walk 走页表 + walker fill TLB
//            JIT block: fast path 包含 PTE_R/W/X + PTE_U + SUM/MXR 检查 (compile-time
//                       baked per priv; SUM/MXR 运行时 inline 读 mstatus)
//            interpreter: 内部 switch 走对应权限规则
//
// dispatcher 的派发逻辑 (block 1): 概念上算出 (regime, current_tlb) 两件派发包 ——
// regime 决定 "用哪套 PTE 检查规则", current_tlb 决定 "走 TLB 时用哪个叶"。D25 简化
// (接口层) 后, 下游函数只吃 current_tlb (NULL 编码 regime); D25.1 修订: dispatcher 内部
// 仍把 regime 显式算出, 表达 "两个独立的派发概念, 现阶段恰好 1:1 一致" + 服务 self-check
// label + 未来 H 扩展若打破 1:1 时不需要重新引入变量。NULL 编码的可靠性: cpu_create 后
// [PRIV_M] 永远 NULL (D23), dispatcher SV32 路径 lazy alloc 后必非 NULL —— 调用方不可能
// 传出 inconsistent state。
// JIT 一侧不同: jit_cache key = (PA, regime), regime 在 JIT 块编译时 baked in 块体,
// 选块即选 regime; 那时候用 regime_t enum 显式更安全 (是 by design 的拍法分裂)。
//
// 未来扩展 (a_01 不实现, 占位):
//   - REGIME_VS (H 扩展): VS-mode 翻译 (V=1 时);
//   - misa-driven 派发: 比如 MU-only ISA 没有 S-mode, U-mode 即使 satp.MODE = Sv32 也无意义,
//     这种 ISA 下 U 直接走 BARE。当前 a_01 默认 MSU 三态, 不实现这条分支。
//
// ============================================================================
// 错误模型 —— mmu_translate_pc 和 mmu_walker_* 走两条不同路径
// ============================================================================
//
// (1) mmu_translate_pc (本文件唯一对外接口, dispatcher 直接调):
//     失败时 return 值 = RV cause code (Privileged Spec, table 3.6):
//       1  = Instruction Access Fault   (PA 落在不可执行物理区域: MMIO / 不存在内存 / PMP 拒绝)
//       12 = Instruction Page Fault     (Sv32 walker 翻译失败: PTE 无效 / X 位 / U 位等)
//     dispatcher 看 return 值, 自己调 trap_raise(hart, cause) 走 trap 路径; mmu 不 longjmp。
//
// (2) mmu_walker_load/store/amo (a01_3+ 接入, JIT block / interpreter 调):
//     失败时不通过 return 值报 cause —— 直接调 trap_raise helper, helper 内部
//     setup CSRs + siglongjmp 到 dispatcher 的 sigsetjmp landing。调用方 (jit/interp) 不
//     会真拿到 return 值, longjmp 已经跳走。
//     这一族的 cause 集 (在调 trap_raise 时由调用方传入):
//       5  = Load Access Fault          (load: PA 落在不可访问区域 / PMP)
//       7  = Store/AMO Access Fault     (store/amo: PA 落在不可写区域)
//       13 = Load Page Fault
//       15 = Store/AMO Page Fault
//
// 为什么两条路径不同: mmu_translate_pc 调用栈浅 (dispatcher 直接调, 1 层), 能优雅 return;
// mmu_walker_* 在 jit block 深栈, 没法 return, 必须 longjmp。
//
// ============================================================================
// trap_raise helper 职责 (a_03 trap.c 接入; 当前仅设计意图)
// ============================================================================
//
// trap_raise(hart, cause) 内部:
//   - 不决定 cause —— cause 由调用方传入 (mmu_walker_*, csr 写 helper, ecall, 等)
//   - 选择写哪一组 trap CSR (m{cause/tval/epc/tvec} 还是 s{cause/tval/epc/tvec})
//     由 priv 等级 + medeleg / sedeleg + 未来 H 扩展的 V 位决定
//   - 写 *cause = cause; *tval = hart->regs[0] (= GVA = 触发 fault 的 PC); *epc = hart->regs[0]
//   - push priv (mstatus.MPP / sstatus.SPP / mstatus.MIE/MPIE 等)
//   - 设 hart->regs[0] = *tvec
//   - siglongjmp 到 dispatcher sigsetjmp landing
//
// (mmu_translate_pc 不调 trap_raise, 它只 return cause; dispatcher 拿到 return 值后
//  自己调 trap_raise(hart, cause) 走相同的"setup CSR + longjmp"逻辑。这两路最终汇在
//  dispatcher 的 sigsetjmp landing 重入下一轮迭代。)
//
// ============================================================================
// PA 落 MMIO 时的行为差异
// ============================================================================
//
// 取指 vs 访存在 PA 落 MMIO 时行为不同 (RV 规范一般约定取指不能从 MMIO 拿):
//
//   mmu_translate_pc:
//     PA 落 MMIO → 直接 access fault (return 1)。取指不能 bus_dispatch。
//     未来 ROM 接入: ROM 可执行 (取指合法), 在 pa_to_fetch_hva 内加分支即可。
//
//   mmu_walker_load/store/amo:
//     PA 落 MMIO → 分发 bus_dispatch (load/store/amo 可走 MMIO)。
//     PA 落 ROM 写时 → trap_raise(hart, 7) 走 access fault; 读 ROM → 直接 host_ptr。
//
// ============================================================================

#ifndef CORE_MMU_H
#define CORE_MMU_H

#include <stdint.h>

#include "cpu.h"
#include "tlb.h"

// ----------------------------------------------------------------------------
// regime_t —— 执行 regime 的 concept-level 命名 (D25 后不作为函数参数)
//
// 当前 mmu_translate_pc / interpret_one_block 接口不真吃 regime_t 参数, 而是用
// current_tlb 的 NULL/非 NULL 编码 (NULL = REGIME_BARE; 非 NULL = REGIME_SV32)。
//
// 设计依据:
//   - cpu_create 后 hart->tlb_table[PRIV_M] 永远 NULL (D23 路线, Trust 不走 TLB)
//   - dispatcher 在 SV32 路径上 lazy alloc 后必传非 NULL (file_plan §1.dispatcher)
//   两条状态不可伪造, 少一个参数 = 少一个 inconsistent state 的 bug 面。
//
// regime_t enum 保留, 用途仅限于:
//   - 模块 doc / 注释里讨论时给概念命名 (写 "REGIME_BARE 路径" 比 "current_tlb == NULL
//     路径" 表意清楚)
//   - dispatcher 临时 self-check 输出的 regime label 字符串
//   - 未来 H 扩展真上时若需要 3 状态 (BARE / SV32 / VS), 再考虑回到显式参数
//     (NULL 编码不够 3 状态; 那时是大手术, 加参数代价小)
//
// 见上方"执行 regime"段对两个值的语义说明。
// ----------------------------------------------------------------------------
typedef enum {
    REGIME_BARE = 0,    // Trust: M-mode 或任何 priv 带 bare satp; bypass TLB, identity
    REGIME_SV32 = 1,    // Checked: S/U + Sv32 satp; via TLB + walker + perm check
    // 未来 H 扩展: REGIME_VS = 2 (VS-mode, V=1 时)
} regime_t;


// ----------------------------------------------------------------------------
// mmu_translate_pc —— dispatcher 取指路径的 PC 翻译
//
// 流程按 regime 分两路:
//
//   REGIME_BARE (Trust):
//     pa = gva (identity)
//     pa_to_fetch_hva(pa, &hva): RAM ✓ / 未来 ROM ✓ / 其它 → return 1 (access fault)
//     不走 TLB, current_tlb 参数被忽略 (调用方可传 NULL)
//
//   REGIME_SV32 (Checked):
//     1. 试图命中 current_tlb (V 位 + tag 比对; 命中再 check X 位):
//          命中 + X = 1 → 返回 PA + HVA (PA 由 HVA 反推, RAM-only TLB 缓存让 sub 一定有效)
//          命中 + X = 0 → return 12 (page fault)
//          未命中 → 走 step 2
//     2. miss → mmu_walk(hart, gva, PERM_X, &pa, ...) 走页表 (a_01 占位 return 12)
//     3. PMP / PMA 物理侧检查 (a_01 全开 skip)
//     4. PA → HVA via pa_to_fetch_hva (RAM ✓, 未来 ROM ✓, 其它 → return 1)
//     5. TLB insert (walker 路径填; entry 内容从 PTE 实际位拷贝)
//
// a_01 实际行为: priv 恒为 PRIV_M, satp = bare → 永远走 REGIME_BARE 分支。
// REGIME_SV32 路径全是占位, 等 Sv32 walker 接入 (a_05+) 真实现。
//
// 参数:
//   hart        — 调用 hart (内部读 hart->regs[0] 作为 gva = pc)
//   current_tlb — NULL = REGIME_BARE (Trust 不走 TLB);
//                 非 NULL = REGIME_SV32, 必须为 dispatcher 选定的叶 TLB
//                          (= hart->tlb_table[priv][asid], dispatcher lazy alloc 后保证非 NULL)
//   pa_out      — 出参; 成功时填 PA (供 dispatcher 查 jit_cache)
//   hva_out     — 出参; 成功时填 HVA (供 dispatcher 传 interp_one_block, 解释器直接取字节)
//
// 返回值 (RV cause code):
//   0  = OK,  pa_out / hva_out 已填
//   1  = Instruction Access Fault — PA 落不可执行物理区 (MMIO / 不存在内存 / 未来 PMP 拒绝)
//   12 = Instruction Page Fault   — Sv32 walker 翻译失败 (a_01 不会触发, 接口预留)
//
// 失败时不填 pa_out / hva_out。失败原因 mtval 应该是触发 fault 的 GVA = hart->regs[0],
// 由 dispatcher 自己填 (mmu 不知道 cpu_t 里 mtval 字段在哪)。
//
// ============================================================================
// dispatcher 的预期使用形态 (示意; 简化形态见 dispatcher.c)
// ============================================================================
//
//   /* dispatcher block 1: 算 (regime, current_tlb) 派发包 (D25 后下游只吃 current_tlb;
//      D25.1 修订后 dispatcher 内部仍持 regime 本地变量) */
//   regime_t regime;
//   tlb_t   *current_tlb;
//   if (hart->priv == PRIV_M || (hart->satp >> 31) == 0) {
//       regime      = REGIME_BARE;
//       current_tlb = NULL;        /* Trust regime: bypass TLB */
//   } else {
//       regime      = REGIME_SV32;
//       uint32_t asid = (hart->satp >> 22) & ASID_MASK;
//       current_tlb = hart->tlb_table[hart->priv][asid];
//       /* future lazy alloc here */
//   }
//
//   uint32_t pa;
//   uint8_t *hva;
//   int cause = mmu_translate_pc(hart, current_tlb, &pa, &hva);  /* 只 pass current_tlb */
//   if (cause != 0) {
//       /* 未来 cpu_t 加 in_trap 字段后 (仅注释):    */
//       /* if (hart->in_trap) { fprintf("Double Fault"); exit(2); }     */
//       /* hart->in_trap = 1;                                            */
//       /* trap_raise(hart, cause); 内部:                                */
//       /*   - 选 m/s 那组 cause/tval/epc/tvec CSR (priv + medeleg/sedeleg) */
//       /*   - 写 *cause = cause; *tval = hart->regs[0]; *epc = hart->regs[0] */
//       /*   - push priv; 设 hart->regs[0] = *tvec                       */
//       continue;  /* 回 dispatcher loop top, sigsetjmp 重落 */
//   }
//   /* OK: dispatcher 把 pa 用于 jit_cache 查找; hva 传给 interp / JIT 块 */
//
// 注: cpu_t 当前没有 in_trap / mcause / mtval / mepc / mtvec / mstatus / scause / stval /
//     sepc / stvec / medeleg / sedeleg / sstatus 等字段 —— 这些 a_03 trap.c 接入 + a01_5
//     dispatcher 真激活时一并加。 全部仅注释示意, 不真改 cpu_t。
//
// ----------------------------------------------------------------------------
int mmu_translate_pc(cpu_t *hart, tlb_t *current_tlb,
                     uint32_t *pa_out, uint8_t **hva_out);

#endif //CORE_MMU_H
