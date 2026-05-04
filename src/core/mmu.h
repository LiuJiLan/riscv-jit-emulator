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
// 错误模型 —— mmu_translate_pc 和 mmu_walker_* 都对接 trap, 但路径不同
// ============================================================================
//
// 跨文件协议见 src/dummy.txt §1 (sigsetjmp / 大入口 vs 直调辅助函数)。两条路径:
//
// (1) mmu_translate_pc (本文件唯一对外接口, dispatcher 直接调; dummy.txt §1 路径 2b):
//     失败时**内部直调 trap_set_state(hart, cause, tval)** 设架构状态 (写 xcause/xtval/xepc/
//     regs[0]=xtvec[deliver_priv]), 不长跳 (调用栈浅, 直接 return 给 dispatcher 接管即可)。
//     return 值仅是 0/非0 的状态信号:
//       0  = fetch OK, pa_out / hva_out 已填, dispatcher 进 block 3
//       非0 = trap 已 deliver, dispatcher continue 让 while(in_trap < 3) 接管退出判断
//     fetch 失败的 cause 集 (mmu_translate_pc 内部填给 trap_set_state):
//       1  = Instruction Access Fault   (PA 落在不可执行物理区域: MMIO / 不存在内存 / PMP 拒绝)
//       12 = Instruction Page Fault     (Sv32 walker 翻译失败: PTE 无效 / X 位 / U 位等)
//     tval = 触发 fault 的 GVA = hart->regs[0] (a_01_5_b 起 mmu 自己填, dispatcher 不再填)。
//
// (2) mmu_walker_load/store/amo (a_01_6+ 接入, JIT block / interpreter 调; dummy.txt §1 路径 2a):
//     失败时调 **trap_raise_exception** (含长跳); helper 内部 trap_set_state + siglongjmp 到
//     dispatcher 的 sigsetjmp landing (a_01_5_c 起真接通)。调用方 (jit/interp) 不会真拿到
//     return 值 (longjmp 已跳走)。
//     这一族的 cause 集 (调 trap_raise_exception 时由调用方传入):
//       5  = Load Access Fault          (load: PA 落在不可访问区域 / PMP)
//       7  = Store/AMO Access Fault     (store/amo: PA 落在不可写区域)
//       13 = Load Page Fault
//       15 = Store/AMO Page Fault
//
// 两条路径调用同一个 trap_set_state 内核 (dummy.txt §1 (2)), 区别只是"控制流如何回到
// dispatcher": mmu_translate_pc 直接 return + continue, mmu_walker_* 经 longjmp。
//
// ============================================================================
// trap helper 职责 (a_01_5_b 实接, trap.c)
// ============================================================================
//
// trap.h 暴露两层 helper:
//
//   trap_set_state(hart, cause, tval) —— 架构语义层 (不长跳)
//     in_trap++; >= 3 早 return (候选 A: 不 deliver, 字段保留第二次状态作 root cause)
//     否则: 选 deliver_priv (a_01_5_b v0 = PRIV_M; 未来 mideleg/medeleg-driven)
//           写 xcause/xtval/xepc[deliver_priv]; regs[0] = xtvec[deliver_priv]; (TODO: 切 priv)
//     返回 in_trap 当前值 (mmu_translate_pc 透传给 dispatcher 当 0/非0 状态信号)。
//
//   trap_raise_exception(hart, cause, tval) —— interpreter / JIT 长跳入口
//     a_01_5_b: 调 trap_set_state + 普通 return (caller goto out)
//     a_01_5_c: 标 _Noreturn, 调 trap_set_state + siglongjmp(*hart->jmp_buf_ptr, 1)
//
// mmu_translate_pc 调路径 (1) trap_set_state, dispatcher 通过 return rc 接管;
// mmu_walker_* / interpreter case 调路径 (2) trap_raise_exception, 经 longjmp 跳回 dispatcher
// (a_01_5_c 起真激活)。两路最终都靠 dispatcher 的 while(in_trap < 3) 退出判断。
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
// 返回值 (a_01_5_b 起改成 0/非0 状态, 不再返回 cause):
//   0   = OK,  pa_out / hva_out 已填, dispatcher 进 block 3
//   非0 = trap 已 deliver (内部已调 trap_set_state 写 xcause/xtval/xepc/regs[0]=xtvec),
//          dispatcher continue 让 while(in_trap < 3) 接管退出判断
//
// 失败时不填 pa_out / hva_out (不需要, dispatcher continue 跳过本轮 block; 下一轮 fetch
// 从 xtvec 开始)。
//
// 不再需要 dispatcher 自己填 mtval — mmu_translate_pc 自己调 trap_set_state 时已经填好
// (cause/tval/epc/regs[0] 都设)。
//
// ============================================================================
// dispatcher 的预期使用形态 (a_01_5_b 实际形态, 见 dispatcher.c)
// ============================================================================
//
//   while (hart->trap.in_trap < 3) {
//       /* block 1: regime / current_tlb */
//       ...
//       int rc = mmu_translate_pc(hart, current_tlb, &pa, &hva);
//       if (rc != 0) continue;     /* trap 已 deliver, regs[0] 已是 xtvec, continue */
//
//       /* block 3: interpret / JIT */
//       interpret_one_block(hart, current_tlb, hva, &local_count);
//   }
//   /* dispatcher 退出: in_trap == 3, main 端 fprintf 表 halt + 未来 reset */
//
// ----------------------------------------------------------------------------
int mmu_translate_pc(cpu_t *hart, tlb_t *current_tlb,
                     uint32_t *pa_out, uint8_t **hva_out);

#endif //CORE_MMU_H
