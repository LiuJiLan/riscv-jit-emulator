//
// Created by liujilan on 2026/4/29.
// mmu 模块对外接口。
//
// 职责 (回归 dummy.txt §8 三层模型):
//   - 走页表 (GVA → GPA / PA) (Sv32 模式; bare 模式恒等)
//   - 检查权限 (PTE 权限位 / PMP / PMA)
//   - 在 PA 落 RAM 还是 MMIO 之间分流, 但**只决定下一步去哪**, 不亲自做"真访问":
//     - RAM 路径: walker fill TLB + 调 RAM 直接 *hva (load) / 调 store_helper (store)
//     - MMIO 路径: 直接调 mmio_read/write_helper (不入 TLB; 走 bus 派发)
//
// mmu 不亲自做 "host_ptr memcpy / 内嵌 RAM 写", 而是调出 lsu 层的 store_helper / 调出
// bus 层的 mmio_*_helper, 跟 §8 三层模型一致。详 src/isa/lsu.h 顶段"调用拓扑"图。
//
// ============================================================================
// helper 颗粒度 / may-longjmp 边界协议
// ============================================================================
//
// mmu_walker_helper_load / store + mmu_walker_helper_amo_* (9 个 Zaamo)
// + mmu_walker_helper_lr_w / sc_w (Zalrsc) 这一族函数,
// **不只是 TLB miss 时的 fallback**, 而是 by design 给 JIT 块出口准备的"不同
// 颗粒度的助手函数" — 每条访问指令对应一个 helper 入口, JIT 不合并成统一大
// 入口。完整协议见 dummy.txt §10。
//
// ============================================================================
// misalign check 隐式契约
// ============================================================================
//
// misalign (gva & (size-1) != 0) 由 caller (interpreter case 入口
// LOAD/STORE/AMO_MISALIGN_CHECK 宏 / 未来 JIT translator emit 段) 已查; 本文件所有
// helper (mmu_walker_helper_load/store + mmu_walker_helper_amo_*) 都信任 caller 已查,
// 不重复。详 src/isa/lsu.h 顶段 + src/isa/amo.h 顶段。
//
// ============================================================================
// 执行 regime —— 项目内部的"两套硬件逻辑"分类
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
// regime 决定 "用哪套 PTE 检查规则", current_tlb 决定 "走 TLB 时用哪个叶"。接口层简化:
// 下游函数只吃 current_tlb (NULL 编码 regime); dispatcher 内部仍把 regime 显式算出, 表达
// "两个独立的派发概念, 现阶段恰好 1:1 一致" + 未来 H 扩展若打破 1:1 时不需要重新引入变量。
// NULL 编码的可靠性: cpu_create 后 [PRIV_M] 永远 NULL (Trust 不走 TLB), dispatcher SV32
// 路径 lazy alloc 后必非 NULL —— 调用方不可能传出 inconsistent state。
// JIT 一侧不同: jit_cache key = (PA, regime), regime 在 JIT 块编译时 baked in 块体,
// 选块即选 regime; 那时候用 regime_t enum 显式更安全 (是 by design 的拍法分裂)。
//
// 未来扩展 (占位):
//   - REGIME_VS (H 扩展): VS-mode 翻译 (V=1 时); cpu_create 阶段已按 misa 派发 [PRIV_U]
//     副本, dispatcher 这里只看 priv + xatp.mode 选 regime, 不需要 misa 分支 (cpu.c
//     [PRIV_U] 段)。当前默认 MSU 三态。
//
// ============================================================================
// 错误模型 —— mmu_translate_pc 和 mmu_walker_* 都对接 trap, 但路径不同
// ============================================================================
//
// 跨文件协议见 src/dummy.txt §1 (sigsetjmp / 大入口 vs 直调辅助函数)。两条路径:
//
// (1) mmu_translate_pc (本文件唯一对外接口, dispatcher 直接调; dummy.txt §1 路径 2b):
//     失败时**内部直调 trap_set_exception_state(hart, cause, tval)** 设架构状态 (写 xcause/xtval/xepc/
//     regs[0]=xtvec[deliver_priv]), 不长跳 (调用栈浅, 直接 return 给 dispatcher 接管即可)。
//     return 值仅是 0/非0 的状态信号:
//       0  = fetch OK, pa_out / hva_out 已填, dispatcher 进 block 3
//       非0 = trap 已 deliver (或 M-mode critical-error 已 set HART_MDT),
//             dispatcher continue 让 while(system_reset_signal == 0) 接管退出判断
//     fetch 失败的 cause 集 (mmu_translate_pc 内部填给 trap_set_exception_state):
//       1  = Instruction Access Fault   (PA 落在不可执行物理区域: MMIO / 不存在内存 / PMP 拒绝)
//       12 = Instruction Page Fault     (Sv32 walker 翻译失败: PTE 无效 / X 位 / U 位等)
//     tval = 触发 fault 的 GVA = hart->regs[0] (mmu 自己填, dispatcher 不再填)。
//     注: 返回值 0 = 成功跟 RV cause 0 (CAUSE_INST_ADDR_MISALIGNED) 不冲突 —
//         cause 0 由 dispatcher 循环顶 + 转跳指令自检产生, 不经 mmu 接口的 return
//         code。详 dummy.txt §9 ("0=成功" 接口约定)。
//
// (2) mmu_walker_load/store/amo (JIT block / interpreter 调; dummy.txt §1 路径 2a):
//     失败时调 **trap_raise_exception** (含长跳); helper 内部 trap_set_exception_state + siglongjmp 到
//     dispatcher 的 sigsetjmp landing。调用方 (jit/interp) 不会真拿到 return 值 (longjmp
//     已跳走)。
//     这一族的 cause 集 (调 trap_raise_exception 时由调用方传入):
//       5  = Load Access Fault          (load: PA 落在不可访问区域 / PMP)
//       7  = Store/AMO Access Fault     (store/amo: PA 落在不可写区域)
//       13 = Load Page Fault
//       15 = Store/AMO Page Fault
//
// 两条路径调用同一个 trap_set_exception_state 内核 (dummy.txt §1 (2)), 区别只是"控制流如何回到
// dispatcher": mmu_translate_pc 直接 return + continue, mmu_walker_* 经 longjmp。
//
// ============================================================================
// trap helper 职责 (trap.c)
// ============================================================================
//
// trap.h 暴露两层 helper:
//
//   trap_set_exception_state(hart, cause, tval) —— 架构语义层 (不长跳)
//     算 deliver_priv (medeleg-driven; M-mode caller 总 M, 否则按 medeleg.bit(cause));
//     检 sstatus.SDT (S-trap 时 SDT=1 → 升级 M cause=DOUBLE_TRAP mtval2=原 tval) +
//     检 mstatus.MDT (M-trap 时 MDT=1 → critical-error: set HART_MDT, return 1);
//     正常 deliver: 写 xcause/xtval/xepc[deliver_priv]; 切 priv; set 对应 xDT=1;
//     regs[0] = xtvec[deliver_priv]。返非 0 = "已设状态, dispatcher continue 接管"。
//
//   trap_raise_exception(hart, cause, tval) —— interpreter / JIT 长跳入口
//     _Noreturn, 内部 trap_set_exception_state + siglongjmp(*hart->jmp_buf_ptr, 1) 跳回 dispatcher
//     一次性 sigsetjmp 落点。
//
// mmu_translate_pc 调路径 (1) trap_set_exception_state, dispatcher 通过 return rc 接管;
// mmu_walker_* / interpreter case 调路径 (2) trap_raise_exception, 经 longjmp 跳回 dispatcher。
// 两路最终都靠 dispatcher 的 while(system_reset_signal == 0) 退出判断。
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
//     PA 落 MMIO → 分发 mmio_*_helper (load/store/amo 可走 MMIO)。
//     PA 落 ROM 写时 → trap_raise(hart, 7) 走 access fault; 读 ROM → 直接 host_ptr。
//
// ============================================================================
// SV32 + MMIO 性能特征
// ============================================================================
//
// SV32 模式下访问 MMIO 物理地址, 每次都要 **重新走 PT** —— 因为 MMIO 不进 TLB
// ("MMIO 不进 TLB" 让 TLB 命中路径结构上不带 RAM/MMIO 分支; 详 dummy.txt §1 末段
// + §4 + §8)。
//
// 单次访问代价:
//   1. mmu_walk 两级 PT walk (~100 cycle, level=1 + level=0; 4MB superpage 时 ~50)
//   2. PTE.A/D 写回 PT (RAM 路径才写; caller 端 atomic_fetch_or 4 字节; MMIO 路径不写)
//   3. mmio_*_helper 内 bus 线性扫表 + device fn 调用 (~50 cycle, device 数量 N=2~5)
//
// 设计权衡:
//   - 收益: TLB fast path 路径结构上不带分支 (load 命中可 *hva 直接), 每条 RAM
//          load 命中 < 5 cycle; RAM 访问占总访问 > 99%, 99% 路径快得多
//   - 代价: MMIO 访问 SV32 模式下每次重 walk, ~150-200 cycle/次
//   - MMIO 访问占比 < 1% (典型 OS profile: 内存访问 vs MMIO ≈ 100:1), 总体 net win
//   - BARE 模式下没 walk 代价 (identity); SV32+MMIO 重 walk 仅影响 OS 跑 MMIO 段
//
// 未来不计划优化 (改进列表也不收): MMIO 进 TLB / TLB 加 is_mmio flag / 把 MMIO
// fast path 单独优化 —— 都会破坏 "TLB 命中路径不带分支" 这一根性, 不值得。
//
// ============================================================================

#ifndef CORE_MMU_H
#define CORE_MMU_H

#include <stdint.h>

#include "cpu.h"
#include "tlb.h"
#include "riscv.h"   // PTE_R/W/X/U / MSTATUS_SUM/MXR / PRIV_U/S/M (check_perm static inline 用)

// ----------------------------------------------------------------------------
// regime_t —— 执行 regime 的 concept-level 命名 (不作为函数参数)
//
// 当前 mmu_translate_pc / interpret_one_block 接口不真吃 regime_t 参数, 而是用
// current_tlb 的 NULL/非 NULL 编码 (NULL = REGIME_BARE; 非 NULL = REGIME_SV32)。
//
// 设计依据:
//   - cpu_create 后 hart->tlb_table[PRIV_M] 永远 NULL (Trust 不走 TLB)
//   - dispatcher 在 SV32 路径上 lazy alloc 后必传非 NULL (dispatcher.c block 1)
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
// mmu_perm_t —— 访问类型枚举 (给 mmu_walk + walker helpers 用)
//
// 跟 PTE 位 mask (PTE_R/W/X) 是两件事: PTE 位是"PTE 内字段值"; mmu_perm_t 是"访问意图",
// walker 内按它分支:
//   - 选 fault cause: load → 13, store → 15, fetch → 12
//   - 决定 set A/D: R/X 仅 set A; W set A+D
//   - SUM 处理: 仅 R/W 路径享 (S+SUM=1 允许 R/W on U-page; X 不享)
//   - MXR 处理: 仅 R 路径享 (X=1 page 当 readable; W/X 不享)
//   - W=1 + R=0 reserved 检查: 仅 W 路径关心
//
// 用 enum + -Wswitch-enum 联动, 加新 perm 类型 (例如未来 atomic R+W 一次) 时编译器
// 强制 walker switch 补 case (跟项目 op_kind_t / csr_op_t 风格一致)。
//
// 为什么不用 PTE 位 mask 当 perm 参数:
//   即使 caller 传 PTE_W=0x04, walker 内还是要 if (perm == PTE_W) 决定 cause/A_D/SUM/MXR
//   分支; 用 enum 把"接口语义" 跟"PTE 位编码" 解耦, 接口干净。
// ----------------------------------------------------------------------------
typedef enum {
    MMU_PERM_R = 0,    // load 访问
    MMU_PERM_W = 1,    // store / AMO 访问
    MMU_PERM_X = 2,    // fetch 访问 (mmu_translate_pc SV32 路径用)
} mmu_perm_t;


// ----------------------------------------------------------------------------
// check_perm —— priv + SUM/MXR + PTE.U/R/W/X 权限检查 (static inline, 三处共用)
//
// 三处调用方:
//   1. mmu_translate_pc (mmu.c) — TLB hit 后调 check_perm(MMU_PERM_X);
//      per-block 1 次 slow path, inline 与否性能差别小, 调用形态 OK
//   2. lsu_load_helper (lsu.h, inline 顶层) — SV32 TLB hit fast path 命中条件内联调
//      check_perm(MMU_PERM_R); inline 形态消除函数 call 开销 (每条 lw/lh/lb 都跑)
//   3. lsu_store_helper (lsu.h, inline 顶层) — SV32 TLB hit fast path 命中条件内联调
//      check_perm(MMU_PERM_W); inline 形态消除函数 call 开销 (跟 load 对称)
//   附: mmu_walk 内 walker 自身也调 check_perm(PERM) (mmu_walker_helper_* 复用 walker)
//
// 三处共用一份 check_perm 实现 → 跟 walker (mmu_walk) 内的 perm 检查同源, 不会两份
// 维护偏离。
//
// 返回: 1 = 权限通过; 0 = 权限不通过 (caller 报 fault, page fault cause 由 caller 选)
//
// 检查顺序 (RV Privileged Spec Vol II §4.3.1, §4.4):
//   1. priv 跟 PTE.U 匹配:
//      - PRIV_U: 必须 PTE.U=1 (U 模式只能访问 user pages)
//      - PRIV_S: PTE.U=0 默认允许; PTE.U=1 仅 mstatus.SUM=1 + perm != X 时允许
//                (S 模式访问 user pages 受 SUM 控制; 即使 SUM=1, 也不允许 fetch X-on-U)
//      - PRIV_M: walker 不应在 M 调用 (caller 防御); 不特检 (信任 caller)
//   2. R/W/X 位:
//      - load (PERM_R): 需要 R=1, 或 mstatus.MXR=1 + X=1 (X-on-readable)
//      - store (PERM_W): 需要 W=1; W=1+R=0 spec reserved 但项目跟 spike 风格不查
//      - fetch (PERM_X): 需要 X=1
//
// 注意: PTE.A/PTE.D 检查不在此函数里 — 项目 hw-managed 风格, walker 内 mmu_walk 算建议
// set A/D 后的 new_pte (walker 不真写回 PT, 由 caller 在 RAM 路径写回); 不像 Svade
// 风格那样把 A=0/D=0 当 fault。
//
// fast path 调用形态 (lsu.h lsu_load_helper SV32 TLB hit 段):
//   if ((entry->pte_flags & PTE_V)
//       && entry->gva_tag == vpn
//       && check_perm(hart, entry->pte_flags, MMU_PERM_R)) { /* return *hva 直接 */ }
//   else { /* fall back mmu_walker_helper_load */ }
//
// SMP / 立即生效说明:
//   check_perm 每次都读 hart->trap._mstatus 当前值 (SUM/MXR), 不缓存; 配合 csr 写
//   是块边界 (decode.h is_block_boundary_inst), 块内 mstatus 不变, 块间重派发用新值。
//   见 mmu.h 段 "立即生效语义"。
// ----------------------------------------------------------------------------
static inline int check_perm(cpu_t *hart, u32_t pte, mmu_perm_t perm) {
    /* 物理上 _mstatus 一份 uint64_t 字段 (dummy.txt §6 (1)+(2a) 特殊); SUM/MXR 在低
     * 32 位, 直接 mask 即可 (不需要先 cast 低 32 位再 mask) */
    int sum = (hart->trap._mstatus & MSTATUS_SUM) != 0;
    int mxr = (hart->trap._mstatus & MSTATUS_MXR) != 0;
    int pte_u = (pte & PTE_U) != 0;

    /* priv + PTE.U 检查 */
    if (hart->priv == PRIV_U) {
        if (!pte_u) { return 0; }
    } else if (hart->priv == PRIV_S) {
        if (pte_u) {
            if (!sum) { return 0; }
            if (perm == MMU_PERM_X) return 0;     /* S+SUM 不允许 X-on-U-page */
        }
    }
    /* PRIV_M: 信任 caller (walker 不应在 M 调用); PRIV_H 槽当前永远 NULL 不到此 */

    /* R/W/X 位检查 — switch on perm 跟 mmu_perm_t enum 配 -Wswitch-enum 联动 */
    switch (perm) {
        case MMU_PERM_R:
            if ((pte & PTE_R) == 0 && !(mxr && (pte & PTE_X))) { return 0; }
            break;
        case MMU_PERM_W:
            if ((pte & PTE_W) == 0) { return 0; }
            /* W=1+R=0 RV spec reserved; 项目跟 spike 风格不查 */
            break;
        case MMU_PERM_X:
            if ((pte & PTE_X) == 0) { return 0; }
            break;
    }
    return 1;
}


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
//          命中 + check_perm(MMU_PERM_X)=1 → 返回 PA + HVA (PA 由 HVA 反推, RAM-only TLB
//                                            缓存让 sub 一定有效)
//          命中 + check_perm 失败 → trap_set_exception_state(12 inst page fault)
//          未命中 → 走 step 2
//     2. miss → mmu_walk(hart, gva, PERM_X, &pa, ...) 走页表
//     3. PMP / PMA 物理侧检查 (当前不实现 PMP, 用 PA 在 RAM 区检查代替)
//     4. PA → HVA via pa_to_fetch_hva (RAM ✓, 未来 ROM ✓, 其它 → access fault)
//     5. TLB insert (walker 路径填; entry 内容从 PTE 实际位拷贝)
//
// 参数:
//   hart        — 调用 hart (内部读 hart->regs[0] 作为 gva = pc)
//   current_tlb — NULL = REGIME_BARE (Trust 不走 TLB);
//                 非 NULL = REGIME_SV32, 必须为 dispatcher 选定的叶 TLB
//                          (= hart->tlb_table[priv][asid], dispatcher lazy alloc 后保证非 NULL)
//   pa_out      — 出参; 成功时填 PA (供 dispatcher 查 jit_cache)
//   hva_out     — 出参; 成功时填 HVA (供 dispatcher 传 interp_one_block, 解释器直接取字节)
//
// 返回值 (0/非0 状态信号, 不返回 cause):
//   0   = OK,  pa_out / hva_out 已填, dispatcher 进 block 3
//   非0 = trap 已 deliver (内部已调 trap_set_exception_state 写 xcause/xtval/xepc/regs[0]=xtvec,
//          或 M-mode critical-error 已 set HART_MDT), dispatcher continue 让
//          while(system_reset_signal == 0) 接管退出判断
//
// 失败时不填 pa_out / hva_out (不需要, dispatcher continue 跳过本轮 block; 下一轮 fetch
// 从 xtvec 开始)。dispatcher 不需要自己填 mtval — mmu_translate_pc 自己调 trap_set_exception_state
// 时已经填好 (cause/tval/epc/regs[0] 都设)。
//
// ============================================================================
// dispatcher 的预期使用形态 (见 dispatcher.c)
// ============================================================================
//
//   while (atomic_load(&system_reset_signal) == 0) {
//       /* block 1: regime / current_tlb */
//       ...
//       int rc = mmu_translate_pc(hart, current_tlb, &pa, &hva);
//       if (rc != 0) continue;     /* trap 已 deliver, regs[0] 已是 xtvec, continue */
//
//       /* block 3: interpret / JIT */
//       interpret_one_block(hart, current_tlb, hva, &local_count);
//   }
//   /* dispatcher 退出: SRS 非 0 (任一 bit, 详 runtime.h), main 按 ABORT_MASK 处理 */
//
// ----------------------------------------------------------------------------
int mmu_translate_pc(cpu_t *hart, tlb_t *current_tlb,
                     uxlen_t *pa_out, uint8_t **hva_out);


// ============================================================================
// SV32 walker —— mmu_walk + mmu_walker_helper_load / store
// ============================================================================
//
// 整体设计 (跟 dummy.txt §8 三层 + lsu.h 顶段调用拓扑一致):
//
//   lsu_load_helper (lsu.h, inline 顶层)         lsu_store_helper (lsu.h, inline 顶层)
//             │                                            │
//             ├─ BARE: IS_GPA_RAM ? *hva : mmio_*_helper
//             │                                            │
//             ├─ SV32 TLB hit + check_perm:                │
//             │   load 直接 return *hva                    store 调 store_helper(hva,...)
//             │                                            │
//             └─ SV32 miss/D=0/perm → fall back ───────────┤
//                                          ↓               ↓
//                            mmu_walker_helper_load    mmu_walker_helper_store
//                            (mmu.c, slow path)        (mmu.c, slow path)
//                                          │               │
//                                          ├─ mmu_walk → (pa, pte_flags, fault, pte_pa)
//                                          │              [walker 不写回 PT, 输出 pte_pa]
//                                          ├─ MMIO (!IS_GPA_RAM):
//                                          │   mmio_read_helper       mmio_write_helper
//                                          │   (不 fill TLB, 不 atomic OR PTE, 直接派发)
//                                          └─ RAM:
//                                              caller atomic OR PTE.A/D +
//                                              fill TLB +              fill TLB +
//                                              直接 *hva               store_helper(hva,...)
//
// fast/slow 不对称的真机理 (详 src/isa/lsu.h 顶段 + dummy.txt §1 末段):
//   - load 路径: TLB 缓存 hva + MMIO 不进 TLB → 命中路径**结构上**不带 RAM/MMIO 分支
//                → 直接 return *hva (没有"store_helper 那样的副作用 helper")
//   - store 路径: 命中也走 store_helper (HVA-based) — 因 LR/SC reservation + 未来
//                SMC 副作用强制经 helper, 跟 RAM/MMIO 分流无关
//   → 不是"性能 vs 副作用" 二分。
//
// ----------------------------------------------------------------------------
// hw-managed A/D 设计 (RV Privileged Spec Vol II §4.3.1, 非 Svade 路径)
// ----------------------------------------------------------------------------
//
// RV spec 非 Svade 扩展默认: hw 自动 set PTE.A (任何访问) / PTE.D (W 访问), 不 clear。
//
// 写回时机:
//   - mmu_walk 内部**不真写回**, 只输出 leaf PTE 的 PA (pte_pa_out)。
//   - 三个 caller (mmu_translate_pc / mmu_walker_helper_load / mmu_walker_helper_store)
//     **在 RAM 路径确认通过后**调 `atomic_fetch_or_explicit(host_pte_addr, set_bits,
//     memory_order_relaxed)` 设 A/D (set_bits 由 caller 按 perm 决定: fetch/load =
//     PTE_A; store = PTE_A | PTE_D)。MMIO 路径不写回。
//
// 为什么 walker 不直接写回:
//   假如 walker 内 atomic OR, walker 成功 set A/D 后, walker_helper 判 IS_GPA_RAM
//   (pa) → MMIO 路径调 mmio_*_helper, 若 bus 未命中 / device 拒绝触发 trap_raise
//   (cause 7) → **PTE.A/D 已 set 但访问实际失败**, 跟 Spike "fail 不 set" 不一致
//   (RV spec implementation-defined 不违规, 但形态不漂亮)。
//   现形态: walker 不写回; walker_helper 只在 IS_GPA_RAM 通过的 RAM 路径 atomic OR。
//   MMIO 路径不写回 (跟 Spike fail-不-set 一致简化版; MMIO 成功访问也不 set, 因
//   MMIO PTE 不缓存到 TLB, 每次重 walk 时仍按 PT 原值, 行为正确)。
//
// 为什么 atomic_fetch_or 而不是 plain memcpy:
//   多 hart 并发 walk 同一 PTE 都拿到"该 set A/D"建议时, plain 4-byte memcpy 可能
//   race 丢 bit (hart 0 set A 跟 hart 1 set D 并发写 PTE 互相覆盖)。atomic_fetch_or
//   保证所有 set bit 都到位。memory_order_relaxed 即可 — PTE A/D 跟 guest 数据访问
//   没 happens-before 依赖, 不需要 acquire/release barrier。
//
// TLB fall back 风格 (项目简化 vs 真实硬件):
//   - 真实硬件优化: TLB entry 内存 PTE 物理地址, hit 时直接写 D 位到 PA 不重 walk
//   - 项目简化: TLB entry 不存 PTE 地址 (省 4 字节, fast path 短小);
//                D=0 + store 命中 TLB → fall back to mmu_walker_helper_store →
//                walker 重 walk + 建议 set D + caller 写回 PT + 重 fill TLB;
//                几百 cycle/次, 但只发生在 page 第一次被 store, 平均下来不亏
//
// 时序场景 (page 第一次 load, 然后 store):
//   1. 第一次 load: TLB miss → walker walk PT, 算 new_pte (set A=1); walker_helper
//                    RAM 路径写回 PT; fill TLB (entry.pte_flags 含 A=1, D=0)
//   2. 第二次 load 同 page: TLB hit, 检查 R + (A 永远 1, 不查) → host load
//   3. 第三次 store 同 page: TLB hit, 检查 W=1 + D=? → D=0 不满足, fall back to
//                            mmu_walker_helper_store → walker 算 new_pte (set D=1) +
//                            caller 写回 PT + 重 fill TLB (现在 D=1) → host store
//   4. 第四次 store 同 page: TLB hit, W=1 + D=1 → 直接 host store, 不 fall back
//
// fast path A 位检查冗余: walker 进 TLB 时永远 set A=1, 后续 fast path 检查 A 永远过, 可 skip;
// D 位检查不冗余: load 时 walker 不 set D, store 时 fast path 必须检查 D 决定 fall back。
//
// SMP A/D atomic (a_03 session_024 末落地):
//   caller 写回 PT 用 atomic_fetch_or_explicit (memory_order_relaxed), 多 hart 并发
//   walk 同 PTE 时不丢 set bit。set_bits = PTE_A (fetch/load) 或 PTE_A | PTE_D (store)。
//   写回点在 caller 端三处 (mmu_translate_pc / mmu_walker_helper_load / store);
//   walker 不再算 OR 后的 new_pte, 只输出 leaf PTE 的 PA (pte_pa_out)。
//
// ----------------------------------------------------------------------------
// G 位 (Global) 处理 (项目 G-agnostic, 简化, 跟 sfence 4.a 哲学一致)
// ----------------------------------------------------------------------------
//
// PTE.G 位 (Global): RV spec sw-managed, 由 OS 设, 标记 kernel 等"对所有 ASID 可见"
// 的 mapping; sfence.vma 严格 spec 要求"清单 ASID 时跳过 G=1 entry"。
//
// 项目策略: walker 不特殊处理 G 位 (PTE.G 会拷到 TLB entry pte_flags 但项目代码不读);
// sfence.vma 4.a 简化方案 (单 ASID 清也是 tlb_clear 整套 1KB) 不区分 G 位, 即 G=1
// entry 也被过度清。RV spec 允许过度清除 (over-flush is correct, just slower);
// 性能损失体现在 kernel page 每次 sfence 后被重清, 但行为正确。跟 plan §1.4 / §1.8
// 简化哲学一致。未来真上 OS 实测发现 kernel TLB miss 是瓶颈再细化。
//
// ----------------------------------------------------------------------------
// 立即生效语义 (RV Privileged Spec Vol II §4.1.11)
// ----------------------------------------------------------------------------
//
// spec: "Changes to sstatus.SUM/MXR ... satp.MODE Bare↔Sv32 ... satp.ASID 都立即生效,
// 不需要执行 SFENCE.VMA"。
//
// 项目通过两个机制自动满足:
//   1. csr 写全部是块边界 (plan §1.6 简化, decode.h is_block_boundary_inst CSRRW 等
//      → return 1): csrw satp / csrw mstatus 后块立即退出, dispatcher 重派发时
//      block 1 重新算 (regime, current_tlb), 用当前 satp/mstatus 值
//   2. walker 每次访问读 hart->trap._mstatus 当前值 (SUM/MXR), 不缓存
//
// 注意: 这跟 "satp 写不自动 sfence" (csr.c csr_satp_write 注释) 不矛盾:
//   - "立即生效新 ASID/MODE": 新 satp 值立即被 dispatcher 用 (block 1 选 leaf TLB)
//   - "旧 ASID 的 TLB entries 不被自动清": 仍残留, 由 guest 显式 sfence 清; 切回
//      旧 ASID 时旧 cached entries 还可见 (这是 RV spec 设计, 让 OS 决定刷新时机)
//
// ----------------------------------------------------------------------------
// PT 物理地址检查 (不实现 PMP, 用 RAM 区检查代替)
// ----------------------------------------------------------------------------
//
// walker 走 PT 时, 每一级 PT 的物理地址必须能被 host 安全访问 — 即在 host_ram_base
// 已 mmap 的 RAM 区内。否则 gpa_to_hva_offset + pa 越界, host segfault。
//
// 严格 RV spec: PT 物理地址过 PMP 检查; 不通过 → access exception (cause 1/5/7,
// 取决于 perm)。当前不实现 PMP, 用"PT 在 GUEST_RAM 区" 检查代替, fault cause 同
// access fault。未来真做 PMP 时, 这里改成 PMP allow 检查, cause 不变。
//
// fixture 应有 reject test: satp.PPN 指向 RAM 外 → cause 5。
//
// ============================================================================
// mmu_walk —— SV32 三级 (实际 2 级) walker; 不长跳, 不 fill TLB, 不写回 PT
// ============================================================================
//
// 走 PT + 权限检查 + 输出 leaf PTE 的 PA (caller 在 RAM 路径 atomic_fetch_or 设
// A/D bit; walker 不真写回 PT)。
// 失败 return 非 0 + 填 fault_cause_out (caller 根据 cause 调 trap_raise_exception)。
// 成功 return 0 + 填 pa_out + pte_flags_out (含建议 set 后的 A/D) + pte_pa_out
// (leaf PTE 在 PT 的 PA, caller atomic OR 用)。
//
// 算法 (RV Privileged Spec Vol II §4.3.2):
//   1. root_pa = (satp.PPN) << 12; vpn1 = vaddr[31:22]; vpn0 = vaddr[21:12];
//      offset = vaddr[11:0]
//   2. level=1: pte1 @ root_pa + vpn1*4
//      - V=0 → page fault
//      - R/W/X 任一非 0 → leaf (4MB superpage)
//      - R/W/X 全 0 → pointer to next-level PT, 进 level=0
//   3. level=0: pte0 @ (pte1.PPN<<12) + vpn0*4
//      - V=0 → page fault
//      - R/W/X 任一非 0 → leaf (4KB page)
//      - R/W/X 全 0 → misformatted (level=0 应 leaf) → page fault
//   4. 权限检查 (priv + SUM/MXR + PTE.U + PTE.R/W/X + W=1+R=0 reserved + 4MB
//      superpage misalign)
//   5. hw-managed A/D 建议 mask: walker 不算 OR 后的 new_pte, 只输出 leaf PTE 的 PA
//      (pte_pa_out); caller 在 RAM 路径 atomic_fetch_or 设 set_bits (PTE_A 或
//      PTE_A | PTE_D, 按 perm 决定)。已 set 的 bit 重 OR 无副作用 (atomic OR 一律跑)。
//   6. 算 PA: 4MB superpage = (pte1.PPN[1] << 22) | (vpn0 << 12) | offset;
//             4KB page = (pte0.PPN << 12) | offset
//
// 4MB superpage: walker 内识别 + 检查 PTE.PPN[0]==0 (不对齐 → page fault); PA 算出后
// caller (walker_helper_*) 仍按 4KB fill TLB (TLB 不带 size 字段)。同 4MB 内不同 4KB
// 偏移每次都要重 walk; correct 但 first-touch 慢, kernel direct-map 用 4MB superpage
// 时 TLB miss 频率高 — walker ~100 cycle, 不在 fast path 不亏 (trade-off: TLB 不带
// size 字段省 fast path 比较开销)。
//
// 参数:
//   hart            - 调用 hart (内部读 hart->priv / hart->trap._mstatus / hart->satp)
//   gva             - guest 虚拟地址
//   perm            - 访问类型 (MMU_PERM_R / W / X), 决定权限/cause/A_D set/SUM-MXR 处理
//   pa_out          - 出参; 成功填 PA
//   pte_flags_out   - 出参; 成功填 leaf PTE 低 10 位 (含建议 set A/D 后的状态);
//                      caller 用于 fill TLB entry.pte_flags
//   fault_cause_out - 出参; 失败填 cause (CAUSE_LOAD_PAGE_FAULT / STORE_PAGE_FAULT /
//                      INST_PAGE_FAULT / LOAD_ACCESS_FAULT / STORE_ACCESS_FAULT /
//                      INST_ACCESS_FAULT)
//   pte_pa_out      - 出参; 成功时填: PT 内 leaf PTE 的 PA (caller 在 RAM 路径
//                      调 atomic_fetch_or 设 A/D bit; memory_order_relaxed 即可,
//                      PTE A/D 跟 guest 数据访问无 happens-before 依赖)。
//                      已 set 的 bit 重 OR 无副作用, walker 不再做 "需 set vs 已
//                      set" 优化判定 — 永远输出 pte_pa, caller 永远 atomic OR。
//
// 返回值: 0 = 成功; 非 0 = 失败 (cause 已填 fault_cause_out; 其他 out 不可读)
//
// caller 模式 (RAM 路径才写回; MMIO 路径不写回, 简化哲学):
//   if (mmu_walk(...) != 0) trap_raise(hart, fault_cause, gva);
//   if (!IS_GPA_RAM(pa)) { /* MMIO 路径: 直走 mmio_*_helper, 不写回 PT */ }
//   else {
//       atomic_fetch_or_explicit((_Atomic uint32_t *)(gpa_to_hva_offset + pte_pa),
//                                set_bits, memory_order_relaxed);
//       /* set_bits = PTE_A (load/fetch); PTE_A | PTE_D (store) */
//       /* RAM 真访问 */
//   }
int mmu_walk(cpu_t *hart, uxlen_t gva, mmu_perm_t perm,
             uxlen_t *pa_out, u32_t *pte_flags_out,
             uint32_t *fault_cause_out,
             uxlen_t *pte_pa_out);


// ============================================================================
// mmu_walker_helper_load —— SV32 load 路径完整流程; helper 长跳风格 (路径 2a)
// ============================================================================
//
// 调用方: lsu.h lsu_load_helper SV32 路径 TLB miss / 权限不齐时; 未来 JIT 翻译产物
//          emit `call mmu_walker_helper_load` 作 JIT 块出口 (helper 颗粒度 by design,
//          见顶段 + dummy.txt §10)。
//
// 流程 (跟 §8 三层模型一致):
//   1. mmu_walk(hart, gva, MMU_PERM_R) → pa + pte_flags + fault_cause + pte_pa
//      (walker 输出 leaf PTE 的 PA, 不真写回; caller 在 RAM 路径 atomic OR)
//      失败 → trap_raise_exception(cause, gva)  /* _Noreturn longjmp */
//   2. PA 落 MMIO (!IS_GPA_RAM) → mmio_read_helper(hart, pa, gva, size) 返值
//      (不 fill TLB; **不 atomic OR PTE.A** — MMIO 路径不写回, 跟 Spike
//      "fail 不 set" 一致简化版; bus 内部 _Noreturn-on-failure, 未命中 / device
//      拒绝直接长跳)
//   3. PA 落 RAM:
//      a. host_ptr = gpa_to_hva_offset + pa
//      b. **atomic OR PTE.A** (RAM 路径; atomic_fetch_or_explicit relaxed; 多 hart
//         并发 walk 同 PTE 不丢 bit)
//      c. fill TLB entry (lazy refresh, 在 host load 之前):
//           entry.gva_tag  = gva >> 12
//           entry.pte_flags = pte_flags  (含建议 set 后的 A=1)
//           entry.host_ptr = host_ptr - (gva & 0xFFF)   /* page 起点 host 地址 */
//      d. **直接 *host_ptr memcpy** (不调子 helper — 跟 lsu_load_helper SV32 命中
//         段同形, walker 已知是 RAM, 没必要再去 lsu 走一圈; 详 dummy.txt §1 末段
//         load/store 不对称真机理)
//      e. return uint32_t (低 size 字节有效, sext/zext 由 caller 做)
//
// 错误路径 trap_raise_exception 长跳, 不返回 caller (caller 不需要 goto out 处理失败)。
uxlen_t mmu_walker_helper_load(cpu_t *hart, tlb_t *current_tlb,
                               uxlen_t gva, uint32_t size);


// ============================================================================
// mmu_walker_helper_store —— SV32 store 路径完整流程; helper 长跳风格
// ============================================================================
//
// 调用方: lsu.h lsu_store_helper SV32 路径 TLB miss / 权限不齐 / D=0 时; 未来 JIT
//          翻译产物 emit `call mmu_walker_helper_store` 作 JIT 块出口。
//
// 流程跟 mmu_walker_helper_load 对称, perm = MMU_PERM_W:
//   1. mmu_walk(hart, gva, MMU_PERM_W) → pa + pte_flags + fault_cause + pte_pa
//      (walker 输出 leaf PTE 的 PA, 不真写回; caller 在 RAM 路径 atomic OR A+D)
//      失败 → trap_raise_exception(15/7, gva)
//   2. PA 落 MMIO → mmio_write_helper(hart, pa, gva, value, size) 直接派发
//      (不 fill TLB; **不 atomic OR PTE.A/D**; 不调 store_helper, 跳过 LR/SC + SMC
//      副作用 — 因 MMIO 不参与 reservation 也非可执行不参与 SMC; 跟 lsu_store_helper
//      BARE MMIO 路径同形)
//   3. PA 落 RAM:
//      a. host_ptr = gpa_to_hva_offset + pa
//      b. **atomic OR PTE.A+D** (RAM 路径; atomic_fetch_or_explicit relaxed; 多 hart
//         并发 walk 同 PTE 不丢 bit)
//      c. fill TLB entry (pte_flags 含建议 set 后的 D=1)
//      d. **store_helper(hart, host_ptr, gva, value, size)** — 委托 lsu 层做
//         RAM 写 + LR/SC reservation 清除占位 + SMC 占位 (跟 lsu.h 顶段同形;
//         store_helper 是唯一统一的 "RAM 写 + 副作用" 入口, 三处 caller 共用;
//         详 dummy.txt §1 末段 load/store 不对称真机理)
//
// 错误路径 trap_raise_exception 长跳, 不返回 caller。
void mmu_walker_helper_store(cpu_t *hart, tlb_t *current_tlb,
                             uxlen_t gva, uxlen_t value, uint32_t size);


// ============================================================================
// mmu_walker_helper_amo_* —— SV32 AMO 路径完整流程; helper 长跳风格 (Zaamo)
// ============================================================================
//
// 9 个 walker (跟 mmu_walker_helper_store 体例同 + 末调对应 amo_xxx_apply 而非 store_helper):
//
//   调用方: isa/amo.h amo_xxx_helper SV32 路径 TLB miss / 权限不齐 / D=0 时; 未来 JIT
//            翻译产物 emit `call mmu_walker_helper_amo_xxx` 作 JIT 块出口.
//
//   流程 (跟 mmu_walker_helper_store 严格对偶, perm = MMU_PERM_W):
//     1. mmu_walk(hart, gva, MMU_PERM_W) → pa + pte_flags + fault_cause + pte_pa
//        失败 → trap_raise_exception(15/cause 7, gva)  /* page fault / access fault */
//     2. PA 落 MMIO → trap_raise(7, gva) — cause 7 store access fault.
//        **不开 mmio_amo_helper** (跟 store walker 不同 — store walker 调 mmio_write_helper;
//        AMO 拒 MMIO, RV spec implementation-defined, Spike/QEMU 同).
//     3. PA 落 RAM:
//        a. host_ptr = gpa_to_hva_offset + pa
//        b. **atomic OR PTE.A+D** (RAM 路径; relaxed; 跟 store walker 同位置)
//        c. fill TLB entry (pte_flags 含 set 后的 D=1)
//        d. **amo_xxx_apply(hart, host_ptr, gva, value)** — 委托 amo 层做 host atomic RMW
//            + lrsc_on_store, 返 sext32(old).
//            (跟 store walker 末调 store_helper 同形态, **但用 9 个独立 apply 而非统一
//             store_helper** — 因 AMO 各 op atomic 操作不同, 不能用 size 分流; 见
//             amo.h 顶段)
//   返回: 32-bit old value (rd 写回值; 跟 amo_xxx_apply 返回值同).
//   错误路径: trap_raise_exception 长跳, 不返回 caller (跟 walker_helper_store 同模式).
//
// 9 个 walker 体几乎一致, mmu.c 用 macro AMO_DEFINE_WALKER 注入避免重复.
// ============================================================================
uxlen_t mmu_walker_helper_amo_add_w (cpu_t *hart, tlb_t *current_tlb, uxlen_t gva, uxlen_t value);
uxlen_t mmu_walker_helper_amo_swap_w(cpu_t *hart, tlb_t *current_tlb, uxlen_t gva, uxlen_t value);
uxlen_t mmu_walker_helper_amo_xor_w (cpu_t *hart, tlb_t *current_tlb, uxlen_t gva, uxlen_t value);
uxlen_t mmu_walker_helper_amo_or_w  (cpu_t *hart, tlb_t *current_tlb, uxlen_t gva, uxlen_t value);
uxlen_t mmu_walker_helper_amo_and_w (cpu_t *hart, tlb_t *current_tlb, uxlen_t gva, uxlen_t value);
uxlen_t mmu_walker_helper_amo_min_w (cpu_t *hart, tlb_t *current_tlb, uxlen_t gva, uxlen_t value);
uxlen_t mmu_walker_helper_amo_max_w (cpu_t *hart, tlb_t *current_tlb, uxlen_t gva, uxlen_t value);
uxlen_t mmu_walker_helper_amo_minu_w(cpu_t *hart, tlb_t *current_tlb, uxlen_t gva, uxlen_t value);
uxlen_t mmu_walker_helper_amo_maxu_w(cpu_t *hart, tlb_t *current_tlb, uxlen_t gva, uxlen_t value);


// ============================================================================
// mmu_walker_helper_lr_w / sc_w —— SV32 LR/SC 路径完整流程 (Zalrsc)
// ============================================================================
//
// 跟 mmu_walker_helper_store 严格对偶, 末调 lrsc_lr_w / lrsc_sc_w (PA-based; lrsc.h):
//
//   lr_w walker (perm = MMU_PERM_R):
//     1. mmu_walk(hart, gva, MMU_PERM_R, ...) → pa / pte_flags / fault_cause / pte_pa
//        失败 → trap_raise_exception(fault_cause, gva)
//     2. PA 不在 RAM → trap_raise(CAUSE_LOAD_ACCESS_FAULT=5, gva)
//        (LR 落 MMIO spec implementation-defined, 项目拒 cause 5; 跟 Spike/QEMU 同)
//     3. RAM:
//        a. atomic OR PTE.A (R-only walk, 不 set D)
//        b. fill TLB entry (跟 load walker 同形态)
//        c. 末调 lrsc_lr_w(hart, pa) — PA-based, 返 zero-ext *pa 32-bit
//
//   sc_w walker (perm = MMU_PERM_W):
//     1. mmu_walk(hart, gva, MMU_PERM_W, ...) → 同上
//        失败 → trap_raise_exception(fault_cause, gva)
//     2. PA 不在 RAM → trap_raise(CAUSE_STORE_ACCESS_FAULT=7, gva)
//        (SC 落 MMIO 拒, 跟 AMO/store walker 同)
//     3. RAM:
//        a. atomic OR PTE.A+D
//        b. fill TLB entry
//        c. 末调 lrsc_sc_w(hart, pa, value) — 返 0=success / 1=fail
//
// 错误路径 trap_raise_exception 长跳, 不返回 caller (跟 walker_helper_store 同模式).
// ============================================================================
uxlen_t mmu_walker_helper_lr_w(cpu_t *hart, tlb_t *current_tlb, uxlen_t gva);
uxlen_t mmu_walker_helper_sc_w(cpu_t *hart, tlb_t *current_tlb, uxlen_t gva, uxlen_t value);

#endif //CORE_MMU_H
