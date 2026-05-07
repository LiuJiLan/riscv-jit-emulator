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
// mmu_perm_t —— 访问类型枚举 (a_01_8 加, 给 mmu_walk + walker helpers 用)
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
    MMU_PERM_X = 2,    // fetch 访问 (未来 mmu_translate_pc 真接 SV32 时用)
} mmu_perm_t;


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


// ============================================================================
// SV32 walker (a_01_8) —— mmu_walk + mmu_walker_helper_load / store
// ============================================================================
//
// 整体设计:
//
//   load_helper (lsu.h, fast path inline)        store_helper (lsu.c, slow path)
//             │                                            │
//             ├─ TLB lookup inline (hit + 权限齐 → host load 完成)
//             │                                            │
//             └─ TLB miss / 权限不齐 → fall back ──────────┤ 同 (TLB lookup + miss
//                                          │               │  / D=0 → fall back)
//                                          ↓               ↓
//                            mmu_walker_helper_load    mmu_walker_helper_store
//                            (mmu.c, slow path)        (mmu.c, slow path)
//                                          │               │
//                                          ├─ mmu_walk → (pa, pte_flags, fault)
//                                          ├─ ram check (PA in RAM 区)
//                                          ├─ fill TLB entry (lazy refresh, D14)
//                                          └─ host load / host store + return
//
// 注意 fast/slow 不对称 (dummy.txt §1 末段):
//   - load 路径: fast = TLB-hit inline, slow = mmu_walker_helper_load
//   - store 路径: store_helper 整体 slow path (extern 函数 call), 但内部仍有
//                  "TLB lookup 短路径"避免每次 walk PT; "fall back" 在 store 路径意思
//                  store_helper 内 inline 查 → mmu_walker_helper_store 函数 (helper 之间分层)
//   - 未来 file_plan §F1 改进: store fast path inline 化, 那时整 store 路径才跟 load 对称
//
// ----------------------------------------------------------------------------
// hw-managed A/D 设计 (RV Privileged Spec Vol II §4.3.1, 非 Svade 路径)
// ----------------------------------------------------------------------------
//
// RV spec 非 Svade 扩展默认: hw 自动 set PTE.A (任何访问) / PTE.D (W 访问), 不 clear。
// 项目按此实现, walker 内顺手 set + 写回 PT (gpa_to_hva offset memcpy 4 字节)。
//
// TLB fall back 风格 (项目简化 vs 真实硬件):
//   - 真实硬件优化: TLB entry 内存 PTE 物理地址, hit 时直接写 D 位到 PA 不重 walk
//   - 项目简化: TLB entry 不存 PTE 地址 (省 4 字节, fast path 短小);
//                D=0 + store 命中 TLB → fall back to mmu_walker_helper_store →
//                walker 重 walk + set D + 写回 PT + 重 fill TLB; 几百 cycle/次,
//                但只发生在 page 第一次被 store, 平均下来不亏
//
// 时序场景 (page 第一次 load, 然后 store):
//   1. 第一次 load: TLB miss → walker walk PT, set A=1 (load 不 set D), 写回 PT;
//                    fill TLB (entry.pte_flags 含 A=1, D=0)
//   2. 第二次 load 同 page: TLB hit, 检查 R + (A 永远 1, 不查) → host load
//   3. 第三次 store 同 page: TLB hit, 检查 W=1 + D=? → D=0 不满足, fall back to
//                            mmu_walker_helper_store → walker set D=1 + 写回 PT +
//                            重 fill TLB (现在 D=1) → host store
//   4. 第四次 store 同 page: TLB hit, W=1 + D=1 → 直接 host store, 不 fall back
//
// fast path A 位检查冗余: walker 进 TLB 时永远 set A=1, 后续 fast path 检查 A 永远过, 可 skip;
// D 位检查不冗余: load 时 walker 不 set D, store 时 fast path 必须检查 D 决定 fall back。
//
// SMP atomic 占位 (plan §1.9, a_01 单 hart 不实现):
//   walker 写回 PT 时单 hart 用 memcpy 即可; SMP 时 PTE 位 set 必须 atomic_fetch_or
//   (跨 hart 同步, 多 hart 并发 walk 同 PTE 时不丢 set)。注释 "// SMP: atomic" 备查。
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
// PT 物理地址检查 (a_01 不实现 PMP, 用 RAM 区检查代替)
// ----------------------------------------------------------------------------
//
// walker 走 PT 时, 每一级 PT 的物理地址必须能被 host 安全访问 — 即在 host_ram_base
// 已 mmap 的 RAM 区内。否则 gpa_to_hva_offset + pa 越界, host segfault。
//
// 严格 RV spec: PT 物理地址过 PMP 检查; 不通过 → access exception (cause 1/5/7,
// 取决于 perm)。a_01 不实现 PMP, 用"PT 在 GUEST_RAM 区" 检查代替, fault cause 同
// access fault。未来真做 PMP 时, 这里改成 PMP allow 检查, cause 不变。
//
// fixture 应有 reject test (Step 8 列表): satp.PPN 指向 RAM 外 → cause 5。
//
// ============================================================================
// mmu_walk —— SV32 三级 (实际 2 级) walker; 不长跳, 不 fill TLB
// ============================================================================
//
// 走 PT + 权限检查 + hw-managed A/D set + 写回 PT。失败 return 非 0 + 填 fault_cause_out
// (caller 根据 cause 调 trap_raise_exception)。成功 return 0 + 填 pa_out + pte_flags_out。
//
// 算法 (RV Privileged Spec Vol II §4.3.2):
//   1. root_pa = (satp.PPN) << 12; vpn1 = vaddr[31:22]; vpn0 = vaddr[21:12];
//      offset = vaddr[11:0]
//   2. level=1: pte1 @ root_pa + vpn1*4
//      - V=0 → page fault
//      - R/W/X 任一非 0 → leaf (4MB superpage; 见 decision N)
//      - R/W/X 全 0 → pointer to next-level PT, 进 level=0
//   3. level=0: pte0 @ (pte1.PPN<<12) + vpn0*4
//      - V=0 → page fault
//      - R/W/X 任一非 0 → leaf (4KB page)
//      - R/W/X 全 0 → misformatted (level=0 应 leaf) → page fault
//   4. 权限检查 (priv + SUM/MXR + PTE.U + PTE.R/W/X + W=1+R=0 reserved + 4MB
//      superpage misalign)
//   5. hw-managed A/D: A 永远 set; W 访问 set D; 已 set 不重写; 写回 PT
//   6. 算 PA: 4MB superpage = (pte1.PPN[1] << 22) | (vpn0 << 12) | offset;
//             4KB page = (pte0.PPN << 12) | offset
//
// 4MB superpage (decision N): walker 内识别 + 检查 PTE.PPN[0]==0 (不对齐 → page fault);
// PA 算出后 caller (walker_helper_*) 仍按 4KB fill TLB (TLB 不带 size 字段)。同 4MB 内
// 不同 4KB 偏移每次都要重 walk; correct 但 first-touch 慢, kernel direct-map 用 4MB
// superpage 时 TLB miss 频率高 — walker ~100 cycle, 不在 fast path 不亏 (decision N
// 末段 trade-off)。
//
// 参数:
//   hart            - 调用 hart (内部读 hart->priv / hart->trap._mstatus / hart->satp)
//   gva             - guest 虚拟地址
//   perm            - 访问类型 (MMU_PERM_R / W / X), 决定权限/cause/A_D set/SUM-MXR 处理
//   pa_out          - 出参; 成功填 PA
//   pte_flags_out   - 出参; 成功填 leaf PTE 低 10 位 (V/R/W/X/U/G/A/D + RSW); caller
//                      用于 fill TLB entry.pte_flags
//   fault_cause_out - 出参; 失败填 cause (CAUSE_LOAD_PAGE_FAULT / STORE_PAGE_FAULT /
//                      INST_PAGE_FAULT / LOAD_ACCESS_FAULT / STORE_ACCESS_FAULT /
//                      INST_ACCESS_FAULT)
//
// 返回值: 0 = 成功; 非 0 = 失败 (cause 已填 fault_cause_out)
int mmu_walk(cpu_t *hart, uint32_t gva, mmu_perm_t perm,
             uint32_t *pa_out, uint32_t *pte_flags_out,
             uint32_t *fault_cause_out);


// ============================================================================
// mmu_walker_helper_load —— SV32 load 路径完整流程; helper 长跳风格 (路径 2a)
// ============================================================================
//
// 调用方: lsu.h load_helper SV32 路径 (Step 6 替换占位段); TLB miss / 权限不齐 时调用。
//
// 流程:
//   1. mmu_walk(hart, gva, MMU_PERM_R) → pa + pte_flags + fault_cause
//      失败 → trap_raise_exception(cause, gva)  /* _Noreturn longjmp */
//   2. PA 在 RAM 区检查 (a_01 不实现 bus_dispatch, 不在 RAM → access fault):
//      失败 → fprintf + trap_raise_exception(CAUSE_LOAD_ACCESS_FAULT, gva)
//   3. host_ptr = gpa_to_hva_offset + pa
//   4. fill TLB entry (D14 lazy refresh, 在 host load 之前, 副作用要在成功路径才发生):
//        entry = current_tlb->e[(gva >> 12) & (TLB_NUM_ENTRIES-1)]
//        entry.gva_tag  = gva >> 12
//        entry.pte_flags = pte_flags  (含 walker set 的 A=1)
//        entry.host_ptr = host_ptr - (gva & 0xFFF)   /* page 起点 host 地址 */
//   5. host load size 字节 + sext/zext (caller 做 — load_helper 接口对齐) → return uint32_t
//
// 错误路径 trap_raise_exception 长跳, 不返回 caller (caller 不需要 goto out 处理失败)。
uint32_t mmu_walker_helper_load(cpu_t *hart, tlb_t *current_tlb,
                                uint32_t gva, uint32_t size);


// ============================================================================
// mmu_walker_helper_store —— SV32 store 路径完整流程; helper 长跳风格
// ============================================================================
//
// 调用方: lsu.c store_helper SV32 路径 (Step 6 替换占位段); store_helper 内 TLB miss /
//   权限不齐 / D=0 时调用。
//
// 流程跟 mmu_walker_helper_load 对称, perm = MMU_PERM_W:
//   1. mmu_walk(hart, gva, MMU_PERM_W) → walker 内 set A+D 写回 PT
//   2. PA 在 RAM 区检查 (失败 → CAUSE_STORE_ACCESS_FAULT)
//   3. host_ptr = gpa_to_hva_offset + pa
//   4. fill TLB entry (pte_flags 含 D=1)
//   5. host store size 字节 (memcpy)
//   6. reservation 清除占位 (a_01 LR/SC 未做; 跟 store_helper BARE 路径同形态注释)
void mmu_walker_helper_store(cpu_t *hart, tlb_t *current_tlb,
                             uint32_t gva, uint32_t value, uint32_t size);

#endif //CORE_MMU_H
