//
// isa/fence —— RV32 FENCE / FENCE.I 指令实现 (Q14 拍 NOP + 副作用)。
//
// 跟 sfence.{c,h} 拆开的原因 (Q19):
//   - opcode 不同: fence/fence.i = 0x0F (MISC-MEM); sfence.vma = 0x73 (SYSTEM + funct7=0x09)
//   - 手册不同卷:   fence/fence.i 在 Vol I unpriv (RVI base + Zifencei);
//                    sfence.vma 在 Vol II §10.6 priv "Supervisor-Mode Memory Management Fence"
//   - 语义不同族:   fence = memory ordering hint; fence.i = i-cache flush;
//                    sfence.vma = TLB shootdown — 三者无共享 helper
//
// 为什么 fence 体可以全 NOP — 双重 cover 论证:
//
//   (1) host x86_64 是 TSO 内存模型, 比 RV 弱内存序模型严:
//         load-load   不重排 (x86 ✓; RV 默认允许重排, fence pred=R succ=R 禁止)
//         load-store  不重排 (x86 ✓)
//         store-store 不重排 (x86 ✓; RV fence pred=W succ=W 禁止)
//         store-load  x86 允许重排 (mfence/lock 抑); RV fence pred=W succ=R 禁止
//       绝大多数 RV fence 组合在 x86 host 上天然满足, 唯一 store-load 例外.
//
//   (2) 但 "真正负责跨 hart 可见性" 在我们 JIT 模型里不是 guest 的 fence,
//       而是 helper 的 C11 atomic memory_order:
//         - store_helper      (lsu.c) — atomic_store_explicit + 显式 memory_order
//         - LR/SC + AMO       (lrsc.c / amo.c — T3) — atomic CAS + acq/rel
//         - block 边界 dispatch  函数调用本身就是 compiler reorder barrier
//       即便 guest 的 fence pred=W succ=R 我们也不 emit host 屏障 — store_helper
//       下次出门时会带 release / 下次进门 load 自带 acquire, ordering 已经在
//       helper 端兜底.
//
// 跟 QEMU/TCG 一致: TCG 把 guest fence 视作 TB 边界天然 serialize, 不 emit host
// 屏障. 真要严格匹配 RV memory model (host 是 ARM/weak-memory 之类) 时, 在本 helper
// 内按 host arch 分编译条件 emit 屏障 — 当前不在目标. 详 plan §2 (deferred).
//
// fence.tso (fm=1000) / pause 暗示 (Zihintpause, fence rd=0,rs1=0,fm=0,pred=0,succ=0)
// 全部按通用 OP_FENCE 走 NOP. 不解 fm/pred/succ 字段 — 256 种 pred×succ 组合都
// 退化为同一空 helper, 没必要 decode 区分.
//
// fence.i 的副作用 (跟 plain fence 区别开):
//   - 调 lrsc_clear_self(hart) — i-cache 同步必伴随 reservation 清 (RV spec 七类清除
//     之一; 见 lrsc.h 顶段 #4)
//   - 不主动调 jit_cache_invalidate_page — invalidate 路径由 SMC SIGSEGV bitmap +
//     dispatcher 顶扫承担 (b_03 T1.a 已接通), fence.i 块边界保证顶扫在 spec 时机
//     执行; 详下方 "fence_i_helper 不调 jit_cache_invalidate_page — 为什么" 段
//
// SMC trigger 端就是 fence.i, 不在 sfence.vma — fence.i 是 i-cache flush (guest
// 自改代码段后的可见性边界, 直接 SMC 相关), sfence.vma 只是 TLB shootdown (地址翻译
// cache 失效, 不动 i-cache / 不动 PA 字节). 上方"三者无共享 helper"的真实含义就是
// 语义边界完全独立, 别把 JIT invalidate 错挂 sfence.vma. 详 sfence.h 顶段"sfence_vma
// _helper 不调 jit_invalidate_block — 为什么"段 (b_02 T5 重审推翻 start_plan_b_02
// §[2] T5 第 5 点; memory `feedback_overturn_plan_leave_trail`).
//
// fence.i 是块边界, plain fence 不是 (Q14 拍): fence.i 改 i-cache 视图, 块内后续指令
// 译码假设失效; plain fence 是 memory ordering, 不动指令流, 可以同 block 继续译.
// is_block_boundary_inst (decode.h) 体现该分类.
//
// fence_i_helper 不调 jit_cache_invalidate_page — 为什么 (b_03 audit Q4.2.1+, 推翻
// start_plan_b_02 末段 trail "未来 SMC chain 这里加 invalidate"):
//
//   b_02 末段 fence_i_helper 顶 doc 写 "未来 SMC chain (a_05+): 这里加 JIT cache
//   page invalidate" 是过时拍法, 基于 "fence.i 必须显式承担 JIT 块失效" 假设.
//   b_03 audit 实际定调:
//
//   1. fence.i RV spec 语义 = instruction stream sync (hart-local i-cache flush);
//      cross-hart 必靠 OS SBI broadcast (SBI_REMOTE_FENCE_I IPI), spec 不保证 cross-
//      hart i-cache coherence without OS 支持
//   2. 我们设计: SMC SIGSEGV bitmap + dispatcher 顶扫 是唯一 invalidate 工作来源
//      (b_03 T1.a 已接通; mprotect host page R-only, 客户机 sw 改 → SIGSEGV handler
//      标 page_dirty + unprotect → dispatcher 顶扫 bitmap → jit_invalidate_page)
//   3. fence.i 在我们 = 块边界 (decode.h is_block_boundary_inst → 1) + lrsc_clear_
//      self 副作用 (RV spec 七类清除 #4); helper 函数体只调 lrsc_clear_self, 不调
//      jit_cache_invalidate_page
//   4. 块边界保证 dispatcher 顶扫 bitmap 在 spec 要求时机执行 — 5 case walk through:
//        Case A (改完调 fence.i): JIT 块译到 fence.i 切块 → 退出 → SMC SIGSEGV 已
//                                 标 bitmap → dispatcher 顶扫触发 invalidate
//        Case B (改完不调 fence.i): SMC SIGSEGV 兜底; 客户机违 spec 但我们仍正确
//                                   (b_03 T1.a 01_smc_single_block fixture verify)
//        Case C (fence.i 没改 page): page_dirty bitmap 空 → 顶扫空转 → JIT 块继续 hit
//        Case D (cross-page 改 + fence.i): handler 按 si_addr page-align 标 dirty →
//                                          dispatcher 顶扫处理 (T1.b 06 SMC 部分 verify)
//        Case E (多 hart race): hart A 改 + hart B 跑 JIT 块; SIGSEGV 在 A 触发, B
//                               退出当前块后才看 bitmap dirty (RV spec 不保 cross-hart
//                               i-cache coherence without IPI, 跟真硬件一致)
//   5. 跟 R15 同精神 (sfence_vma_helper 不调 jit_invalidate_block; 见 sfence.h:15-33):
//      JIT 子系统不主动响应 RV 同步指令; instruction sync 走 SMC SIGSEGV bitmap, TLB
//      sync 不影响 JIT key. 统一原则.
//   6. industry 一致: QEMU TCG user-mode + Box64 都走 mprotect + SIGSEGV + page-based
//      invalidate, 不靠 guest fence.i 主动调
//
//   推翻 trail: b_03_audit_decision.md Q4.2.1+ + b_03_session_001 user 拍 + memory
//   `feedback_overturn_plan_leave_trail`. 未来开发若再质疑 "fence.i 是否该真调
//   jit_cache_invalidate_page", 先读本段.
//

#ifndef ISA_FENCE_H
#define ISA_FENCE_H

#include "core/cpu.h"        // cpu_t


// FENCE (opcode 0x0F, funct3=000): 通用 memory ordering fence 兼 fence.tso / pause hint.
// 全 NOP — 上方"双重 cover 论证"; (void)hart 抑制 unused.
void fence_helper(cpu_t *hart);

// FENCE.I (opcode 0x0F, funct3=001, Zifencei): i-cache flush, 副作用 lrsc_clear_self.
// 不主动调 jit_cache_invalidate_page — 详顶段 "fence_i_helper 不调 ... 为什么" 段
// (b_03 audit Q4.2.1+; 推翻 b_02 末段 "未来 SMC chain 这里加" trail).
void fence_i_helper(cpu_t *hart);

#endif //ISA_FENCE_H
