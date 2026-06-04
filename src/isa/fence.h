//
// Created by liujilan on 2026/6/4.
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
//   - 未来 SMC chain hook: 块边界 JIT cache page invalidate / page_dirty bitmap
//     从这里加 (a_05+); 当前不实
//
// fence.i 是块边界, plain fence 不是 (Q14 拍): fence.i 改 i-cache 视图, 块内后续指令
// 译码假设失效; plain fence 是 memory ordering, 不动指令流, 可以同 block 继续译.
// is_block_boundary_inst (decode.h) 体现该分类.
//

#ifndef ISA_FENCE_H
#define ISA_FENCE_H

#include "core/cpu.h"        // cpu_t


// FENCE (opcode 0x0F, funct3=000): 通用 memory ordering fence 兼 fence.tso / pause hint.
// 全 NOP — 上方"双重 cover 论证"; (void)hart 抑制 unused.
void fence_helper(cpu_t *hart);

// FENCE.I (opcode 0x0F, funct3=001, Zifencei): i-cache flush, 副作用 lrsc_clear_self.
// 当前只清 reservation; 未来加 JIT cache invalidate (a_05+ SMC chain).
void fence_i_helper(cpu_t *hart);

#endif //ISA_FENCE_H
