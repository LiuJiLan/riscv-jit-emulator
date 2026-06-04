//
// Created by liujilan on 2026/6/4.
// isa/lrsc —— RV32 A 扩展 LR.W / SC.W (Zalrsc) + 全局 reservation 数据结构。
//
// T1 / T3 划分 (lrsc_amo_decision.md):
//   T1 (本次) — 文件骨架: 函数声明全套到位 + lrsc_clear_self 空体 (fence.i 真调).
//                lr_w / sc_w / on_store / on_device_write / init / destroy 也都给空 stub,
//                让 T2 期间 amo.c / lsu.c / trap.c / wfi.c / sfence.c 埋的 hook 链接通过;
//                stub 行为 (返回 0 / NOP) 在 T1/T2 路径上是不可达的占位, 不依赖正确性.
//   T3       — 填实: per-PA hash 桶锁 (K=64, Fibonacci hash) + per-hart
//                _Atomic uint32_t reservation_addr (哨兵 INVALID=0xFFFFFFFFu) + LR lock-free
//                / SC 锁内扫所有 hart 清匹配 reservation; Store / AMO 锁内同 + clear.
//                详方案见 lrsc_amo_decision.md "(主方案) 走向 c".
//
// 七类清除时机 (RV Privileged Spec §14.2.1 "Reserved Memory Locations"):
//   1. context switch / trap → trap_set_*_state 入口
//   2. sfence.vma            → sfence_vma_helper 末段
//   3. wfi 醒来               → wfi_wait 后立刻
//   4. fence.i               → 本文件 (T1 真调; trail 跟 SMC 一类的 i-cache 同步刷)
//   5. 普通 store 命中 reservation → lsu.c store_helper 末段调 lrsc_on_store(hart, pa)
//   6. AMO 命中 reservation  → amo.c AMO 末段调 lrsc_on_store
//   7. device DMA / virtio_blk 写 RAM → io_worker 完成段调 lrsc_on_device_write
//
// trap 协议: 本模块不抛 trap (LR/SC misalign / access fault 在 lsu / mmu 路径已处理;
//   reservation 操作纯 in-memory data structure 不触发 RV trap)。
//

#ifndef ISA_LRSC_H
#define ISA_LRSC_H

#include <stdint.h>

#include "core/cpu.h"        // cpu_t
#include "riscv.h"           // uxlen_t / u32_t (typedef family; dummy.txt §13)


// ----------------------------------------------------------------------------
// 生命周期 (POR / teardown 由 main 调; T3 实)
// ----------------------------------------------------------------------------
void lrsc_init(void);
void lrsc_destroy(void);

// ----------------------------------------------------------------------------
// lrsc_clear_self —— 清当前 hart 自己的 reservation (lock-free, 自己写自己字段)
//
// 调用点 (T2/T3 期间逐步接通):
//   - fence.i             (T1 真接通; 见本文件顶段七类 #4)
//   - trap_set_*_state    (T3; #1)
//   - sfence_vma_helper   (T3; #2)
//   - wfi_wait 醒来        (T3; #3)
//
// T1 空体: hart 的 reservation 字段还没加 (T3 才加 _Atomic uint32_t reservation_addr 到
//   cpu_t), 现在直接 (void)hart 抑制 unused, NOP 返回. fence.i 路径在 T1 调本函数是
//   "占位真调", 不是死调 —— T3 一上 reservation 字段就生效, 不需要改 fence.c.
// ----------------------------------------------------------------------------
void lrsc_clear_self(cpu_t *hart);

// ----------------------------------------------------------------------------
// LR.W / SC.W 指令入口 (T3 填实)
//
// 签名按 lrsc_amo_decision.md (主方案 c) 钉死:
//   lr_w: 返 *pa 32-bit 值 (zero-ext 到 uxlen_t); 副作用: self.reservation = pa & ~3u
//   sc_w: 返 0 = success / 1 = fail; 失败原因 = reservation 不匹配或被其他 hart 清掉
//
// gva → pa 由 caller (interpreter / 未来 translator) 走 lsu/mmu 路径换好再传; helper
// 本身不感知 TLB / mmu walker (跟 lsu.c store_helper 同抽象层 — HVA/PA based, 不管 GVA)。
// ----------------------------------------------------------------------------
uxlen_t lrsc_lr_w(cpu_t *hart, uxlen_t pa);
uxlen_t lrsc_sc_w(cpu_t *hart, uxlen_t pa, uxlen_t value);

// ----------------------------------------------------------------------------
// 普通 store / AMO 末调 (T2 期间 lsu / amo 埋 hook; T3 填实)
//
// 语义: 该 PA 被写入 → 扫所有 hart, 凡 reservation 命中本 PA word 的全清.
// T1 空体: (void)hart; (void)pa; NOP. lsu/amo 埋调点也不影响 T1 路径正确性.
// ----------------------------------------------------------------------------
void lrsc_on_store(cpu_t *hart, uint32_t pa);

// ----------------------------------------------------------------------------
// device DMA / virtio_blk 写 RAM 末调 — blanket clear (不知 PA 粒度, 全清所有 hart 的
// reservation; DMA 频率低 us+ 量级, 全清不亏)。T3 填实, T1 空体.
// ----------------------------------------------------------------------------
void lrsc_on_device_write(void);

#endif //ISA_LRSC_H
