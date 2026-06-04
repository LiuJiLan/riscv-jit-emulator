//
// Created by liujilan on 2026/6/4.
// isa/lrsc —— RV32 A 扩展 LR.W / SC.W (Zalrsc) + 全局 reservation 数据结构。
//
// 实装形态 (主方案 c, 决议见 trade_off_log §T LR/SC + lrsc_amo_decision.md):
//   - per-hart _Atomic uxlen_t reservation_addr[MAX_HARTS] static 数组 (cap 分配,
//     n_harts 扫; 哨兵 LRSC_INVALID_ADDR = ~(uxlen_t)0). 字段挂 lrsc.c 内不嵌
//     cpu_t (跟 PLIC plic_ctx_eip / CLINT mtimecmp/msip 体例对齐; 决策表 #39 +
//     dummy.txt §7 "shared 数据封装在自己模块内 + cacheline 友好").
//   - global pthread_mutex_t lrsc_buckets[K] (K = 1 << LRSC_BUCKET_BITS = 64,
//     config.h). bucket_lock(pa) inline static Fibonacci hash (Q17/Q18).
//   - LR.W lock-free: memcpy *pa (plain RAM read, 跟 lsu_load_helper 同体例) +
//     atomic_store self.reservation = pa & ~3u (reservation 字段 atomic, 跨 hart 协议状态)
//   - SC.W 锁内 (按 spec line 59-60 "Regardless of success or failure, executing
//     an SC.W instruction invalidates any reservation held by this hart"):
//       不匹配 → atomic_store self.res=INVALID + 释锁 + 返 1
//       匹配   → memcpy *pa ← value (plain RAM write, 跟 lsu store_helper 同体例) +
//                扫所有 hart 清匹配 (含自己, 即清自己) + 释锁 + 返 0
//   *pa access 走 plain memcpy 而非 atomic: LR/SC atomicity 由 reservation 协议 +
//   bucket lock 保证 (不像 amo apply RMW 必须 host-atomic), 跟 lsu plain RAM 体例对齐
//   - 普通 store / AMO 锁内 (lrsc_on_store): 扫所有 hart 清匹配 reservation (含自己;
//     RV spec line 106-107 强制 cross-hart, line 116-118 self-store 是
//     implementation discretion, 我们选清 — Spike/QEMU 同选, 跟 c 主方案 "严格
//     不留 spec 漏口" 宗旨一致, lrsc_amo_decision.md Q5 A3a)
//   - device DMA 锁外 (lrsc_on_device_write): blanket clear, 不查范围全清所有 hart
//     reservation (DMA 频率低 us+ 量级, 全清不亏; 跟 spec line 108-109 强制最小要求
//     一致, 更严更安全)
//
// 七类清除时机 (RV Privileged Spec §14.2.1 "Reserved Memory Locations"):
//   1. context switch / trap → trap_set_*_state 入口
//   2. sfence.vma            → sfence_vma_helper 末段
//   3. wfi 醒来               → wfi_wait 后立刻
//   4. fence.i               → isa/fence.c fence_i_helper
//   5. 普通 store 命中 reservation → lsu.c store_helper 末段调 lrsc_on_store
//   6. AMO 命中 reservation  → amo.c 9 apply 末段调 lrsc_on_store
//   7. device DMA / virtio_blk 写 RAM → io_worker 完成段调 lrsc_on_device_write
//
// trap 协议: 本模块不抛 trap (LR/SC misalign / access fault 在 caller 路径
//   已处理; reservation 操作纯 in-memory data structure 不触发 RV trap)。
//
// 类型规约 (dummy.txt §13):
//   PA = uxlen_t (XLEN-tied; RV32 = uint32_t / RV64 切时一起变 uint64_t)。本文件
//   所有 pa 参数 + reservation_addr 字段 + LRSC_INVALID_ADDR 哨兵均按此体例。
//
// fast/slow path framing 已评估维持 sacred 原则 (trade_off_log §T fast-slow-framing +
//   access_helper_call_graph.md §8). helper call ~5-10 cycle vs lrsc_on_store
//   ~100-300 cycle, call 占 ~3-5%; inline 进 JIT 会失去 host -O3 优化机会
//   (AsmJit 低级 builder, gcc 看不到 runtime emit). apply 层不消除, 跟 sacred fast/
//   slow path 原则一致.
//

#ifndef ISA_LRSC_H
#define ISA_LRSC_H

#include <stddef.h>          // NULL (current_tlb == NULL 编码 REGIME_BARE; inline 顶层用)
#include <stdint.h>

#include "config.h"          // TLB_NUM_ENTRIES (inline 顶层 SV32 TLB hit 用)
#include "core/cpu.h"        // cpu_t
#include "core/mmu.h"        // mmu_walker_helper_lr_w/sc_w + check_perm + MMU_PERM_R/W
#include "core/tlb.h"        // tlb_t / tlb_e_t (SV32 TLB hit fast path)
#include "core/trap.h"       // trap_raise_exception (_Noreturn longjmp; MMIO 拒)
#include "platform/ram.h"    // IS_GPA_RAM / gpa_to_hva_offset
#include "riscv.h"           // uxlen_t / PTE_V / PTE_D / CAUSE_LOAD/STORE_ACCESS_FAULT


// ----------------------------------------------------------------------------
// LRSC_INVALID_ADDR —— reservation 哨兵 (lrsc_amo_decision.md Q1)
//
// ~(uxlen_t)0 = 全 1 (RV32: 0xFFFFFFFFu; RV64: 0xFFFFFFFF_FFFFFFFFu). guest RAM
// 起点 GUEST_RAM_START=0x80000000 + size 128MB 上限远低于全 1, 任何 host 都不可能
// 真有 PA 全 1, 永远非法 — 不留 RV64 review 债 (用字面 0xFFFFFFFFu 在 RV64 下
// 是合法 PA, 会撞).
// ----------------------------------------------------------------------------
#define LRSC_INVALID_ADDR    (~(uxlen_t)0)


// ----------------------------------------------------------------------------
// 生命周期 (POR / teardown 由 main 调)
//
// lrsc_init  — bucket 锁数组 init (按 cap LRSC_BUCKET_NUM 全 init) + per-hart
//              reservation 初始 INVALID (按 cap 全清, lifecycle 配对体例).
// lrsc_destroy — bucket 锁 destroy (cap 配对). lrsc 模块无辅助线程, 没有 join 路径,
//              destroy 纯 cleanup (谁 spawn 谁 join, dummy.txt §12).
// ----------------------------------------------------------------------------
void lrsc_init(void);
void lrsc_destroy(void);

// ----------------------------------------------------------------------------
// lrsc_clear_self —— 清当前 hart 自己的 reservation (lock-free, 自己写自己字段)
//
// 调用点 (七类 #1-#4):
//   - fence.i             (fence.c fence_i_helper)
//   - trap_set_*_state    (trap.c)
//   - sfence_vma_helper   (sfence.c)
//   - wfi_wait 醒来        (interpreter.c case OP_WFI; wfi_wait 用 hartid 不持 cpu_t,
//                          清调点放 interpreter caller 端更干净不污染 wfi.h 接口)
//
// lock-free 理由: 自己写自己字段 atomic_store release, 其它路径 atomic_load acquire
// 读 — 跟 store/SC/on_store 的"扫所有 hart 清匹配"路径无 happens-before 关系
// (清 INVALID 是单向, 不参与 RV 内存模型 ordering; spec implementation discretion
// 允许多个时机 redundant 清).
// ----------------------------------------------------------------------------
void lrsc_clear_self(cpu_t *hart);

// ----------------------------------------------------------------------------
// LR.W / SC.W 指令入口
//
// 签名 (PA-based, 跟 lsu.c store_helper HVA-based 同抽象层 — caller 已走
// lsu/mmu 路径转换 + perm/MMIO check):
//   lr_w: 返 *pa 32-bit 值 (zero-ext 到 uxlen_t); 副作用: self.reservation = pa & ~3u.
//         lock-free (LR-vs-Store race spec line 142-152 允许两种 linearization).
//   sc_w: 返 0 = success / 1 = fail. 拿 bucket lock + 比对 self.reservation:
//         不匹配 atomic_store INVALID + return 1; 匹配 atomic_store *pa = value +
//         扫所有 hart 清匹配 reservation (含自己, 自然清自己) + return 0.
//         无论成败 self.reservation 都被清成 INVALID (spec line 59-60).
//
// caller 已保证 pa 字对齐 (case 入口 LR/SC_MISALIGN_CHECK cause 4/6) + 在 RAM 区
// (MMIO 已 trap cause 5/7); 内部 GPA → HVA 用 gpa_to_hva_offset.
// ----------------------------------------------------------------------------
uxlen_t lrsc_lr_w(cpu_t *hart, uxlen_t pa);
uxlen_t lrsc_sc_w(cpu_t *hart, uxlen_t pa, uxlen_t value);

// ----------------------------------------------------------------------------
// lrsc_on_store —— 普通 store / AMO 末调 (caller 端: lsu.c store_helper +
// amo.c 9 apply)
//
// 语义: 该 PA 被写入 → 拿 bucket lock + 扫所有 hart, 凡 reservation 命中本 PA word
// (= pa & ~3u) 的全清 (含自己, RV spec line 116-118 implementation discretion;
// 跟 Spike/QEMU 同选). cap MAX_HARTS 数组但扫 n_harts 范围 (dummy.txt §15).
// ----------------------------------------------------------------------------
void lrsc_on_store(cpu_t *hart, uxlen_t pa);

// ----------------------------------------------------------------------------
// lrsc_on_device_write —— device DMA / virtio_blk 写 RAM 末调
//
// blanket clear: 不知 PA 粒度 (device 可能写整 sector), 全清所有 hart reservation
// (按 n_harts 扫). lock-free (单向 atomic_store INVALID 写不读, 无 race; SC 内
// 拿 bucket 锁后 atomic_load 看 INVALID 失败, ok).
//
// 接通点: virtio_blk io_worker pread/pwrite 完成后调 (T4 接).
// 其它 device (CLINT/PLIC/UART/test_dev) 都 MMIO 区不写 RAM, 不需 hook.
// ----------------------------------------------------------------------------
void lrsc_on_device_write(void);


// ----------------------------------------------------------------------------
// lrsc_lr_helper / lrsc_sc_helper —— RV LR.W / SC.W 指令的顶层 inline helper
//                                     (interpreter / 未来 translator 直调)
//
// 跟 lsu_load_helper / amo_xxx_helper 严格对偶 (BARE/SV32 分流 + TLB hit fast path):
//   BARE: IS_GPA_RAM 命中 → 直调 lrsc_lr_w/lrsc_sc_w; MMIO 拒 trap_raise(5/7)
//   SV32: TLB hit (V + tag + perm; LR 需 R, SC 需 W+D) → 调 lrsc_lr_w/lrsc_sc_w
//         miss / 权限不齐 / D=0 (SC) → mmu_walker_helper_lr_w/sc_w
//
// 参数:
//   hart        - 调用 hart
//   current_tlb - NULL = REGIME_BARE / 非 NULL = REGIME_SV32 (dispatcher 选定叶 TLB)
//   gva         - guest 虚拟地址 = ea = READ_REG(rs1); LR/SC 无 imm offset; caller 已查
//                  misalign (LR cause 4 / SC cause 6)
//   value (SC)  - rs2 寄存器值, SC 写入 *pa
//
// 返回:
//   LR: zero-ext(*pa) 32-bit 值
//   SC: 0 = success / 1 = fail
//
// 错误 (longjmp 走 trap_raise_exception, 不返回 caller):
//   - misalign            → caller 已查
//   - BARE PA 不在 RAM    → trap_raise(5 for LR / 7 for SC, gva)
//   - SV32 walker fault   → mmu_walker_helper_lr_w/sc_w 内 trap_raise(15/5/7, gva)
//
// memory_order: 全 seq_cst (Q11; aq/rl 精确化推迟 plan §2 #8).
// ----------------------------------------------------------------------------
static inline uxlen_t lrsc_lr_helper(cpu_t *hart, tlb_t *current_tlb, uxlen_t gva) {
    /* misalign 由 caller (interpreter case LOAD_MISALIGN_CHECK) 已查; 隐式契约 */

    if (current_tlb == NULL) {
        /* REGIME_BARE: pa = gva (identity); 内联 RAM/MMIO 分流 */
        if (IS_GPA_RAM(gva)) {
            return lrsc_lr_w(hart, gva);
        }
        /* LR 落 MMIO → cause 5 load access fault (spec implementation-defined, 拒;
         * Spike/QEMU 同; 不开 mmio_lr_helper) */
        trap_raise_exception(hart, CAUSE_LOAD_ACCESS_FAULT, /*tval*/gva);
        return 0;  /* _Noreturn longjmp; 消除编译器警告 */
    }

    /* REGIME_SV32: TLB hit fast path (V + tag + R perm; 跟 lsu_load_helper 同形态) */
    {
        const uint32_t vpn   = gva >> 12;
        const uint32_t index = vpn & (TLB_NUM_ENTRIES - 1);
        tlb_e_t *entry = &current_tlb->e[index];

        if ((entry->pte_flags & PTE_V)
            && entry->gva_tag == vpn
            && check_perm(hart, (uint32_t)entry->pte_flags, MMU_PERM_R)) {
            /* TLB 命中: 直接从 entry->host_ptr 反推 pa = entry->host_ptr -
             * gpa_to_hva_offset + (gva & 0xFFF). 不是 — 更简单: caller 端 BARE 路径
             * 已经直接调 lrsc_lr_w(hart, gva)(因 gva == pa); SV32 命中时 entry->
             * host_ptr 是 page 起点 host 地址, host_ptr + (gva & 0xFFF) 才是真 host
             * 地址 — 但 lrsc_lr_w 需要 PA. 反推:
             *   pa = (uxlen_t)((uintptr_t)(entry->host_ptr + (gva & 0xFFF)) -
             *                  (uintptr_t)gpa_to_hva_offset)
             * 等价于 PA = (PT 上算出的 leaf PA) + (gva & 0xFFF), 跟 walker 一致. */
            uxlen_t pa = (uxlen_t)((uintptr_t)(entry->host_ptr + (gva & 0xFFFu))
                                   - (uintptr_t)gpa_to_hva_offset);
            return lrsc_lr_w(hart, pa);
        }
    }
    /* fall back to walker (含完整 perm 检查 + set A 写回 PT + RAM 调 lrsc_lr_w /
     * MMIO 拒 cause 5 / fill TLB; 失败 trap_raise 长跳) */
    return mmu_walker_helper_lr_w(hart, current_tlb, gva);
}

static inline uxlen_t lrsc_sc_helper(cpu_t *hart, tlb_t *current_tlb,
                                     uxlen_t gva, uxlen_t value) {
    if (current_tlb == NULL) {
        /* REGIME_BARE */
        if (IS_GPA_RAM(gva)) {
            return lrsc_sc_w(hart, gva, value);
        }
        /* SC 落 MMIO → cause 7 store access fault (跟 AMO/store walker 同) */
        trap_raise_exception(hart, CAUSE_STORE_ACCESS_FAULT, /*tval*/gva);
        return 0;
    }

    /* REGIME_SV32: TLB hit fast path (V + tag + D + W perm; 跟 lsu_store_helper /
     * amo_xxx_helper SV32 段同形态 — SC 是 store 类需要 W 权限 + D=1) */
    {
        const uint32_t vpn   = gva >> 12;
        const uint32_t index = vpn & (TLB_NUM_ENTRIES - 1);
        tlb_e_t *entry = &current_tlb->e[index];

        if ((entry->pte_flags & PTE_V)
            && entry->gva_tag == vpn
            && (entry->pte_flags & PTE_D)
            && check_perm(hart, (uint32_t)entry->pte_flags, MMU_PERM_W)) {
            uxlen_t pa = (uxlen_t)((uintptr_t)(entry->host_ptr + (gva & 0xFFFu))
                                   - (uintptr_t)gpa_to_hva_offset);
            return lrsc_sc_w(hart, pa, value);
        }
    }
    return mmu_walker_helper_sc_w(hart, current_tlb, gva, value);
}

#endif //ISA_LRSC_H
