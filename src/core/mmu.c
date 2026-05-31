//
// Created by liujilan on 2026/4/29.
// mmu 模块实现 —— mmu_translate_pc + 内部 pa_to_fetch_hva + SV32 walker
// (mmu_walk) + walker_helper_load/store + hw-managed A/D。
//
// 顶部模块文档见 mmu.h (含 regime 二分 / 错误模型 / trap_raise 职责 / MMIO 行为差异 +
// SV32 walker 设计 / hw-managed A/D / TLB fall back / G-agnostic / 立即生效)。
// 报错风格见 src/dummy.txt §5 (mmu_translate_pc 不 fprintf, 错误码就是 RV cause);
// walker_helper_* 在 PA 不在 RAM 时走 mmio_*_helper (MMIO 派发, 见 dummy.txt §8 / §9);
// RAM 时直接 *hva (load) / 调 store_helper(hva,...) (store), 跟 §8 三层模型一致。
//

#include "mmu.h"

#include "config.h"
#include "isa/lsu.h"        // store_helper (HVA-based, RAM 写 + LR/SC + SMC 副作用)
#include "platform/bus.h"   // mmio_read/write_helper (MMIO 派发, _Noreturn-on-failure)
#include "platform/ram.h"
#include "riscv.h"
#include "tlb.h"
#include "trap.h"     // trap_set_exception_state (路径 2b) + trap_raise_exception (路径 2a, walker helpers)

#include <stdatomic.h>  // atomic_fetch_or_explicit (PTE A/D set 多 hart safe)
#include <stddef.h>   // NULL (current_tlb == NULL 编码 REGIME_BARE)
#include <stdint.h>
#include <stdio.h>    // fprintf (PT 物理地址越界 占位提示)
#include <string.h>   // memcpy (walker 读 PT, walker_helper_* host load/store)

// check_perm 是 mmu.h 的 static inline 函数 (三处共用: mmu_translate_pc /
// lsu_load_helper / lsu_store_helper); 详见 mmu.h check_perm 段 doc。
// 本文件直接调用, 不另起 forward decl。


// ----------------------------------------------------------------------------
// pa_to_fetch_hva —— PA → 取指 HVA
//
// 给定 PA, 返回该地址在 host 端能直接读字节的 HVA。失败说明 PA 不在任何"可执行物理区域",
// 调用方 (mmu_translate_pc) 应据此报 Instruction Access Fault。
//
// 当前只认 RAM 区。未来 platform/rom.c 进来后, 这里加 ROM 分支即可,
// 调用方 (mmu_translate_pc) 不需要改。
//
// 这个函数与 mmu_walker_load/store/amo 的 "PA → 访问" helper 不能合并 —— 三方语义不同:
//   fetch:  RAM ✓ ROM ✓ 其它 → access fault (取指不能 bus_dispatch)
//   load:   RAM ✓ ROM ✓ 其它 → bus_dispatch (load 可走 MMIO)
//   store:  RAM ✓ ROM × (写 ROM 是 access fault) 其它 → bus_dispatch
// 所以三个 helper 分别命名 + 各自实现, 不强求统一。
// ----------------------------------------------------------------------------
static int pa_to_fetch_hva(uxlen_t pa, uint8_t **hva_out) {
    // RAM 区检查 (IS_GPA_RAM 见 ram.h)
    if (IS_GPA_RAM(pa)) {
        *hva_out = gpa_to_hva_offset + pa;
        return 0;
    }

    // 未来 ROM 区 (当前未实现):
    //   if ((uint32_t)(pa - ROM_START) < ROM_SIZE) {
    //       *hva_out = gpa_to_rom_offset + pa;
    //       return 0;
    //   }

    return -1;  // PA 不在任何可执行区域 → 调用方 raise access fault
}

int mmu_translate_pc(cpu_t *hart, tlb_t *current_tlb,
                     uxlen_t *pa_out, uint8_t **hva_out) {
    uxlen_t gva = hart->regs[0];    // pc (cpu.h: regs[0] 物理位置存 pc, x0 走特殊路径)

    // ========================================================================
    // REGIME_BARE (Trust): current_tlb == NULL, bypass TLB, identity
    //
    // M-mode 或任何 priv 带 satp.MODE == bare 都走这里。规范层面 bare 没 PTE 没权限语义,
    // M-mode 也不查权限位。real CPU 在 bare 下也 bypass MMU/TLB, 我们对齐。
    //
    // 用 current_tlb == NULL 作为 REGIME_BARE 的判定 (regime_t enum 不作函数参数,
    // 详见 mmu.h regime_t doc 段)。
    // ========================================================================
    if (current_tlb == NULL) {
        uxlen_t pa = gva;           // identity 映射
        if (pa_to_fetch_hva(pa, hva_out) != 0) {
            // 直调 trap_set_exception_state (dummy.txt §1 路径 2b, mmu_translate_pc 不长跳);
            // cause=1 (Instruction Access Fault, RV spec §3.1.16 cause table); tval=fetch GVA。
            // 返回 trap_set_exception_state 返值给 dispatcher (0/非0 信号; dispatcher continue 让 while 兜底)。
            return (int)trap_set_exception_state(hart, CAUSE_INST_ACCESS_FAULT, /*tval*/gva);
        }
        *pa_out = pa;
        return 0;
    }

    // ========================================================================
    // REGIME_SV32 (Checked): TLB + walker + PTE 权限检查
    //
    // 流程:
    //   1. 试图命中 current_tlb (TLB hit fast path; check_perm 复用 walker 逻辑)
    //   2. miss → mmu_walk(MMU_PERM_X) → pa + pte_flags + pte_pa
    //      (walker 不写回 PT, 输出 leaf PTE 的 PA; caller 在 RAM 路径 atomic OR)
    //      - walk 失败 → trap_set_exception_state(fault_cause, gva); fault_cause 是 walker 选的
    //        (page fault 12 / access fault 1)
    //   3. PA 在 RAM 区检查 (用 pa_to_fetch_hva, 跟 BARE 路径同; 取指不能从 MMIO 拿, 不
    //      在 RAM → access fault cause 1; 未来 ROM 接入时 pa_to_fetch_hva 内加 ROM 分支)
    //   3.5 RAM 路径写回 PTE.A (MMIO 取指上面已 access fault 走掉, 不到此点 — 跟
    //       Spike "fail 不 set" 一致)
    //   4. fill TLB (lazy refresh: 在 ram check 后, 即将"成功"前 fill)
    //   5. 返回 PA + HVA
    //
    // 跟 mmu_walker_helper_load/store 不同: 这里 trap 用 trap_set_exception_state (路径 2b 不长跳),
    // 因为 mmu_translate_pc 在 dispatcher 主循环帧内 return 给 dispatcher 接管即可 (跟
    // BARE 路径风格一致); walker_helper_* 在解释器深栈用 trap_raise_exception 长跳。
    // ========================================================================

    // Step 1: 命中尝试 current_tlb (TLB hit fast path)
    //
    // A 位检查冗余: walker 进 TLB 时永远 set A=1, fast path 检查永远过, 可 skip;
    // check_perm 内不查 A。check_perm 复用 walker 内 perm 逻辑 — fetch perm 检查跟
    // walker 内 fetch 路径不重写两份。
    {
        const uint32_t vpn = gva >> 12;
        const uint32_t index = vpn & (TLB_NUM_ENTRIES - 1);
        tlb_e_t *entry = &current_tlb->e[index];

        if ((entry->pte_flags & PTE_V) && entry->gva_tag == vpn) {
            // tag + V 命中 → fetch perm 检查 (X + priv/PTE_U + SUM)
            if (!check_perm(hart, (uint32_t)entry->pte_flags, MMU_PERM_X)) {
                return (int)trap_set_exception_state(hart, CAUSE_INST_PAGE_FAULT, /*tval*/gva);
            }
            uint8_t *hva = entry->host_ptr + (gva & 0xFFFu);
            *hva_out = hva;
            // PA 由 HVA 反推 (TLB 只缓存 RAM, gpa_to_hva_offset 一定有效)
            *pa_out = (uxlen_t)((uintptr_t)hva - (uintptr_t)gpa_to_hva_offset);
            return 0;
        }
        // 未命中: tag 不匹配 或 V = 0 → 走 Step 2
    }

    // Step 2: miss → mmu_walk
    {
        uxlen_t  pa;
        u32_t    pte_flags;
        uint32_t fault_cause;
        uxlen_t  pte_pa;
        if (mmu_walk(hart, gva, MMU_PERM_X, &pa, &pte_flags, &fault_cause,
                     &pte_pa) != 0) {
            /* walker 失败 cause: X 路径 page fault 12 (V=0 / perm 错 / superpage misaligned),
             * 或 access fault 1 (PT 不在 RAM, 未来 PMP 路径) */
            return (int)trap_set_exception_state(hart, fault_cause, /*tval*/gva);
        }

        // Step 3: PA 在 RAM 区检查 (取指不能从 MMIO 拿; 用 pa_to_fetch_hva 跟 BARE 路径同)
        uint8_t *hva;
        if (pa_to_fetch_hva(pa, &hva) != 0) {
            /* pa_to_fetch_hva 失败时 PTE.A 还没 set (caller 在 RAM 路径才 set;
             * 这里失败走 trap 跟 Spike "fetch MMIO fail 不 set PTE.A" 一致) */
            return (int)trap_set_exception_state(hart, CAUSE_INST_ACCESS_FAULT, /*tval*/gva);
        }

        // Step 3.5: RAM 路径已通过 (pa_to_fetch_hva), 现在 atomic set PTE.A
        // (memory_order_relaxed: PTE A/D set 跟 guest 数据访问无 happens-before 依赖;
        //  多 hart 并发 walk 同 PTE 时 atomic_fetch_or 保留所有 set bit 不丢失。
        //  MMIO 路径上面已 access fault 走掉, 不会到这里, 所以这里写回安全)
        atomic_fetch_or_explicit((_Atomic uint32_t *)(gpa_to_hva_offset + pte_pa),
                                 PTE_A, memory_order_relaxed);

        // Step 4: fill TLB (lazy refresh; 在 ram check 后, return 0 之前 — 副作用要在
        // 即将"成功" 前 fill)
        const uint32_t vpn   = gva >> 12;
        const uint32_t index = vpn & (TLB_NUM_ENTRIES - 1);
        tlb_e_t *entry = &current_tlb->e[index];
        entry->gva_tag   = vpn;
        entry->pte_flags = (uint16_t)pte_flags;
        entry->host_ptr  = hva - (gva & 0xFFFu);    /* page 起点 host 地址 */

        // Step 5: 返回 PA + HVA
        *pa_out  = pa;
        *hva_out = hva;
        return 0;
    }
}


// ============================================================================
// SV32 walker —— pf_cause_for / af_cause_for / mmu_walk +
//                mmu_walker_helper_load / mmu_walker_helper_store
//
// 设计 doc 见 mmu.h "SV32 walker" 段 (含 fast/slow path 关系图, hw-managed A/D 时序场景,
// TLB fall back 风格, G-agnostic, 立即生效, PT-not-in-RAM access fault 防御)。
// ============================================================================

// ---- pf_cause_for / af_cause_for —— 按 perm 选 page-fault / access-fault cause ----
//
// 用 file-static 函数 + switch on mmu_perm_t 而不是 ?: 链, 因为 -Wswitch-enum 强制
// 列全 case (未来加 perm 类型时编译器逼着补 case, 不漏)。

static uint32_t pf_cause_for(mmu_perm_t perm) {
    switch (perm) {
        case MMU_PERM_R: return CAUSE_LOAD_PAGE_FAULT;
        case MMU_PERM_W: return CAUSE_STORE_PAGE_FAULT;
        case MMU_PERM_X: return CAUSE_INST_PAGE_FAULT;
    }
    return 0;  // -Wswitch-enum 下不可达; return 防 -Wreturn-type
}

static uint32_t af_cause_for(mmu_perm_t perm) {
    switch (perm) {
        case MMU_PERM_R: return CAUSE_LOAD_ACCESS_FAULT;
        case MMU_PERM_W: return CAUSE_STORE_ACCESS_FAULT;
        case MMU_PERM_X: return CAUSE_INST_ACCESS_FAULT;
    }
    return 0;
}


// check_perm 是 mmu.h 的 static inline 函数; 本文件 mmu_walk + mmu_translate_pc 都通过
// mmu.h 的 inline 函数调用。


// ============================================================================
// mmu_walk —— SV32 walker (设计见 mmu.h doc)
// ============================================================================
//
// 三级 (实际 2 级) walk: level=1 → leaf (4MB superpage) 或 next-level → level=0 → leaf
// (4KB)。失败 return -1 + 填 fault_cause_out (page fault / access fault); 成功 return 0
// + 填 pa_out + pte_flags_out (含建议 set 后的 A/D) + pte_pa_out (leaf PTE 在 PT
// 的 PA, caller atomic OR 用)。
//
// hw-managed A/D (RV Spec 非 Svade): walker 内**不真写回 PT**, 只输出 leaf PTE
// 的 PA (pte_pa_out); caller 在 RAM 路径 (IS_GPA_RAM 通过) 调
// atomic_fetch_or_explicit(host_pte_addr, set_bits, memory_order_relaxed) 设
// A/D bit (set_bits = PTE_A 或 PTE_A | PTE_D, 按 perm 决定); MMIO 路径不写回
// (跟 Spike "fail 不 set" 一致简化版)。
//
// SMP safe: 多 hart 并发 walk 同一 PTE 时 atomic_fetch_or 保留所有 set bit 不丢
// (plain memcpy 会 race 丢 bit, 详 mmu.h "为什么 atomic_fetch_or" 段)。
// memory_order_relaxed: PTE A/D 跟 guest 数据访问无 happens-before 依赖。

int mmu_walk(cpu_t *hart, uxlen_t gva, mmu_perm_t perm,
             uxlen_t *pa_out, u32_t *pte_flags_out,
             uint32_t *fault_cause_out,
             uxlen_t *pte_pa_out) {
    /* 默认 pte_pa_out 由 leaf 分支覆盖; 失败路径不读 (caller 走 trap) */
    *pte_pa_out = 0;

    const uint32_t pf_cause = pf_cause_for(perm);
    const uint32_t af_cause = af_cause_for(perm);

    // satp 拆段 (项目仅支持 satp.MODE = 0/1; walker 不应在 BARE 路径调; 信任 caller)
    const uxlen_t  satp = hart->satp;
    const uint32_t root_ppn = satp & 0x3FFFFFu;          /* 22 位 PPN */
    const uxlen_t  root_pa  = root_ppn << 12;

    // SV32 VA 拆 (10|10|12)
    const uint32_t vpn1   = (gva >> 22) & 0x3FFu;
    const uint32_t vpn0   = (gva >> 12) & 0x3FFu;
    const uint32_t offset = gva & 0xFFFu;

    // ---- level=1 walk ----
    const uxlen_t  pte1_pa = root_pa + (vpn1 << 2);      /* 4 字节每 PTE */
    if (!IS_GPA_RAM(pte1_pa)) {
        /* PT 物理地址不在 RAM (当前不实现 PMP, 用 RAM 区检查代替) → access fault.
         * 未来 PMP 接入: 这里改成 PMP allow 检查 + cause 不变。 */
        *fault_cause_out = af_cause;
        return -1;
    }
    u32_t    pte1;
    memcpy(&pte1, gpa_to_hva_offset + pte1_pa, 4);

    if ((pte1 & PTE_V) == 0) {
        *fault_cause_out = pf_cause;
        return -1;
    }

    if ((pte1 & (PTE_R | PTE_W | PTE_X)) == 0) {
        // ---- pointer-to-next-level → level=0 walk ----
        const uint32_t pte1_full_ppn = (pte1 >> 10);     /* PTE bits[31:10] = PPN 22 位 */
        const uxlen_t  pte1_full_pa  = pte1_full_ppn << 12;

        const uxlen_t  pte0_pa = pte1_full_pa + (vpn0 << 2);
        if (!IS_GPA_RAM(pte0_pa)) {
            *fault_cause_out = af_cause;
            return -1;
        }
        u32_t    pte0;
        memcpy(&pte0, gpa_to_hva_offset + pte0_pa, 4);

        if ((pte0 & PTE_V) == 0) {
            *fault_cause_out = pf_cause;
            return -1;
        }
        if ((pte0 & (PTE_R | PTE_W | PTE_X)) == 0) {
            /* level=0 必须 leaf; non-leaf 是 misformatted PT → page fault */
            *fault_cause_out = pf_cause;
            return -1;
        }

        if (!check_perm(hart, pte0, perm)) {
            *fault_cause_out = pf_cause;
            return -1;
        }

        // hw-managed A/D: walker 只输出 leaf PTE 的 PA; caller 在 RAM 路径调
        // atomic_fetch_or 设 A/D, 多 hart 并发 safe (memory_order_relaxed)。已 set
        // 的 bit 重 OR 无副作用, walker 不再做"需 set vs 已 set"优化判。
        *pte_pa_out = pte0_pa;
        // pte_flags_out 反映建议 set 后状态 (caller TLB 用 pte_flags 的 A/D bit)
        pte0 |= PTE_A;
        if (perm == MMU_PERM_W) { pte0 |= PTE_D; }

        // 算 PA: leaf PPN<<12 | offset
        const uint32_t leaf_ppn = (pte0 >> 10);          /* 22 位 PPN */
        *pa_out         = (leaf_ppn << 12) | offset;
        *pte_flags_out  = pte0 & 0x3FFu;                 /* 低 10 位: V/R/W/X/U/G/A/D + RSW */
        return 0;
    }

    // ---- level=1 leaf (4MB superpage) ----
    //
    // misaligned superpage 检查: PTE.PPN[0] (PTE bits[19:10]) 必须 0 (4MB 物理对齐)。
    const uint32_t pte1_ppn0 = (pte1 >> 10) & 0x3FFu;    /* 10 位 */
    if (pte1_ppn0 != 0) {
        /* misaligned superpage → page fault (RV Spec §4.3.2) */
        *fault_cause_out = pf_cause;
        return -1;
    }

    if (!check_perm(hart, pte1, perm)) {
        *fault_cause_out = pf_cause;
        return -1;
    }

    // hw-managed A/D: superpage 形态跟 level=0 leaf 同 (walker 输出 PA, caller atomic OR)
    *pte_pa_out = pte1_pa;
    pte1 |= PTE_A;
    if (perm == MMU_PERM_W) { pte1 |= PTE_D; }

    // 算 4MB superpage PA: PTE.PPN[1] (12 位) | VPN[0] (来自 vaddr) | offset (12 位)
    const uint32_t pte1_ppn1 = (pte1 >> 20) & 0xFFFu;    /* PTE bits[31:20] = PPN[1] 12 位 */
    *pa_out         = (pte1_ppn1 << 22) | (vpn0 << 12) | offset;
    *pte_flags_out  = pte1 & 0x3FFu;
    return 0;
}


// ============================================================================
// mmu_walker_helper_load —— SV32 load 路径完整流程 (slow path; 长跳风格)
// ============================================================================

uxlen_t mmu_walker_helper_load(cpu_t *hart, tlb_t *current_tlb,
                               uxlen_t gva, uint32_t size) {
    uxlen_t  pa;
    u32_t    pte_flags;
    uint32_t fault_cause;
    uxlen_t  pte_pa;
    if (mmu_walk(hart, gva, MMU_PERM_R, &pa, &pte_flags, &fault_cause,
                 &pte_pa) != 0) {
        trap_raise_exception(hart, fault_cause, /*tval*/gva);   /* _Noreturn longjmp */
    }

    // PA 不在 RAM 区 → MMIO 派发 (不入 TLB, plan §1.4 + dummy.txt §8)
    // mmio_read_helper 内部 _Noreturn-on-failure (未命中 / device 拒绝都 longjmp,
    // 不返回 caller; 见 dummy.txt §9 "0=成功" + cause 0 路径)。
    //
    // MMIO 路径**不写回 PTE.A** — 跟 Spike "fail 不 set" 一致简化版 (MMIO
    // 成功访问也不 set, 因 MMIO PTE 不缓存到 TLB, 每次重 walk 时按 PT 原值,
    // 行为正确; 见 mmu.h hw-managed A/D 段)。
    if (!IS_GPA_RAM(pa)) {
        return mmio_read_helper(hart, pa, gva, size);
    }

    uint8_t *host_ptr       = gpa_to_hva_offset + pa;
    uint8_t *page_host_base = host_ptr - (gva & 0xFFFu);  /* page 起点 host 地址 */

    // RAM 路径 atomic set PTE.A (memory_order_relaxed: 多 hart 并发 walk 同 PTE 时
    // atomic_fetch_or 保留所有 set bit; 不跟 guest 数据访问建立 happens-before)
    atomic_fetch_or_explicit((_Atomic uint32_t *)(gpa_to_hva_offset + pte_pa),
                             PTE_A, memory_order_relaxed);

    // Fill TLB entry (lazy refresh: 在 ram check 后, host load 之前; 副作用要在
    // 即将成功访问时才发生, 避免没成功污染 TLB)
    const uint32_t vpn   = gva >> 12;
    const uint32_t index = vpn & (TLB_NUM_ENTRIES - 1);
    tlb_e_t *entry = &current_tlb->e[index];
    entry->gva_tag   = vpn;
    entry->pte_flags = (uint16_t)pte_flags;
    entry->host_ptr  = page_host_base;

    // host load size 字节 (低 size 字节有效, 高位 0); sext/zext 由 caller 做
    uxlen_t value = 0;
    memcpy(&value, host_ptr, size);
    return value;
}


// ============================================================================
// mmu_walker_helper_store —— SV32 store 路径完整流程 (slow path; 长跳风格)
// ============================================================================

void mmu_walker_helper_store(cpu_t *hart, tlb_t *current_tlb,
                             uxlen_t gva, uxlen_t value, uint32_t size) {
    uxlen_t  pa;
    u32_t    pte_flags;
    uint32_t fault_cause;
    uxlen_t  pte_pa;
    if (mmu_walk(hart, gva, MMU_PERM_W, &pa, &pte_flags, &fault_cause,
                 &pte_pa) != 0) {
        trap_raise_exception(hart, fault_cause, /*tval*/gva);
    }

    // PA 不在 RAM 区 → MMIO 派发 (不入 TLB; plan §1.4 + dummy.txt §8 / §9)
    //
    // MMIO 路径**不写回 PTE.A/D** — 跟 load 路径同形态简化 (MMIO 失败 trap_raise
    // 时 PTE.A/D 不会已 set; 见 mmu.h hw-managed A/D 段)。
    if (!IS_GPA_RAM(pa)) {
        mmio_write_helper(hart, pa, gva, value, size);  // _Noreturn-on-failure
        return;
    }

    uint8_t *host_ptr       = gpa_to_hva_offset + pa;
    uint8_t *page_host_base = host_ptr - (gva & 0xFFFu);

    // RAM 路径 atomic set PTE.A+D (memory_order_relaxed: 多 hart 并发 walk safe;
    // 不跟 guest 数据访问建立 happens-before; 详 walker_helper_load 同位置注释)
    atomic_fetch_or_explicit((_Atomic uint32_t *)(gpa_to_hva_offset + pte_pa),
                             PTE_A | PTE_D, memory_order_relaxed);

    // Fill TLB entry (含建议 set 的 D=1, 跟 load 路径同形态)
    const uint32_t vpn   = gva >> 12;
    const uint32_t index = vpn & (TLB_NUM_ENTRIES - 1);
    tlb_e_t *entry = &current_tlb->e[index];
    entry->gva_tag   = vpn;
    entry->pte_flags = (uint16_t)pte_flags;
    entry->host_ptr  = page_host_base;

    // RAM 写 + LR/SC reservation 清除 + SMC 占位 — 全部委托 store_helper (HVA-based)。
    // store_helper 是唯一统一的 "RAM 写 + 副作用" 入口, lsu_store_helper BARE / SV32 hit
    // / 本 walker_helper RAM 路径三处共用一份逻辑 (见 lsu.h 顶段 load/store 不对称真机理
    // + dummy.txt §1 末段)。MMIO 路径不调 store_helper, 已在上面分流直走 mmio_write_helper。
    store_helper(hart, host_ptr, /*gva for tval*/gva, value, size);
}
