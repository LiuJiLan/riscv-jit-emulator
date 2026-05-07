//
// Created by liujilan on 2026/4/29.
// a01_2 mmu 模块实现 —— mmu_translate_pc + 内部 pa_to_fetch_hva。
// a_01_8 增量: SV32 walker (mmu_walk) + walker_helper_load/store + hw-managed A/D。
//
// 顶部模块文档见 mmu.h (含 regime 二分 / 错误模型 / trap_raise 职责 / MMIO 行为差异 +
// a_01_8 SV32 walker 设计 / hw-managed A/D / TLB fall back / G-agnostic / 立即生效)。
// 报错风格见 src/dummy.txt §5 (mmu_translate_pc 不 fprintf, 错误码就是 RV cause;
// walker_helper_* 在 PA 不在 RAM 时 fprintf 占位提示, 跟 lsu.c 同形态)。
//

#include "mmu.h"

#include "config.h"
#include "platform/ram.h"
#include "riscv.h"
#include "tlb.h"
#include "trap.h"     // trap_set_state (路径 2b) + trap_raise_exception (路径 2a, walker helpers)

#include <stddef.h>   // NULL (D25 后 if (current_tlb == NULL) 第一次用到)
#include <stdint.h>
#include <stdio.h>    // fprintf (a_01_8 walker_helper_* PA 不在 RAM 占位提示)
#include <string.h>   // memcpy (a_01_8 walker 读写 PT, walker_helper_* host load/store)

// ----------------------------------------------------------------------------
// pa_to_fetch_hva —— PA → 取指 HVA
//
// 给定 PA, 返回该地址在 host 端能直接读字节的 HVA。失败说明 PA 不在任何"可执行物理区域",
// 调用方 (mmu_translate_pc) 应据此报 Instruction Access Fault。
//
// 当前 a_01 只认 RAM 区。未来 platform/rom.c 进来后, 这里加 ROM 分支即可,
// 调用方 (mmu_translate_pc) 不需要改。
//
// 这个函数与 mmu_walker_load/store/amo 的 "PA → 访问" helper 不能合并 —— 三方语义不同:
//   fetch:  RAM ✓ ROM ✓ 其它 → access fault (取指不能 bus_dispatch)
//   load:   RAM ✓ ROM ✓ 其它 → bus_dispatch (load 可走 MMIO)
//   store:  RAM ✓ ROM × (写 ROM 是 access fault) 其它 → bus_dispatch
// 所以三个 helper 分别命名 + 各自实现, 不强求统一。
// ----------------------------------------------------------------------------
static int pa_to_fetch_hva(uint32_t pa, uint8_t **hva_out) {
    // RAM 区: 利用无符号下溢比较 (gpa < GUEST_RAM_START 时差值变成很大的 u32, 自然 >= GUEST_RAM_SIZE)
    if ((uint32_t)(pa - GUEST_RAM_START) < GUEST_RAM_SIZE) {
        *hva_out = gpa_to_hva_offset + pa;
        return 0;
    }

    // 未来 ROM 区 (a_01 未实现, 见 file_plan platform/rom.c TODO):
    //   if ((uint32_t)(pa - ROM_START) < ROM_SIZE) {
    //       *hva_out = gpa_to_rom_offset + pa;
    //       return 0;
    //   }

    return -1;  // PA 不在任何可执行区域 → 调用方 raise access fault
}

int mmu_translate_pc(cpu_t *hart, tlb_t *current_tlb,
                     uint32_t *pa_out, uint8_t **hva_out) {
    uint32_t gva = hart->regs[0];   // pc (cpu.h: regs[0] 物理位置存 pc, x0 走特殊路径)

    // ========================================================================
    // REGIME_BARE (Trust): current_tlb == NULL, bypass TLB, identity
    //
    // M-mode 或任何 priv 带 satp.MODE == bare 都走这里。规范层面 bare 没 PTE 没权限语义,
    // M-mode 也不查权限位。real CPU 在 bare 下也 bypass MMU/TLB, 我们对齐。
    //
    // 用 current_tlb == NULL 作为 REGIME_BARE 的判定 (D25; regime_t enum 不作函数参数,
    // 详见 mmu.h regime_t doc 段)。
    // ========================================================================
    if (current_tlb == NULL) {
        uint32_t pa = gva;          // identity 映射
        if (pa_to_fetch_hva(pa, hva_out) != 0) {
            // a_01_5_b: 直调 trap_set_state (dummy.txt §1 路径 2b, mmu_translate_pc 不长跳);
            // cause=1 (Instruction Access Fault, RV spec §3.1.16 cause table); tval=fetch GVA.
            // 返回 in_trap 当前值给 dispatcher (0/非0 信号; dispatcher continue 让 while 兜底)。
            return (int)trap_set_state(hart, CAUSE_INST_ACCESS_FAULT, /*tval*/gva);
        }
        *pa_out = pa;
        return 0;
    }

    // ========================================================================
    // REGIME_SV32 (Checked): TLB + walker + PTE 权限检查
    //
    // 5 步 (a_01 大部分占位; a_05+ Sv32 walker 接入时填):
    //   1. 试图命中 current_tlb
    //   2. miss → mmu_walk 走页表 (a_01: 占位 return 12)
    //   3. PMP / PMA 检查 (a_01: 全开 skip)
    //   4. PA → HVA via pa_to_fetch_hva
    //   5. TLB insert (walker 路径填; entry 内容从 PTE 实际位拷贝)
    // ========================================================================

    // Step 1: 命中尝试 current_tlb
    {
        const uint32_t vpn = gva >> 12;
        const uint32_t index = vpn & (TLB_NUM_ENTRIES - 1);
        tlb_e_t *entry = &current_tlb->e[index];

        if ((entry->pte_flags & PTE_V) && entry->gva_tag == vpn) {
            // tag + V 命中 → 检查取指权限 (X 位)
            // (S/U 还要查 PTE_U + SUM/MXR; 等 a_05+ Sv32 真接入时按 priv emit-bake 加上)
            if ((entry->pte_flags & PTE_X) == 0) {
                // a_01_5_b: 同 BARE 路径, 直调 trap_set_state; cause=12 (Instruction Page Fault,
                // PTE 翻译失败 — X 位不允许); tval=fetch GVA。
                // a_01 不会触发 (priv 恒 M, current_tlb == NULL → 走 BARE 不进 SV32), SV32
                // 路径占位等 a_01_7 SV32 walker 接入时真激活。
                return (int)trap_set_state(hart, CAUSE_INST_PAGE_FAULT, /*tval*/gva);
            }
            uint8_t *hva = entry->host_ptr + (gva & 0xFFFu);
            *hva_out = hva;
            // PA 由 HVA 反推 (TLB 只缓存 RAM, gpa_to_hva_offset 一定有效)
            *pa_out = (uint32_t)((uintptr_t)hva - (uintptr_t)gpa_to_hva_offset);
            return 0;
        }
        // 未命中: tag 不匹配 或 V = 0 → 走 Step 2
    }

    // Step 2: miss → mmu_walk
    //
    // 真上 Sv32 后:
    //   ret = mmu_walk(hart, gva, PERM_X, &pa, &fault_cause);
    //   if (ret != 0) return (int)trap_set_state(hart, fault_cause, gva);  // page fault
    //
    // a_01 占位: 直接走 trap_set_state(12) (本来就走不到 SV32 分支, 因为 priv 恒 M;
    // a_01_5_b 形式上保持接口对齐, 未来 SV32 walker 替换上方那行 mmu_walk 调用即可)。
    return (int)trap_set_state(hart, CAUSE_INST_PAGE_FAULT, /*tval*/gva);

    // (Step 3-5 在 walker 接入时一并填; a_01 不会执行到)
}


// ============================================================================
// a_01_8 SV32 walker —— pf_cause_for / af_cause_for / check_perm + mmu_walk +
//                       mmu_walker_helper_load / mmu_walker_helper_store
//
// 设计 doc 见 mmu.h "SV32 walker (a_01_8)" 段 (含 fast/slow path 关系图, hw-managed A/D
// 时序场景, TLB fall back 风格, G-agnostic, 立即生效, PT-not-in-RAM access fault 防御)。
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


// ---- check_perm —— priv + SUM/MXR + PTE.U/R/W/X 权限检查 (file-static inline) ----
//
// 返回: 1 = 权限通过; 0 = 权限不通过 (caller 报 page fault)
//
// 检查顺序 (RV Privileged Spec Vol II §4.3.1, 4.4):
//   1. priv 跟 PTE.U 匹配:
//      - PRIV_U: 必须 PTE.U=1 (U 模式只能访问 user pages)
//      - PRIV_S: PTE.U=0 默认允许; PTE.U=1 仅 mstatus.SUM=1 + perm != X 时允许
//                (S 模式访问 user pages 受 SUM 控制; 即使 SUM=1, 也不允许 fetch X-on-U)
//      - PRIV_M: walker 不应在 M 调用 (caller 防御); 不特检 (信任 caller)
//   2. R/W/X 位:
//      - load (PERM_R): 需要 R=1, 或 mstatus.MXR=1 + X=1 (X-on-readable)
//      - store (PERM_W): 需要 W=1; W=1+R=0 spec reserved 但项目跟 spike 风格不查 (caller
//                         若构造 W=1+R=0 PTE, 项目当合法; 真做时按 spec 查可补)
//      - fetch (PERM_X): 需要 X=1
//
// 注意: PTE.A/PTE.D 检查不在此函数里 — 项目 hw-managed 风格, walker 内 mmu_walk 顺手
// set A/D 写回 PT (见下方 mmu_walk 实现); 不像 Svade 风格那样把 A=0/D=0 当 fault。

static inline int check_perm(cpu_t *hart, uint32_t pte, mmu_perm_t perm) {
    uint32_t mstatus_lo = (uint32_t)(hart->trap._mstatus & 0xFFFFFFFFu);
    int sum = (mstatus_lo & MSTATUS_SUM) != 0;
    int mxr = (mstatus_lo & MSTATUS_MXR) != 0;
    int pte_u = (pte & PTE_U) != 0;

    // priv + PTE.U 检查
    if (hart->priv == PRIV_U) {
        if (!pte_u) return 0;
    } else if (hart->priv == PRIV_S) {
        if (pte_u) {
            if (!sum) return 0;
            if (perm == MMU_PERM_X) return 0;     /* S+SUM 不允许 X-on-U-page */
        }
    }
    /* PRIV_M: 信任 caller (walker 不应在 M 调用); PRIV_H 槽 a_01_8 永远 NULL 不到此 */

    // R/W/X 位检查 — switch on perm 跟 mmu_perm_t enum 配 -Wswitch-enum 联动
    switch (perm) {
        case MMU_PERM_R:
            if ((pte & PTE_R) == 0 && !(mxr && (pte & PTE_X))) return 0;
            break;
        case MMU_PERM_W:
            if ((pte & PTE_W) == 0) return 0;
            /* W=1+R=0 RV spec reserved; 项目跟 spike 风格不查 */
            break;
        case MMU_PERM_X:
            if ((pte & PTE_X) == 0) return 0;
            break;
    }
    return 1;
}


// ============================================================================
// mmu_walk —— SV32 walker (设计见 mmu.h doc)
// ============================================================================
//
// 三级 (实际 2 级) walk: level=1 → leaf (4MB superpage, decision N) 或 next-level →
// level=0 → leaf (4KB)。失败 return -1 + 填 fault_cause_out (page fault / access fault);
// 成功 return 0 + 填 pa_out + pte_flags_out (含 walker set 后的 A/D)。
//
// hw-managed A/D (RV Spec 非 Svade): walker 内顺手 set A=1 (任何访问) 和 D=1 (W 访问),
// 写回 PT。已 set 不重写 (避免无用写, SMP 时减原子开销)。
//
// SMP atomic 占位: 当前单 hart 用 memcpy 写回 PT; SMP 时改 atomic_fetch_or 跨 hart 同步
// (PTE 位 set 必须 atomic; 多 hart 并发 walk 同 PTE 时不丢 set)。

int mmu_walk(cpu_t *hart, uint32_t gva, mmu_perm_t perm,
             uint32_t *pa_out, uint32_t *pte_flags_out,
             uint32_t *fault_cause_out) {
    const uint32_t pf_cause = pf_cause_for(perm);
    const uint32_t af_cause = af_cause_for(perm);

    // satp 拆段 (a_01_8 项目仅支持 satp.MODE = 0/1; walker 不应在 BARE 路径调; 信任 caller)
    const uint32_t satp = hart->satp;
    const uint32_t root_ppn = satp & 0x3FFFFFu;          /* 22 位 PPN */
    const uint32_t root_pa  = root_ppn << 12;

    // SV32 VA 拆 (10|10|12)
    const uint32_t vpn1   = (gva >> 22) & 0x3FFu;
    const uint32_t vpn0   = (gva >> 12) & 0x3FFu;
    const uint32_t offset = gva & 0xFFFu;

    // ---- level=1 walk ----
    const uint32_t pte1_pa = root_pa + (vpn1 << 2);      /* 4 字节每 PTE */
    if ((uint32_t)(pte1_pa - GUEST_RAM_START) >= GUEST_RAM_SIZE) {
        /* PT 物理地址不在 RAM (a_01 不实现 PMP, 用 RAM 区检查代替) → access fault.
         * 未来 PMP 接入: 这里改成 PMP allow 检查 + cause 不变。 */
        *fault_cause_out = af_cause;
        return -1;
    }
    uint32_t pte1;
    memcpy(&pte1, gpa_to_hva_offset + pte1_pa, 4);

    if ((pte1 & PTE_V) == 0) {
        *fault_cause_out = pf_cause;
        return -1;
    }

    if ((pte1 & (PTE_R | PTE_W | PTE_X)) == 0) {
        // ---- pointer-to-next-level → level=0 walk ----
        const uint32_t pte1_full_ppn = (pte1 >> 10);     /* PTE bits[31:10] = PPN 22 位 */
        const uint32_t pte1_full_pa  = pte1_full_ppn << 12;

        const uint32_t pte0_pa = pte1_full_pa + (vpn0 << 2);
        if ((uint32_t)(pte0_pa - GUEST_RAM_START) >= GUEST_RAM_SIZE) {
            *fault_cause_out = af_cause;
            return -1;
        }
        uint32_t pte0;
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

        // hw-managed A/D set + 写回 PT
        uint32_t new_pte0 = pte0 | PTE_A;
        if (perm == MMU_PERM_W) new_pte0 |= PTE_D;
        if (new_pte0 != pte0) {
            // SMP: 改用 atomic_fetch_or(&pte_in_pt, PTE_A | (W ? PTE_D : 0)) 跨 hart 同步
            memcpy(gpa_to_hva_offset + pte0_pa, &new_pte0, 4);
            pte0 = new_pte0;
        }

        // 算 PA: leaf PPN<<12 | offset
        const uint32_t leaf_ppn = (pte0 >> 10);          /* 22 位 PPN */
        *pa_out         = (leaf_ppn << 12) | offset;
        *pte_flags_out  = pte0 & 0x3FFu;                 /* 低 10 位: V/R/W/X/U/G/A/D + RSW */
        return 0;
    }

    // ---- level=1 leaf (4MB superpage, decision N) ----
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

    // hw-managed A/D set + 写回 PT
    uint32_t new_pte1 = pte1 | PTE_A;
    if (perm == MMU_PERM_W) new_pte1 |= PTE_D;
    if (new_pte1 != pte1) {
        // SMP: atomic_fetch_or
        memcpy(gpa_to_hva_offset + pte1_pa, &new_pte1, 4);
        pte1 = new_pte1;
    }

    // 算 4MB superpage PA: PTE.PPN[1] (12 位) | VPN[0] (来自 vaddr) | offset (12 位)
    const uint32_t pte1_ppn1 = (pte1 >> 20) & 0xFFFu;    /* PTE bits[31:20] = PPN[1] 12 位 */
    *pa_out         = (pte1_ppn1 << 22) | (vpn0 << 12) | offset;
    *pte_flags_out  = pte1 & 0x3FFu;
    return 0;
}


// ============================================================================
// mmu_walker_helper_load —— SV32 load 路径完整流程 (slow path; 长跳风格)
// ============================================================================

uint32_t mmu_walker_helper_load(cpu_t *hart, tlb_t *current_tlb,
                                uint32_t gva, uint32_t size) {
    uint32_t pa, pte_flags, fault_cause;
    if (mmu_walk(hart, gva, MMU_PERM_R, &pa, &pte_flags, &fault_cause) != 0) {
        trap_raise_exception(hart, fault_cause, /*tval*/gva);   /* _Noreturn longjmp */
    }

    // PA 在 RAM 区检查 (a_01 不接 bus_dispatch; PA 不在 RAM 一律 access fault)
    if ((uint32_t)(pa - GUEST_RAM_START) >= GUEST_RAM_SIZE) {
        fprintf(stderr,
                "[mmu_walker_helper_load] PA 0x%08x not in RAM (MMIO bus_dispatch not implemented in a_01_8)\n",
                pa);
        trap_raise_exception(hart, CAUSE_LOAD_ACCESS_FAULT, /*tval*/gva);
    }

    uint8_t *host_ptr       = gpa_to_hva_offset + pa;
    uint8_t *page_host_base = host_ptr - (gva & 0xFFFu);  /* page 起点 host 地址 */

    // Fill TLB entry (D14 lazy refresh: 在 ram check 后, host load 之前; 副作用要在
    // 即将成功访问时才发生, 避免没成功污染 TLB)
    const uint32_t vpn   = gva >> 12;
    const uint32_t index = vpn & (TLB_NUM_ENTRIES - 1);
    tlb_e_t *entry = &current_tlb->e[index];
    entry->gva_tag   = vpn;
    entry->pte_flags = (uint16_t)pte_flags;
    entry->host_ptr  = page_host_base;

    // host load size 字节 (低 size 字节有效, 高位 0); sext/zext 由 caller 做
    uint32_t value = 0;
    memcpy(&value, host_ptr, size);
    return value;
}


// ============================================================================
// mmu_walker_helper_store —— SV32 store 路径完整流程 (slow path; 长跳风格)
// ============================================================================

void mmu_walker_helper_store(cpu_t *hart, tlb_t *current_tlb,
                             uint32_t gva, uint32_t value, uint32_t size) {
    uint32_t pa, pte_flags, fault_cause;
    if (mmu_walk(hart, gva, MMU_PERM_W, &pa, &pte_flags, &fault_cause) != 0) {
        trap_raise_exception(hart, fault_cause, /*tval*/gva);
    }

    if ((uint32_t)(pa - GUEST_RAM_START) >= GUEST_RAM_SIZE) {
        fprintf(stderr,
                "[mmu_walker_helper_store] PA 0x%08x not in RAM (MMIO bus_dispatch not implemented in a_01_8)\n",
                pa);
        trap_raise_exception(hart, CAUSE_STORE_ACCESS_FAULT, /*tval*/gva);
    }

    uint8_t *host_ptr       = gpa_to_hva_offset + pa;
    uint8_t *page_host_base = host_ptr - (gva & 0xFFFu);

    // Fill TLB entry (含 walker set 的 D=1, 跟 load 路径同形态)
    const uint32_t vpn   = gva >> 12;
    const uint32_t index = vpn & (TLB_NUM_ENTRIES - 1);
    tlb_e_t *entry = &current_tlb->e[index];
    entry->gva_tag   = vpn;
    entry->pte_flags = (uint16_t)pte_flags;
    entry->host_ptr  = page_host_base;

    // host store size 字节
    memcpy(host_ptr, &value, size);

    // reservation 清除 (LR/SC) 占位 — 跟 store_helper BARE 路径同形态
    // a_01 没接 LR/SC, reservation_t struct 也未定; 占位等 a_03+ A 扩展真做。
    // SMC page_dirty 检测占位同 store_helper BARE 路径。
}
