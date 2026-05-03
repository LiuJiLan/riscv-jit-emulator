//
// Created by liujilan on 2026/4/29.
// a01_2 mmu 模块实现 —— mmu_translate_pc + 内部 pa_to_fetch_hva。
//
// 顶部模块文档见 mmu.h (含 regime 二分 / 错误模型 / trap_raise 职责 / MMIO 行为差异)。
// 报错风格见 src/dummy.txt §5 (mmu_translate_pc 不 fprintf, 错误码就是 RV cause; 调用方
// 在 trap path 里如果想打 log, 可以自己 fprintf)。
//

#include "mmu.h"

#include "config.h"
#include "platform/ram.h"
#include "riscv.h"
#include "tlb.h"

#include <stddef.h>   // NULL (D25 后 if (current_tlb == NULL) 第一次用到)
#include <stdint.h>

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
            return 1;               // RV cause 1 = Instruction Access Fault
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
                return 12;          // RV cause 12 = Instruction Page Fault (X 不允许)
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
    //   if (ret != 0) return 12;     // page fault
    //
    // a_01 占位: 直接 return 12 (本来就走不到 SV32 分支, 因为 priv 恒 M)
    return 12;

    // (Step 3-5 在 walker 接入时一并填; a_01 不会执行到)
}
