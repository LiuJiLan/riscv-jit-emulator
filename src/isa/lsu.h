//
// Created by liujilan on 2026/5/5.
// a_01_6 isa/lsu —— RV32 load/store ISA helpers (Spike `riscv/insns/{load,store}.h` 概念对应)。
//
// 不对称设计 (file_plan §8.interpreter D 区, dummy.txt §1 末段):
//   - load_helper:  static inline 在本头文件, caller (interpreter / 未来 translator emit-equiv)
//                    内联进 fast path。BARE 路径直接 host load; SV32 + TLB hit 也是 inline
//                    命中, 只有 miss 才走 walker_helper_load slow path (a_01_7+ 加)。
//   - store_helper: extern 函数在 lsu.c, 总是走 helper call (slow path)。
//                    理由: LR/SC reservation 清除 / 未来 SMC 检测 / 未来副作用扩展, 都需 helper
//                    介入; store fast path inline 化是 file_plan 待办的未来改进。
//   interpreter 与 (未来) translator 都遵守这条不对称, 完整背景见 dummy.txt §1 末段。
//
// trap 协议 (dummy.txt §1 路径 2a, helper 长跳):
//   - misalign:    trap_raise_exception(hart, 4 load misalign / 6 store misalign, gva)
//   - access fault: trap_raise_exception(hart, 5 load access  / 7 store access, gva)
//   - SV32 page fault (a_01_7+): trap_raise_exception(hart, 13 load page / 15 store page, gva)
//   helper 内 trap_raise_exception 是 _Noreturn longjmp, 不返回 caller; caller 不需要 goto out
//   (但保留无害, 跟 OP_UNSUPPORTED case 同形态)。
//
// misalign 处理 (Spike 风格):
//   RV spec 允许实现选择: (a) 硬件支持 misalign (慢但成功) 或 (b) trap (cause 4/6)。
//   项目选 (b) trap 路径, 实现简单不需要拆字节; LR/SC + AMO 强制对齐本来就是这条路径。
//   QEMU 默认 (a) split, Spike 默认 (b) trap, 我们跟 Spike。
//
// 跨页 (a_01_6 BARE):
//   BARE 路径下 RAM 是 mmap 连续区域, 跨 4K 边界本身 OK; 跨 RAM 边界 (RAM 末端 vs 后续地址不
//   在 RAM 区) 走 access fault (cause 5/7), 不拆字节。
//   a_01_7 SV32 加进来时, 跨 PTE 边界化简为 trap (具体 cause 等真做时拍, 主流模拟器都允许
//   这种简化; RV spec 也不强制要求支持跨页)。
//

#ifndef ISA_LSU_H
#define ISA_LSU_H

#include <stdint.h>
#include <stdio.h>      // fprintf (BARE access fault / SV32 占位 提示)
#include <string.h>     // memcpy: 防 strict-aliasing / unaligned 风险

#include "config.h"          // GUEST_RAM_START / GUEST_RAM_SIZE / TLB_NUM_ENTRIES
#include "core/cpu.h"        // cpu_t
#include "core/mmu.h"        // mmu_walker_helper_load (a_01_8 Step 7 SV32 fall back)
#include "core/tlb.h"        // tlb_t (current_tlb 参数类型)
#include "core/trap.h"       // trap_raise_exception (_Noreturn longjmp)
#include "platform/ram.h"    // gpa_to_hva_offset (BARE host load 解地址)
#include "riscv.h"           // CAUSE_LOAD_* (Exception Code 宏) + PTE_* 位


// ----------------------------------------------------------------------------
// load_helper —— RV load 指令的统一 helper (static inline, fast path)
//
// 调用方: interpreter.c 5 load case + (未来) JIT translator emit。
//
// 参数:
//   hart        — 调用 hart
//   current_tlb — NULL = REGIME_BARE (Trust 不走 TLB);
//                 非 NULL = REGIME_SV32, dispatcher 选定的叶 TLB (a_01_7+ 真用)
//   gva         — guest 虚拟地址 = ea (READ_REG(rs1) + imm)
//   size        — 1 / 2 / 4 (LB/LBU = 1, LH/LHU = 2, LW = 4)
//
// 返回: 32 位 host load 结果, 低 size 字节有效, 高位 0 (memcpy 到预初始化 0 的 buffer)。
//        sext/zext 由 caller 自做 (方案 A, helper 不知 signed):
//   case OP_LB:  WRITE_REG(d.rd, (uint32_t)(int32_t)(int8_t)  load_helper(hart, ct, ea, 1));
//   case OP_LH:  WRITE_REG(d.rd, (uint32_t)(int32_t)(int16_t) load_helper(hart, ct, ea, 2));
//   case OP_LW:  WRITE_REG(d.rd,                              load_helper(hart, ct, ea, 4));
//   case OP_LBU: WRITE_REG(d.rd,                              load_helper(hart, ct, ea, 1));
//   case OP_LHU: WRITE_REG(d.rd,                              load_helper(hart, ct, ea, 2));
//
// 错误 (longjmp 走 trap_raise_exception, 不返回 caller):
//   - misalign  → trap_raise(4, gva)
//   - 不在 RAM (a_01_6 没接 bus_dispatch) → fprintf + trap_raise(5, gva)
//   - SV32 占位 (a_01_6 不可达, 因 priv 恒 M; 形式上保接口对齐) → fprintf + trap_raise(13, gva)
// ----------------------------------------------------------------------------
static inline uint32_t load_helper(cpu_t *hart, tlb_t *current_tlb,
                                   uint32_t gva, uint32_t size) {
    // Step 1: misalign (Spike 风格严格对齐)。size=1 时 mask=0, LB/LBU 永远过。
    if ((gva & (size - 1u)) != 0u) {
        trap_raise_exception(hart, CAUSE_LOAD_ADDR_MISALIGNED, /*tval*/gva);  // _Noreturn longjmp
    }

    // Step 2: REGIME_BARE (current_tlb == NULL) — identity + RAM 检查 + host load
    if (current_tlb == NULL) {
        uint32_t pa = gva;  // identity
        // RAM 区检查 (无符号下溢比较, 跟 mmu.c pa_to_fetch_hva 一致)。
        if ((uint32_t)(pa - GUEST_RAM_START) >= GUEST_RAM_SIZE) {
            // PA 不在 RAM 区。a_01_6 bus_dispatch 未实现, fprintf 提示 + trap access fault。
            // 未来 a_01_7+ 加 platform/bus.c 后, 这里改为 bus_dispatch 路径 (MMIO load 走 bus);
            // 现在一律 cause 5 (Load Access Fault)。
            fprintf(stderr,
                    "[lsu] load: PA 0x%08x not in RAM (MMIO bus_dispatch not implemented in a_01_6)\n",
                    pa);
            trap_raise_exception(hart, CAUSE_LOAD_ACCESS_FAULT, /*tval*/gva);  // _Noreturn longjmp
        }

        uint8_t *host_ptr = gpa_to_hva_offset + pa;
        uint32_t value = 0;
        memcpy(&value, host_ptr, size);  // 低 size 字节有效, 高位 0 (value 已初始化为 0)
        return value;
    }

    // Step 3: REGIME_SV32 (current_tlb 非 NULL) — TLB lookup fast path + miss/perm 错 fall back
    //
    // fast path: V + tag + R 全命中 → host load (简化 perm 检查; 不查 priv/SUM/MXR; D18
    // 设计点 5 A 位永远 1 不查; corner case 留 a01_9 解决 — S 模式访问 PTE.U=1 page 没 SUM
    // 时 fast path 可能错放过, 真做 OS 时改用 mmu.h 暴露 check_perm 复用)。
    //
    // 不命中 (miss / V=0 / R=0 / 任何不齐) → fall back mmu_walker_helper_load — walker 内
    // check_perm 做完整 priv+SUM/MXR/X-via-MXR 检查 + 必要时 set A 写回 PT + fill TLB; 失败
    // 时 walker_helper 内 trap_raise_exception (cause 13 page fault / cause 5 access fault)
    // 长跳, 不返回 caller。
    {
        const uint32_t vpn   = gva >> 12;
        const uint32_t index = vpn & (TLB_NUM_ENTRIES - 1);
        tlb_e_t *entry = &current_tlb->e[index];

        /* fast path: V + tag + R 命中 (A 位 walker 进 TLB 时永远 set, 此处 skip) */
        if ((entry->pte_flags & PTE_V)
            && entry->gva_tag == vpn
            && (entry->pte_flags & PTE_R)) {
            uint8_t *host_ptr = entry->host_ptr + (gva & 0xFFFu);
            uint32_t value = 0;
            memcpy(&value, host_ptr, size);
            return value;
        }
    }
    /* fall back to walker (含完整 perm 检查 + fill TLB; 失败 trap_raise_exception 长跳) */
    return mmu_walker_helper_load(hart, current_tlb, gva, size);
}


// ----------------------------------------------------------------------------
// store_helper —— RV store 指令的统一 helper (extern, slow path)
//
// 详细 doc 见 lsu.c。signature:
//   hart        — 调用 hart
//   current_tlb — NULL = REGIME_BARE; 非 NULL = REGIME_SV32 (a_01_7+ 真用)
//   gva         — guest 虚拟地址 = ea (READ_REG(rs1) + imm)
//   value       — 要 store 的值 (32 位); SB 写 value 低 8 位, SH 写低 16 位, SW 写全部 32 位,
//                  由 size 决定写多少字节 (helper 内部 memcpy size 字节)
//   size        — 1 / 2 / 4 (SB = 1, SH = 2, SW = 4)
//
// 错误 (longjmp 走 trap_raise_exception, 不返回 caller):
//   - misalign  → trap_raise(6 store misalign, gva)
//   - 不在 RAM → fprintf + trap_raise(7 store access, gva)
//   - SV32 占位 → fprintf + trap_raise(15 store page fault, gva)
// ----------------------------------------------------------------------------
void store_helper(cpu_t *hart, tlb_t *current_tlb,
                  uint32_t gva, uint32_t value, uint32_t size);

#endif //ISA_LSU_H
