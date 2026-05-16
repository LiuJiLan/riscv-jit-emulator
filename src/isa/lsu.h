//
// Created by liujilan on 2026/5/5.
// isa/lsu —— RV32 load/store ISA helpers (Spike `riscv/insns/{load,store}.h` 概念对应)。
//
// ============================================================================
// 调用拓扑 (跟 dummy.txt §8 三层模型一致)
// ============================================================================
//
//   interpreter LOAD case (5 个 LB/LH/LW/LBU/LHU)
//     ├─ misalign check (LOAD_MISALIGN_CHECK 宏, interpreter.c 顶段)
//     └─ lsu_load_helper [inline 顶层]
//          ├─ BARE (current_tlb == NULL):
//          │    IS_GPA_RAM(gva) ? *hva (inline) : mmio_read_helper(...)
//          ├─ SV32 TLB hit (V + tag + check_perm R):
//          │    return *(entry->host_ptr + offset)   ← 直接 *hva, 不调子 helper
//          └─ SV32 miss:
//               mmu_walker_helper_load(...)  [extern; mmu.c]
//
//   interpreter STORE case (3 个 SB/SH/SW)
//     ├─ misalign check (STORE_MISALIGN_CHECK 宏)
//     └─ lsu_store_helper [inline 顶层]
//          ├─ BARE:
//          │    IS_GPA_RAM(gva) ? store_helper(hva, ...) : mmio_write_helper(...)
//          ├─ SV32 TLB hit (V + tag + D + check_perm W):
//          │    store_helper(hva, ...)        ← LR/SC + SMC 副作用必经
//          └─ SV32 miss:
//               mmu_walker_helper_store(...)  [extern; mmu.c]
//
// ============================================================================
// load / store 不对称的真机理 (跨文件协议见 dummy.txt §1 末段)
// ============================================================================
//
// 旧叙事 "load 性能敏感所以 inline / store 副作用所以 extern" 不准确。真机理:
//
//   1. TLB 缓存的是 hva (tlb_e_t.host_ptr, 见 tlb.h)
//   2. MMIO 不进 TLB (tlb §A2 决策, MMIO 不被 walker fill 进 TLB)
//   3. → TLB 命中路径 **结构上**不带 RAM/MMIO 分支 (entry 内根本不可能是 MMIO)
//   4. → load 命中可以**直接 `return *hva`**, 不需要任何子 helper, 不需 IS_GPA_RAM
//   5. store 命中仍然必须走 store_helper, 不是因为"store 慢就慢点", 而是 LR/SC
//      reservation 清除 / 未来 SMC 副作用强制经 helper, 跟 RAM/MMIO 分流无关
//
// → load / store 不对称 = "副作用强制经 helper vs 没副作用可走 TLB 直接 *hva",
//   不是 "性能 vs 副作用" 二分。
//
// tlb.h "MMIO 不进 TLB" 的真意 = "让 TLB 命中路径结构上不带分支", 不是单纯
// "省 TLB 空间 / MMIO 命中率低"。
//
// ============================================================================
// misalign check 隐式契约
// ============================================================================
//
// misalign check (gva & (size-1) != 0) 由 caller 在 case 入口完成
// (LOAD/STORE_MISALIGN_CHECK 宏, interpreter.c 顶段), helper 内不查。所有
// helper (lsu_*_helper / mmu_walker_helper_* / store_helper) 都信任 caller 已查。
//
// 未来 JIT translator emit 时也得遵守同契约 (先 emit misalign check 再 emit
// call walker_helper) — 是 translator 责任, 不是 helper 的。
//
// ============================================================================
// trap 协议 (dummy.txt §1 路径 2a, helper 长跳)
// ============================================================================
//
//   - access fault: bus 内 mmio_read/write_helper 失败 → trap_raise(5/7, gva)
//   - SV32 page fault: walker_helper_load/store 内 mmu_walk 失败 → trap_raise(13/15/5/7, gva)
//   - LR 落 MMIO / SC 落 MMIO (未来 amo_lr/sc_helper): 永远 access fault (RV spec
//     implementation-defined; 项目跟 Spike/QEMU 一致, MMIO 上不支持 atomic)
//
// helper 内 trap_raise_exception 是 _Noreturn longjmp, 不返回 caller; caller 不需要
// goto out (但保留无害, 跟 OP_UNSUPPORTED case 同形态)。
//
// ============================================================================
// misalign 处理 (Spike 风格)
// ============================================================================
//
//   RV spec 允许实现选择: (a) 硬件支持 misalign (慢但成功) 或 (b) trap (cause 4/6)。
//   项目选 (b) trap 路径, 实现简单不需要拆字节; LR/SC + AMO 强制对齐本来就是这条路径。
//   QEMU 默认 (a) split, Spike 默认 (b) trap, 我们跟 Spike。
//
// ============================================================================
// 跨页
// ============================================================================
//
//   BARE 路径下 RAM 是 mmap 连续区域, 跨 4K 边界本身 OK; 跨 RAM 边界 (RAM 末端 vs
//   后续地址不在 RAM 区) 走 access fault (cause 5/7), 不拆字节。
//   SV32 路径跨 PTE 边界化简为 trap (具体 cause 真细化时拍, 主流模拟器都允许这种
//   简化; RV spec 也不强制要求支持跨页)。
//

#ifndef ISA_LSU_H
#define ISA_LSU_H

#include <stdint.h>
#include <string.h>     // memcpy: 防 strict-aliasing / unaligned 风险

#include "config.h"          // GUEST_RAM_START / GUEST_RAM_SIZE / TLB_NUM_ENTRIES
#include "core/cpu.h"        // cpu_t
#include "core/mmu.h"        // mmu_walker_helper_load/store (SV32 miss fall back) + check_perm
#include "core/tlb.h"        // tlb_t / tlb_e_t (current_tlb / TLB hit fast path)
#include "core/trap.h"       // trap_raise_exception (_Noreturn longjmp)
#include "platform/bus.h"    // mmio_read/write_helper (BARE MMIO 派发, _Noreturn-on-failure)
#include "platform/ram.h"    // IS_GPA_RAM / gpa_to_hva_offset
#include "riscv.h"           // PTE_V / PTE_D


// ----------------------------------------------------------------------------
// store_helper —— forward decl (定义见 lsu.c 末; 完整 doc 在本文件末段)
//
// 提前声明因为下面 lsu_store_helper inline 内部要调 store_helper; C 顺序依赖
// (函数调用前必须先声明), 否则会被 clangd / gcc 误认为隐式 int 声明跟下面真 void
// 声明 conflict。
// ----------------------------------------------------------------------------
void store_helper(cpu_t *hart, uint8_t *hva, uint32_t gva_for_tval,
                  uint32_t value, uint32_t size);


// ----------------------------------------------------------------------------
// lsu_load_helper —— RV load 指令的顶层 inline helper (interpreter 直调)
//
// JIT 不调 lsu_load_helper (JIT 自己 inline 等价逻辑 + emit call mmu_walker_helper_load
// 作 slow path 块出口); 本 helper 是给 interpreter 用的"顶层分流 + TLB hit fast path"
// 的封装, inline 后等价于 interpreter case 内联展开了分流。
//
// 调用方: interpreter.c 5 load case (LB/LH/LW/LBU/LHU)。
//
// 参数:
//   hart        — 调用 hart
//   current_tlb — NULL = REGIME_BARE / 非 NULL = REGIME_SV32 (dispatcher 选定的叶 TLB)
//   gva         — guest 虚拟地址 = ea (READ_REG(rs1) + imm); caller 已查 misalign
//   size        — 1 / 2 / 4 (LB/LBU = 1, LH/LHU = 2, LW = 4)
//
// 返回: 32 位 host load 结果, 低 size 字节有效, 高位 0; sext/zext 由 caller 自做
//        (LB int8/LH int16 cast → int32 → uint32; LBU/LHU 直传)
//
// 错误 (longjmp 走 trap_raise_exception, 不返回 caller):
//   - misalign  → caller 已查; helper 内不会触发
//   - BARE PA 不在 RAM → mmio_read_helper 内未命中 / device 拒绝 → trap_raise(5/cause)
//   - SV32 walker fault → walker_helper_load 内 trap_raise(13/5, gva)
// ----------------------------------------------------------------------------
static inline uint32_t lsu_load_helper(cpu_t *hart, tlb_t *current_tlb,
                                       uint32_t gva, uint32_t size) {
    // misalign check: 由 caller (interpreter case 入口 LOAD_MISALIGN_CHECK 宏) 已查;
    // helper 内不重复 — 形成隐式契约 (见顶段 doc "misalign check 隐式契约")。

    if (current_tlb == NULL) {
        /* REGIME_BARE: pa = gva (identity); 内联 RAM/MMIO 分流 (BARE 路径必查,
         * 因为 BARE 不进 TLB) */
        if (IS_GPA_RAM(gva)) {
            uint8_t *host_ptr = gpa_to_hva_offset + gva;
            uint32_t value = 0;
            memcpy(&value, host_ptr, size);  // 低 size 字节有效, 高位 0
            return value;
        }
        // MMIO 派发: bus 内部 _Noreturn-on-failure (未命中 / device 拒绝都 longjmp,
        // 不返回 caller); 见 dummy.txt §8 / §9
        return mmio_read_helper(hart, gva, /*gva for tval*/gva, size);
    }

    /* REGIME_SV32: TLB hit fast path */
    //
    // 命中条件: V + tag + check_perm(R) — check_perm 是 mmu.h 的 static inline, 跟
    //   walker 同源, 完整查 priv/PTE_U/SUM/MXR + R-or-(MXR&&X)。
    //
    // 命中后**直接 return *hva, 不调子 helper** — 因为 TLB 缓存的是 hva 且 MMIO
    //   不进 TLB → 命中路径结构上不可能是 MMIO, 不需要 IS_GPA_RAM 分支 (见顶段
    //   见顶段 doc)。
    //
    // 不命中 (miss / V=0 / perm 不齐) → fall back mmu_walker_helper_load — walker 内
    //   重做 walk + check_perm + 必要时 set A 写回 PT + fill TLB (RAM) / 调
    //   mmio_read_helper (MMIO); 失败 trap_raise (cause 13 page / 5 access)。
    {
        const uint32_t vpn   = gva >> 12;
        const uint32_t index = vpn & (TLB_NUM_ENTRIES - 1);
        tlb_e_t *entry = &current_tlb->e[index];

        if ((entry->pte_flags & PTE_V)
            && entry->gva_tag == vpn
            && check_perm(hart, (uint32_t)entry->pte_flags, MMU_PERM_R)) {
            uint8_t *host_ptr = entry->host_ptr + (gva & 0xFFFu);
            uint32_t value = 0;
            memcpy(&value, host_ptr, size);
            return value;
        }
    }
    /* fall back to walker (含完整 perm 检查 + RAM/MMIO 分流 + fill TLB / 真访问;
     * 失败 trap_raise_exception 长跳) */
    return mmu_walker_helper_load(hart, current_tlb, gva, size);
}


// ----------------------------------------------------------------------------
// lsu_store_helper —— RV store 指令的顶层 inline helper (interpreter 直调)
//
// 跟 lsu_load_helper 同形态, 但 SV32 TLB 命中时**仍调 store_helper** (extern), 不
// 像 load 命中直接 *hva — 因为 store 有 LR/SC reservation 清除 + 未来 SMC 副作用,
// 强制经 helper (见顶段 doc 不对称真机理)。
//
// 调用方: interpreter.c 3 store case (SB/SH/SW)。
//
// 参数:
//   hart        — 调用 hart
//   current_tlb — NULL = REGIME_BARE / 非 NULL = REGIME_SV32
//   gva         — guest 虚拟地址 = ea (READ_REG(rs1) + imm); caller 已查 misalign
//   value       — 要 store 的值 (32 位); SB 写 value 低 8 位, SH 写低 16 位, SW
//                  写全 32 位, 由 size 决定写多少字节
//   size        — 1 / 2 / 4 (SB = 1, SH = 2, SW = 4)
//
// 错误 (longjmp 走 trap_raise_exception, 不返回 caller):
//   - misalign  → caller 已查
//   - BARE PA 不在 RAM → mmio_write_helper 内 trap_raise
//   - SV32 walker fault → walker_helper_store 内 trap_raise (cause 15/7, gva)
// ----------------------------------------------------------------------------
static inline void lsu_store_helper(cpu_t *hart, tlb_t *current_tlb,
                                    uint32_t gva, uint32_t value, uint32_t size) {
    // misalign 由 caller 已查 (interpreter case 入口 STORE_MISALIGN_CHECK 宏)

    if (current_tlb == NULL) {
        /* REGIME_BARE: pa = gva; 内联 RAM/MMIO 分流 */
        if (IS_GPA_RAM(gva)) {
            uint8_t *host_ptr = gpa_to_hva_offset + gva;
            store_helper(hart, host_ptr, /*gva for tval*/gva, value, size);
            return;
        }
        // MMIO 不参与 LR/SC reservation (LR 落 MMIO 已 access fault, 没机会建
        // reservation); MMIO 非可执行不参与 SMC; 所以 MMIO 路径不调 store_helper,
        // 直接 mmio_write_helper (跳过 reservation / SMC 副作用) — 跟 §8 三层模型
        // 一致 (mmio 是设备访问, 不是 RAM 写)
        mmio_write_helper(hart, gva, /*gva for tval*/gva, value, size);
        return;
    }

    /* REGIME_SV32: TLB hit fast path */
    //
    // 命中条件: V + tag + D + check_perm(W)
    //   D 位必查 — load 路径 walker 不 set D, store 时 D=0 必 fall back walker 重 set
    //   (hw-managed D 关键路径时序场景: walker 第一次 set D 写回 PT + 重 fill TLB;
    //   之后 fast path 直接调 store_helper)
    //
    // 命中后调 store_helper(hva, ...) — 不像 load 命中直接 *hva, 因 store 副作用
    // (LR/SC reservation 清除 + 未来 SMC) 强制经 helper。store_helper
    // HVA-based, caller 已确认 RAM (TLB 命中 → RAM), 不需重复 IS_GPA_RAM。
    //
    // 不命中 (miss / V=0 / D=0 / perm 不齐) → fall back mmu_walker_helper_store。
    {
        const uint32_t vpn   = gva >> 12;
        const uint32_t index = vpn & (TLB_NUM_ENTRIES - 1);
        tlb_e_t *entry = &current_tlb->e[index];

        if ((entry->pte_flags & PTE_V)
            && entry->gva_tag == vpn
            && (entry->pte_flags & PTE_D)
            && check_perm(hart, (uint32_t)entry->pte_flags, MMU_PERM_W)) {
            uint8_t *host_ptr = entry->host_ptr + (gva & 0xFFFu);
            store_helper(hart, host_ptr, /*gva for tval*/gva, value, size);
            return;
        }
    }
    /* fall back to walker (含完整 perm 检查 + set A+D 写回 PT + RAM 调 store_helper /
     * MMIO 调 mmio_write_helper / fill TLB; 失败 trap_raise 长跳) */
    mmu_walker_helper_store(hart, current_tlb, gva, value, size);
}


// ----------------------------------------------------------------------------
// store_helper —— RAM 写 + 副作用 (extern, HVA-based, lsu.c)
//
// 调用方 (3 处, 全部已确认 PA 落 RAM):
//   1. lsu_store_helper BARE + IS_GPA_RAM 命中 (hva = gpa_to_hva_offset + gva)
//   2. lsu_store_helper SV32 TLB 命中 (hva = entry->host_ptr + offset)
//   3. mmu_walker_helper_store SV32 miss + RAM 路径 (hva = gpa_to_hva_offset + pa)
//
// 接口 HVA-based: caller 已分流 RAM/MMIO, store_helper 不再判 IS_GPA_RAM (跟旧版
// GVA-based + 内部分流不同); MMIO 路径 caller 直接调 mmio_write_helper, 不经
// store_helper (跳过 LR/SC + SMC 副作用)。
//
// 参数:
//   hart           — 调用 hart
//   hva            — host 虚拟地址 (已确认 RAM)
//   gva_for_tval   — guest 虚拟地址, 仅供未来 LR/SC reservation 触发的 trap_raise
//                     当 tval 用 (当前 reservation 占位 noop, gva 实际未消费, 加
//                     (void)gva_for_tval 抑制 unused warning)
//   value          — 要写入的值 (低 size 字节有效)
//   size           — 1 / 2 / 4
//
// 内部:
//   (a) host store: memcpy(hva, &value, size)
//   (b) reservation 清除占位 (LR/SC, A 扩展真做时填)
//   (c) SMC page_dirty 占位 (jit/smc.c, JIT 真做时填)
//
// 错误路径: 当前无 (RAM 写 memcpy 不会失败; 未来 reservation/SMC 真做时可能 trap)
//
// 注: 实际 prototype 声明在本文件顶段 (forward decl, 解 lsu_store_helper inline
// 内调用的 C 顺序依赖); 本段只放完整 doc, 不重复声明。
// ----------------------------------------------------------------------------

#endif //ISA_LSU_H
