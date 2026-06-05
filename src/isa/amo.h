//
// Created by liujilan on 2026/6/4.
// isa/amo —— RV32 A 扩展 Zaamo 9 个 AMO ops (AMOADD/SWAP/XOR/OR/AND/MIN/MAX/MINU/MAXU.W)
//
// 跟 LR/SC (Zalrsc) 拆开 (lrsc_amo_decision.md Q12): A 扩展 RV spec 已拆 Zaamo/Zalrsc 子
// 扩展, 实装跟 spec 子扩展边界对齐。LR/SC 见 isa/lrsc.{c,h}。
//
// 3-layer 模型 (跟 isa/lsu 严格对偶):
//   顶层 inline (本文件 amo.h) — 9 个 amo_xxx_helper, 跟 lsu_store_helper 体例; BARE/SV32
//     分流, SV32 TLB hit fast path inline (跟 store 同形态; sacred fast/slow path 原则).
//   中间 RAM 端副作用入口 (amo.c) — 9 个 amo_xxx_apply (extern, HVA-based), 内做 host C11
//     atomic_fetch_xxx + lrsc_on_store(hart, pa) (bucket lock + 扫所有 hart 清匹配
//     reservation) + 返 sext32(old). 跟 store_helper 一对一对偶, 但**不复用
//     store_helper** — store_helper 内 memcpy 跟 AMO RMW 不兼容
//     ("amo 不直接调用 store").
//   慢路径 walker (mmu.c) — 9 个 mmu_walker_helper_amo_xxx, 跟 mmu_walker_helper_store
//     体例; walk + PTE A+D OR + TLB fill + 末调 amo_xxx_apply.
//
// 9 op funct5 mapping (RV Unprivileged Spec Vol I "A" extension):
//   funct5=0x00 AMOADD.W   - atomic_fetch_add_explicit
//   funct5=0x01 AMOSWAP.W  - atomic_exchange_explicit
//   funct5=0x04 AMOXOR.W   - atomic_fetch_xor_explicit
//   funct5=0x08 AMOOR.W    - atomic_fetch_or_explicit
//   funct5=0x0C AMOAND.W   - atomic_fetch_and_explicit
//   funct5=0x10 AMOMIN.W   - signed min;  C11 无 atomic_fetch_min, 手写 CAS loop
//   funct5=0x14 AMOMAX.W   - signed max;  同 CAS loop
//   funct5=0x18 AMOMINU.W  - unsigned min; 同 CAS loop
//   funct5=0x1C AMOMAXU.W  - unsigned max; 同 CAS loop
// funct5=0x02 LR.W / 0x03 SC.W 是 Zalrsc, 不在本文件 (见 isa/lrsc.{c,h}).
//
// 编码字段:
//   bits[6:0]=opcode 0x2F   bits[14:12]=funct3 010 (.W)
//   bits[31:27]=funct5      decode 区分 9 op (decode.c 0x2F case)
//   bits[26:25]=aq/rl       Q11 拍全 seq_cst, 字段不解 (decode 不提取, op_kind 选择不依赖)
//   bits[24:20]=rs2 (value)  bits[19:15]=rs1 (gva)  bits[11:7]=rd (写回 sext32(old))
//
// memory_order: 全 seq_cst (Q11; aq/rl 精确化推迟 plan §2 #8). x86 host 上 atomic_fetch_*
//   seq_cst 退化为 LOCK 前缀 + 原子 RMW, perf 损失小.
//
// misalign cause = 6 (CAUSE_STORE_ADDR_MISALIGNED; AMO 跟 store 同 cause; RV spec 4-byte
//   对齐要求, Q8). caller (interpreter case 入口 AMO_MISALIGN_CHECK 宏) 已查; 本 helper
//   内不重复 (跟 lsu STORE_MISALIGN_CHECK 同源 - 隐式契约).
//
// MMIO AMO 行为: spec implementation-defined, 我们拒 → trap cause 7 (Spike/QEMU 同; 不开
//   mmio_amo_helper). BARE !IS_GPA_RAM 在 amo_xxx_helper inline 顶段拒; SV32 walker MMIO
//   在 mmu_walker_helper_amo_xxx 内拒.
//
// walker fault cause: mmu_walk(PERM_W) 失败自然返 cause 15 (CAUSE_STORE_PAGE_FAULT;
//   RV spec store/AMO 一起算 page fault), 跟 store walker 体例同, 不开 cause 区分.
//
// 不对偶点 (跟 store 唯一差异): 9 op 各自 1 套 (顶层 + apply + walker), store 只 1 套但
//   有 size 参数 (SB/SH/SW). 原因: AMO 各 op 调不同 C11 atomic_fetch_xxx, 不能用 size
//   分流; 拆 9 op helper 跟 dummy.txt §10 "helper 颗粒度 per RV 指令" 对齐, JIT 翻译时
//   helper 地址静态绑定 emit 出的 call 指令.
//
// fast/slow path framing 已评估维持 sacred 原则 (trade_off_log §T fast-slow-framing +
//   access_helper_call_graph.md §8). 本模块按 helper 颗粒度走 (AMO 在 atomics list,
//   "Never inline these into JIT-emitted code"). helper call 开销 ~5-10 cycle vs
//   apply 内 lrsc_on_store 100-300 cycle, call 占 ~3-5%; inline 进 JIT 会失去 host -O3
//   优化机会 (AsmJit 低级 builder, gcc 看不到 runtime emit). apply 层不消除.
//

#ifndef ISA_AMO_H
#define ISA_AMO_H

#include <stddef.h>          // NULL (current_tlb == NULL 编码 REGIME_BARE)
#include <stdint.h>

#include "config.h"          // TLB_NUM_ENTRIES
#include "core/cpu.h"        // cpu_t
#include "core/mmu.h"        // mmu_walker_helper_amo_xxx (SV32 miss fall back) + check_perm
#include "core/tlb.h"        // tlb_t / tlb_e_t (current_tlb / TLB hit fast path)
#include "core/trap.h"       // trap_raise_exception (_Noreturn longjmp)
#include "platform/ram.h"    // IS_GPA_RAM / gpa_to_hva_offset
#include "riscv.h"           // PTE_V / PTE_D / CAUSE_STORE_ACCESS_FAULT / uxlen_t


// ----------------------------------------------------------------------------
// amo_xxx_apply —— forward decl (定义见 amo.c; 完整 doc 在本文件末段)
//
// 提前声明因为下面 amo_xxx_helper inline 顶层要调 apply; C 顺序依赖 (函数调用前必须先
// 声明; 跟 lsu.h store_helper forward decl 同模式).
// ----------------------------------------------------------------------------
uxlen_t amo_add_w_apply (cpu_t *hart, uint8_t *hva, uxlen_t gva_for_tval, uxlen_t value);
uxlen_t amo_swap_w_apply(cpu_t *hart, uint8_t *hva, uxlen_t gva_for_tval, uxlen_t value);
uxlen_t amo_xor_w_apply (cpu_t *hart, uint8_t *hva, uxlen_t gva_for_tval, uxlen_t value);
uxlen_t amo_or_w_apply  (cpu_t *hart, uint8_t *hva, uxlen_t gva_for_tval, uxlen_t value);
uxlen_t amo_and_w_apply (cpu_t *hart, uint8_t *hva, uxlen_t gva_for_tval, uxlen_t value);
uxlen_t amo_min_w_apply (cpu_t *hart, uint8_t *hva, uxlen_t gva_for_tval, uxlen_t value);
uxlen_t amo_max_w_apply (cpu_t *hart, uint8_t *hva, uxlen_t gva_for_tval, uxlen_t value);
uxlen_t amo_minu_w_apply(cpu_t *hart, uint8_t *hva, uxlen_t gva_for_tval, uxlen_t value);
uxlen_t amo_maxu_w_apply(cpu_t *hart, uint8_t *hva, uxlen_t gva_for_tval, uxlen_t value);


// ----------------------------------------------------------------------------
// amo_xxx_helper —— RV AMO 指令的顶层 inline helper (interpreter 直调)
//
// 跟 lsu_store_helper 严格对偶 (BARE/SV32 分流 + TLB hit fast path); 唯一差异 = MMIO
// 路径 trap cause 7 (而非 mmio_write_helper 派发) — AMO on MMIO spec implementation-
// defined, 我们拒 (跟 Spike/QEMU 同).
//
// 调用方: interpreter.c 9 AMO case (OP_AMO_ADD_W / SWAP_W / XOR_W / OR_W / AND_W /
//          MIN_W / MAX_W / MINU_W / MAXU_W).
//
// 参数:
//   hart        - 调用 hart
//   current_tlb - NULL = REGIME_BARE / 非 NULL = REGIME_SV32_S 或 _U (dispatcher 选定的叶 TLB)
//   gva         - guest 虚拟地址 = ea = READ_REG(rs1); AMO 无 imm offset; caller 已查
//                  misalign (AMO_MISALIGN_CHECK 宏, cause 6)
//   value       - rs2 寄存器值 (AMO 操作的 "源" 值; 不同 op 含义不同 — ADD/AND/OR/XOR
//                  是位级源; SWAP 是覆盖值; MIN/MAX 是比较值)
//
// 返回: sext32(old) — RV spec "rd = sign-extended old value at addr". RV32 下 sext32 跟
//        原 32-bit 值等价 (uxlen_t 已经是 uint32_t).
//
// 错误 (longjmp 走 trap_raise_exception, 不返回 caller):
//   - misalign  → caller 已查; helper 内不会触发
//   - BARE PA 不在 RAM → trap_raise(7, gva) [cause 7 store access fault; AMO 落 MMIO 拒]
//   - SV32 walker fault → mmu_walker_helper_amo_xxx 内 trap_raise(15/7, gva)
//     [15 page fault / 7 access fault]
// ----------------------------------------------------------------------------

// AMO_DEFINE_INLINE_HELPER —— 9 个 helper 体一致, 只换 apply / walker 函数名, 用 macro
// 注入避免 9 份重复代码. macro 展开后是 9 份独立 static inline 函数 (gcc/clang 内联展开
// 仍单独翻译给 interpreter 各 case, JIT 未来翻译时按 op_kind 各调对应 apply / walker).
#define AMO_DEFINE_INLINE_HELPER(name)                                                  \
static inline uxlen_t amo_##name##_helper(cpu_t *hart, tlb_t *current_tlb,              \
                                          uxlen_t gva, uxlen_t value) {                 \
    /* misalign 由 caller (interpreter case AMO_MISALIGN_CHECK) 已查; 隐式契约 */       \
                                                                                        \
    if (current_tlb == NULL) {                                                          \
        /* REGIME_BARE: pa = gva (identity); 内联 RAM/MMIO 分流 */                      \
        if (IS_GPA_RAM(gva)) {                                                          \
            uint8_t *host_ptr = gpa_to_hva_offset + gva;                                \
            return amo_##name##_apply(hart, host_ptr, /*gva for tval*/gva, value);      \
        }                                                                               \
        /* AMO 落 MMIO → cause 7 store access fault (spec implementation-defined, 我们 \
         * 拒; 不开 mmio_amo_helper; 跟 Spike/QEMU 一致) */                              \
        trap_raise_exception(hart, CAUSE_STORE_ACCESS_FAULT, /*tval*/gva);              \
        /* _Noreturn longjmp; 下面 return 0 不可达, 仅消除编译器 "control reaches end" \
         * 警告 (跟 lsu.h 同模式; trap_raise_exception 已标 _Noreturn) */                \
        return 0;                                                                       \
    }                                                                                   \
                                                                                        \
    /* REGIME_SV32_S/_U: TLB hit fast path (跟 lsu_store_helper SV32 段同形 — V + tag + D + \
     * check_perm(W); AMO 需要 W 权限因为是 RMW). 命中后调 apply (跟 store 命中调       \
     * store_helper 同体例; 副作用入口 helper, sacred fast/slow path 原则). */          \
    {                                                                                   \
        const uint32_t vpn   = gva >> 12;                                               \
        const uint32_t index = vpn & (TLB_NUM_ENTRIES - 1);                             \
        tlb_e_t *entry = &current_tlb->e[index];                                        \
                                                                                        \
        if ((entry->pte_flags & PTE_V)                                                  \
            && entry->gva_tag == vpn                                                    \
            && (entry->pte_flags & PTE_D)                                               \
            && check_perm(hart, (uint32_t)entry->pte_flags, MMU_PERM_W)) {              \
            uint8_t *host_ptr = entry->host_ptr + (gva & 0xFFFu);                       \
            return amo_##name##_apply(hart, host_ptr, /*gva for tval*/gva, value);      \
        }                                                                               \
    }                                                                                   \
    /* fall back to walker (含完整 perm 检查 + set A+D 写回 PT + RAM 调 apply / MMIO 拒 \
     * cause 7 / fill TLB; 失败 trap_raise 长跳) */                                      \
    return mmu_walker_helper_amo_##name(hart, current_tlb, gva, value);                 \
}

AMO_DEFINE_INLINE_HELPER(add_w)
AMO_DEFINE_INLINE_HELPER(swap_w)
AMO_DEFINE_INLINE_HELPER(xor_w)
AMO_DEFINE_INLINE_HELPER(or_w)
AMO_DEFINE_INLINE_HELPER(and_w)
AMO_DEFINE_INLINE_HELPER(min_w)
AMO_DEFINE_INLINE_HELPER(max_w)
AMO_DEFINE_INLINE_HELPER(minu_w)
AMO_DEFINE_INLINE_HELPER(maxu_w)

#undef AMO_DEFINE_INLINE_HELPER


// ----------------------------------------------------------------------------
// amo_xxx_apply —— host C11 atomic RMW + 副作用 (extern, HVA-based, amo.c)
//
// 调用方 (3 处, 全部已确认 PA 落 RAM, 4-byte aligned, 有 W 权限):
//   1. amo_xxx_helper BARE + IS_GPA_RAM 命中 (hva = gpa_to_hva_offset + gva)
//   2. amo_xxx_helper SV32 TLB 命中 (hva = entry->host_ptr + offset)
//   3. mmu_walker_helper_amo_xxx SV32 miss + RAM 路径 (hva = gpa_to_hva_offset + pa)
//
// 接口 HVA-based: caller 已分流 RAM/MMIO, apply 不再判 IS_GPA_RAM (跟 store_helper 同形态).
// MMIO 路径在 caller 各自拒 (顶层 inline BARE / walker SV32 各自 trap_raise cause 7),
// 不经 apply.
//
// 签名带 gva_for_tval 是跟 store_helper 对齐 ("通用性 + 方便未来考虑 TLB
// 命中的修改问题"; 当前 reservation 协议路径无 trap, gva 实际未消费 — 加
// (void)gva_for_tval 抑制 unused).
//
// 参数:
//   hart           — 调用 hart
//   hva            — host 虚拟地址 (已确认 RAM, 4-byte aligned)
//   gva_for_tval   — guest 虚拟地址, 仅供未来 reservation 触发的 trap_raise 当 tval 用
//                     (当前 reservation 协议路径无 trap, (void)gva 抑制 unused)
//   value          — rs2 寄存器值, 32-bit (AMO 各 op 含义不同)
//
// 返回: 32-bit old value at hva (sext32 在 RV32 下跟原值等价; uxlen_t = uint32_t).
//        调用方写 rd = 本返回值.
//
// 内部 (5 macro + 4 手写):
//   (a) ADD/SWAP/XOR/OR/AND: 直调 host C11 atomic_fetch_xxx / atomic_exchange (seq_cst),
//        macro 注入避免重复
//   (b) MIN/MAX/MINU/MAXU: 手写 CAS loop (C11 无 atomic_fetch_min/max), seq_cst
//   (c) 末段 lrsc_on_store(hart, pa) — bucket lock 内扫所有 hart 清匹配 reservation
//        pa 反推: pa = (uxlen_t)((uintptr_t)hva - (uintptr_t)gpa_to_hva_offset)
//
// 错误路径: 当前无 (host atomic RMW 不会失败; 未来 reservation 真做时可能 trap, 用
//   gva_for_tval 当 tval)
//
// 注: 实际 prototype 声明在本文件顶段 (forward decl, 解 amo_xxx_helper inline 内调用
//   的 C 顺序依赖); 本段只放完整 doc, 不重复声明.
// ----------------------------------------------------------------------------

#endif //ISA_AMO_H
