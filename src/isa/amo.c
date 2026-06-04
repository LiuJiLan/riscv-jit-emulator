//
// Created by liujilan on 2026/6/4.
// isa/amo —— 9 个 amo_xxx_apply 实现 (extern, HVA-based; 副作用 = host atomic RMW + lrsc_on_store).
//
// 顶部接口 doc + 3-layer 模型 + funct5 mapping + 编码字段 + memory_order/cause 选择 + fast/slow
// framing T3 重评估 trail 全部见 amo.h. 本文件只放 9 个 apply 实现, 不重复 doc.
//
// 实装策略 (Q16 a 路径, T2 user 拍):
//   5 ops (ADD/SWAP/XOR/OR/AND) - macro AMO_DEFINE_APPLY_DIRECT 注入, 一行映射到对应
//                                  C11 atomic_fetch_*/atomic_exchange 函数
//   4 ops (MIN/MAX/MINU/MAXU)   - macro AMO_DEFINE_APPLY_CAS 注入手写 CAS loop (C11 无
//                                  atomic_fetch_min/max). cmp_expr 参数控制 signed/unsigned
//

#include "amo.h"

#include <stdatomic.h>
#include <stdint.h>

#include "lrsc.h"            // lrsc_on_store (T1 占位真调; T3 上 reservation 字段后生效)
#include "platform/ram.h"    // gpa_to_hva_offset (pa 反推; Q10 实装备注同模式)


// ----------------------------------------------------------------------------
// 5 ops 用 atomic_fetch_* / atomic_exchange 直接映射
//
// macro 体一致, 只换 C11 函数名. 全部 memory_order_seq_cst (Q11; aq/rl 精确化推迟 plan
// §2 #8). x86 host 上 atomic_fetch_* seq_cst 退化为 LOCK 前缀 + 原子 RMW, perf 损失小.
//
// pa 反推: pa = (uint32_t)((uintptr_t)hva - (uintptr_t)gpa_to_hva_offset)
//   gpa_to_hva_offset 是 ram_init 时算出的常量指针 (host RAM 基址), 减法得 guest PA.
//   uintptr_t cast 是为消除 pointer arithmetic 的 sign-extension 边界 (gpa_to_hva_offset
//   有时是大正数, 直接 ptr 减可能溢出 ptrdiff_t).
// ----------------------------------------------------------------------------
#define AMO_DEFINE_APPLY_DIRECT(name, c11_fn)                                           \
uxlen_t amo_##name##_apply(cpu_t *hart, uint8_t *hva, uxlen_t gva_for_tval,             \
                           uxlen_t value) {                                             \
    (void)gva_for_tval;  /* T1/T2 不消费; T3 reservation 真做后接通当 trap tval */      \
    _Atomic uint32_t *target = (_Atomic uint32_t *)hva;                                 \
    uint32_t old = c11_fn(target, (uint32_t)value, memory_order_seq_cst);               \
    uint32_t pa  = (uint32_t)((uintptr_t)hva - (uintptr_t)gpa_to_hva_offset);           \
    lrsc_on_store(hart, pa);  /* T1 占位真调; T3 真扫所有 hart reservation */           \
    return old;                                                                         \
}

AMO_DEFINE_APPLY_DIRECT(add_w,  atomic_fetch_add_explicit)
AMO_DEFINE_APPLY_DIRECT(swap_w, atomic_exchange_explicit)
AMO_DEFINE_APPLY_DIRECT(xor_w,  atomic_fetch_xor_explicit)
AMO_DEFINE_APPLY_DIRECT(or_w,   atomic_fetch_or_explicit)
AMO_DEFINE_APPLY_DIRECT(and_w,  atomic_fetch_and_explicit)

#undef AMO_DEFINE_APPLY_DIRECT


// ----------------------------------------------------------------------------
// 4 ops (MIN/MAX/MINU/MAXU) 用 CAS loop 手写
//
// C11 无 atomic_fetch_min / atomic_fetch_max — 这是 spec 真缺口 (跟 add/xor/and/or 不
// 同, MIN/MAX 不是位级 reduction, 无法用 LOCK CMPXCHG 之外的单指令实现).
//
// CAS loop 模式 (跟 dummy.txt §10 同源):
//   1. load 当前 *target 到 old (seq_cst)
//   2. 算 new_val = cmp_expr ? old : v32 (cmp_expr=true 时保留 old, 即"无需写")
//   3. CAS (*target, expected=old, desired=new_val); 成功就 break
//   4. 失败时 atomic_compare_exchange_weak 自动把当前 *target 真值写回 expected (即 old),
//      retry 时不需要重新 load
//
// memory_order: 成功 seq_cst, 失败也 seq_cst (Q11; 推迟 acquire/release 精确化).
//
// weak 版本: 可 spuriously fail 但比 strong 快; loop 兜底处理 (跟普通 CAS spinloop 同).
//
// cmp_expr 选择 "保留 old (不写)" 的条件:
//   MIN signed:    old <= value → 保留 old  (signed cast 比较)
//   MAX signed:    old >= value → 保留 old
//   MINU unsigned: old <= value → 保留 old
//   MAXU unsigned: old >= value → 保留 old
//
// 注意: MIN/MAX 是 "**写**入 min/max", 不是 "**返回** min/max"; rd 还是 old 原值 (跟其它
// AMO 一致). cmp_expr "保留 old" = "不更新 mem (mem 已经是 min/max)".
// ----------------------------------------------------------------------------
#define AMO_DEFINE_APPLY_CAS(name, cmp_expr)                                            \
uxlen_t amo_##name##_apply(cpu_t *hart, uint8_t *hva, uxlen_t gva_for_tval,             \
                           uxlen_t value) {                                             \
    (void)gva_for_tval;                                                                 \
    _Atomic uint32_t *target = (_Atomic uint32_t *)hva;                                 \
    uint32_t v32 = (uint32_t)value;                                                     \
    uint32_t old = atomic_load_explicit(target, memory_order_seq_cst);                  \
    for (;;) {                                                                          \
        uint32_t new_val = (cmp_expr) ? old : v32;                                      \
        if (atomic_compare_exchange_weak_explicit(target, &old, new_val,                \
                memory_order_seq_cst, memory_order_seq_cst)) {                          \
            break;  /* old 是 CAS 之前真值 (即 spec "load 旧值"), 退 loop 返 rd */      \
        }                                                                               \
        /* CAS 失败: atomic_compare_exchange_weak 已把当前 *target 真值写回 &old,      \
         * 直接 retry (不需 re-load) */                                                  \
    }                                                                                   \
    uint32_t pa = (uint32_t)((uintptr_t)hva - (uintptr_t)gpa_to_hva_offset);            \
    lrsc_on_store(hart, pa);                                                            \
    return old;                                                                         \
}

AMO_DEFINE_APPLY_CAS(min_w,  (int32_t)old <= (int32_t)v32)
AMO_DEFINE_APPLY_CAS(max_w,  (int32_t)old >= (int32_t)v32)
AMO_DEFINE_APPLY_CAS(minu_w, old <= v32)
AMO_DEFINE_APPLY_CAS(maxu_w, old >= v32)

#undef AMO_DEFINE_APPLY_CAS
