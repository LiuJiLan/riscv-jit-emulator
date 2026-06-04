//
// Created by liujilan on 2026/6/4.
// isa/lrsc —— LR.W / SC.W / reservation 清除函数族 (T3 主方案 c 实装)。
//
// 顶部接口 doc + 七类清除时机 + 类型规约 全部见 lrsc.h. 本文件只放实装, 不重复 doc.
//
// 内部数据 (file-static):
//   lrsc_reservations[MAX_HARTS]  per-hart _Atomic uxlen_t (字粒度 word-aligned PA;
//                                  哨兵 LRSC_INVALID_ADDR); cap 分配, 跨 hart 扫
//                                  按 n_harts 范围 (dummy.txt §15 cap-vs-n_harts).
//   lrsc_buckets[LRSC_BUCKET_NUM] 全局 bucket pthread_mutex_t 数组 (K = 64);
//                                  Fibonacci hash 散列 pa_word.
//
// memory_order: 全 seq_cst (Q11; aq/rl 精确化推迟 plan §2 #8). x86 host 上 atomic
// 退化为 LOCK 前缀 + 原子 RMW, perf 损失小.
//
// 跨 host (x86 / ARM64) 隐式假设: 普通 RAM access (lsu memcpy) 不走 atomic, 但 LR/SC
// 路径内部读 *pa / 写 *pa 都用 atomic_load_explicit / atomic_store_explicit 显式
// ordering — host x86 TSO 下退化为 mov, ARM64 下生成 ldar/stlr; 不依赖 host memory
// model (lrsc_amo_decision.md "主方案补充 — RAM access ordering" 段).
//

#include "lrsc.h"

#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>           // fprintf (init fail 路径)
#include <string.h>          // memcpy (*pa access; 跟 lsu memcpy 同体例) / strerror

#include "config.h"          // MAX_HARTS / LRSC_BUCKET_BITS / EOL
#include "core/cpu.h"        // n_harts (extern)
#include "platform/ram.h"    // gpa_to_hva_offset (GPA → HVA)
#include "riscv.h"           // uxlen_t


/* ============================================================================
 * 内部数据 (file-static)
 * ============================================================================ */

#define LRSC_BUCKET_NUM    (1U << LRSC_BUCKET_BITS)

static _Atomic uxlen_t lrsc_reservations[MAX_HARTS];
static pthread_mutex_t lrsc_buckets[LRSC_BUCKET_NUM];


/* ============================================================================
 * bucket_lock — Fibonacci hash 散列 pa_word → bucket idx (Q18)
 *
 * Fibonacci hash (乘 2^32 / φ = 2654435761u) 抗局部性聚集 — 相邻 PA 散到不同桶;
 * 一次 32-bit 乘法 (wrap-around) + shift, host x86_64 ~2-3 ns. 撞桶率从 ~50%
 * 降到 ~5%.
 *
 * 32-bit 乘法 wrap-around 关键: pa_word 跟 2654435761u 都 cast 到 uint32_t, 相
 * 乘结果在 uint32_t 内 mod 2^32 (C 标准 unsigned arithmetic 不 UB), 然后 >>
 * (32 - LRSC_BUCKET_BITS) 取乘积高 LRSC_BUCKET_BITS 位作 idx. 不能用
 * (uint64_t)pa_word cast 后乘 — 那会得 64-bit 乘积, shift 26 位后 idx 高达 38
 * bit OOB.
 *
 * pa_word = pa >> 2 (字粒度对齐, Q2); pa 高 30 bit 进 hash, 低 2 bit (sub-word
 * offset) 丢弃 — sub-word overlap 由 caller 用 pa_word_aligned 比对自然 cover.
 *
 * RV64 切换 (§13 review 项): pa_word 升 uint64_t 时 (uint32_t)cast 截断高 32 位,
 * cross-page-table 区域撞桶率上升但 correct (无 OOB); 真切 RV64 时 hash function
 * 同步升 64-bit Fibonacci 常数 (11400714819323198485ull = 2^64 / φ).
 * ============================================================================ */
static inline pthread_mutex_t *bucket_lock(uxlen_t pa) {
    uxlen_t  pa_word = pa >> 2;
    uint32_t hash    = (uint32_t)pa_word * 2654435761u;
    uint32_t idx     = hash >> (32u - LRSC_BUCKET_BITS);
    return &lrsc_buckets[idx];
}


/* ============================================================================
 * lifecycle
 *
 * lifecycle 配对按 cap (MAX_HARTS / LRSC_BUCKET_NUM); 跟 dummy.txt §15 第 1 类
 * (数组定义 / lifecycle 配对) 判据一致 — n_harts 改成 8 也不漏 init/destroy.
 * ============================================================================ */

void lrsc_init(void) {
    /* bucket 锁 init (cap 配对). 失败 fprintf + 继续 (lifecycle init 体例同 wfi_init /
     * clint_init — fail 不 propagate 上去, 后续路径自然走 abort). */
    for (uint32_t i = 0; i < LRSC_BUCKET_NUM; i++) {
        int rc = pthread_mutex_init(&lrsc_buckets[i], NULL);
        if (rc != 0) {
            fprintf(stderr, "[lrsc] pthread_mutex_init(bucket %u) failed: %s" EOL,
                    i, strerror(rc));
        }
    }

    /* 全 hart reservation 初始 INVALID (cap 全清). 此时 main 还没 spawn hart 线程,
     * 单线程写 atomic 字段 ordering 不重要, release 是 lifecycle 体例统一. */
    for (uint32_t i = 0; i < MAX_HARTS; i++) {
        atomic_store_explicit(&lrsc_reservations[i], LRSC_INVALID_ADDR,
                              memory_order_release);
    }
}

void lrsc_destroy(void) {
    /* cap 配对 destroy. pthread_mutex_destroy 对持有中的锁 UB, 但 main lifecycle 调
     * destroy 时所有 hart pthread 已 join (谁 spawn 谁 join), 没人持锁; 返值忽略. */
    for (uint32_t i = 0; i < LRSC_BUCKET_NUM; i++) {
        (void)pthread_mutex_destroy(&lrsc_buckets[i]);
    }
}


/* ============================================================================
 * lrsc_clear_self — lock-free 自清 (七类 #1-#4)
 * ============================================================================ */
void lrsc_clear_self(cpu_t *hart) {
    atomic_store_explicit(&lrsc_reservations[hart->hartid], LRSC_INVALID_ADDR,
                          memory_order_release);
}


/* ============================================================================
 * lrsc_lr_w — LR.W lock-free
 *
 * 流程 (按 lrsc_amo_decision.md 主方案):
 *   1. pa_word = pa & ~3u (字对齐; caller 已保证, 这里兜底)
 *   2. GPA → HVA: hva = gpa_to_hva_offset + pa_word (caller 已保证在 RAM 区)
 *   3. memcpy *hva → value (4-byte, plain RAM read 跟 lsu_load_helper 同体例)
 *   4. atomic_store self.reservation = pa_word (建 reservation)
 *   5. zero-ext 返 value
 *
 * 不拿 bucket lock 理由: LR vs cross-hart Store/SC 的 race spec line 142-152 允许
 * 两种 linearization (Store < LR < SC 成功 / LR < Store < SC 失败); LR + SC sequence
 * 的 atomicity 由 reservation 协议保证 (SC 拿 bucket lock + 查 self.reservation),
 * 不由 LR *pa load 本身的 host-atomic 保证.
 *
 * *pa 用 plain memcpy 跟 lsu_load_helper 同体例 (不用 atomic_load): amo apply 用
 * atomic_fetch_xxx 是因为 RMW 必须 host-atomic (拆 read+modify+write 会 lost-update);
 * LR 是 read-only, atomic 不是必须 — 跟 lsu plain RAM read 体例对齐更合适, ARM64
 * host 切换时跟 lsu 一并升级 (plan §2 + lrsc_amo_decision.md Q20).
 * ============================================================================ */
uxlen_t lrsc_lr_w(cpu_t *hart, uxlen_t pa) {
    uxlen_t pa_word = pa & ~(uxlen_t)3;

    /* *pa 32-bit 当前值 — plain memcpy 跟 lsu_load_helper SV32 hit / BARE RAM 路径
     * 同体例 (host x86 TSO + 4-byte aligned mov 单指令原子, 行为等价 atomic). */
    uint32_t value;
    memcpy(&value, gpa_to_hva_offset + pa_word, 4);

    /* 建 reservation (覆盖旧值即清旧, Q5 A2 spec line 134-136 强制).
     * reservation 字段必须 atomic — 真跨 hart 共享的"协议状态" (区别 RAM 字节). */
    atomic_store_explicit(&lrsc_reservations[hart->hartid], pa_word,
                          memory_order_seq_cst);

    return (uxlen_t)value;  /* zero-ext (32-bit value 进 uxlen_t) */
}


/* ============================================================================
 * lrsc_sc_w — SC.W 锁内
 *
 * 流程:
 *   1. pa_word = pa & ~3u
 *   2. 拿 bucket(pa) 锁
 *   3. atomic_load self.reservation; 跟 pa_word 比对
 *      - 不匹配: atomic_store self.res = INVALID + 释锁 + 返 1
 *      - 匹配:   memcpy *hva ← value (RAM 写, plain 跟 lsu store_helper 同体例)
 *                扫所有 hart 清匹配 reservation (含自己, 自然清自己)
 *                释锁 + 返 0
 *   4. 无论成败 self.reservation 最终都是 INVALID (spec line 59-60).
 *
 * *pa 用 plain memcpy 跟 lrsc_lr_w 顶段 doc 同源: SC.W atomicity 由 bucket lock +
 * reservation 协议保证 (lock 串行化"同 PA 上的 SC vs lrsc_on_store vs AMO apply",
 * reservation 状态保证 LR + SC 配对原子), 不由 *pa 那一刀 store 的 host-atomic 保证.
 * ============================================================================ */
uxlen_t lrsc_sc_w(cpu_t *hart, uxlen_t pa, uxlen_t value) {
    uxlen_t pa_word = pa & ~(uxlen_t)3;
    pthread_mutex_t *bucket = bucket_lock(pa_word);

    pthread_mutex_lock(bucket);

    uxlen_t self_res = atomic_load_explicit(&lrsc_reservations[hart->hartid],
                                            memory_order_acquire);
    if (self_res != pa_word) {
        /* SC 失败: reservation 不匹配 (被别人清, 或本来没 LR). 仍清自己 res. */
        atomic_store_explicit(&lrsc_reservations[hart->hartid], LRSC_INVALID_ADDR,
                              memory_order_release);
        pthread_mutex_unlock(bucket);
        return 1u;
    }

    /* SC 成功: 写 RAM + 扫所有 hart 清匹配 reservation (含自己).
     * RAM 写用 plain memcpy 跟 lsu store_helper 同体例 (SC.W atomicity 由 bucket
     * lock + reservation 协议保证, 不依赖 *pa store 的 host-atomic; 4-byte aligned
     * memcpy 在 host x86 TSO 编译成单 mov, 行为等价 atomic_store). */
    memcpy(gpa_to_hva_offset + pa_word, &value, 4);

    for (uint32_t h = 0; h < n_harts; h++) {
        uxlen_t res = atomic_load_explicit(&lrsc_reservations[h],
                                           memory_order_acquire);
        if (res == pa_word) {
            atomic_store_explicit(&lrsc_reservations[h], LRSC_INVALID_ADDR,
                                  memory_order_release);
        }
    }

    pthread_mutex_unlock(bucket);
    return 0u;
}


/* ============================================================================
 * lrsc_on_store — 普通 store / AMO 末调
 *
 * 流程:
 *   1. pa_word = pa & ~3u
 *   2. 拿 bucket(pa) 锁
 *   3. 扫所有 hart (n_harts 范围, dummy.txt §15 第 1 类越界): reservation == pa_word
 *      则 atomic_store INVALID (含自己, RV spec line 116-118 implementation
 *      discretion 选清, lrsc_amo_decision.md Q5 A3a)
 *   4. 释锁
 *
 * 注: 跟 SC 的 "扫所有 hart 清匹配" 同模式 — bucket 锁串行化"同 PA 上的 store/SC"
 * 跟"同 PA 上的另一 store/SC" 之间的扫操作, 防 race lost-update.
 *
 * hart 参数当前未消费 (含自己一并扫); 保留参数为未来 "self vs cross 分流统计" /
 * "lifecycle assertion" 等扩展铺路, 同 lsu store_helper 签名形态.
 * ============================================================================ */
void lrsc_on_store(cpu_t *hart, uxlen_t pa) {
    (void)hart;
    uxlen_t pa_word = pa & ~(uxlen_t)3;
    pthread_mutex_t *bucket = bucket_lock(pa_word);

    pthread_mutex_lock(bucket);
    for (uint32_t h = 0; h < n_harts; h++) {
        uxlen_t res = atomic_load_explicit(&lrsc_reservations[h],
                                           memory_order_acquire);
        if (res == pa_word) {
            atomic_store_explicit(&lrsc_reservations[h], LRSC_INVALID_ADDR,
                                  memory_order_release);
        }
    }
    pthread_mutex_unlock(bucket);
}


/* ============================================================================
 * lrsc_on_device_write — device DMA blanket clear (锁外)
 *
 * 全清所有 hart reservation (n_harts 范围, cap-vs-n_harts §15). 不拿 bucket 锁
 * 理由: 单向 atomic_store INVALID (不读 reservation 当前值), 无 race; 任何并发
 * SC 拿 bucket 锁后 atomic_load 看 INVALID 失败, 自然合规.
 *
 * 比 spec 最小要求严: spec line 108-109 仅强制 "device 写 LR-accessed bytes" 清,
 * line 116-118 reservation set 其它字节是 implementation discretion. blanket clear
 * 更严但合规 (落 spurious failure latitude, line 221-237).
 * ============================================================================ */
void lrsc_on_device_write(void) {
    for (uint32_t h = 0; h < n_harts; h++) {
        atomic_store_explicit(&lrsc_reservations[h], LRSC_INVALID_ADDR,
                              memory_order_release);
    }
}
