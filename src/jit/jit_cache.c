//
// Created by liujilan on 2026/6/7.
// jit/jit_cache.c —— JIT 块缓存 + EBR RCU 实装.
//
// 顶部接口 doc + 设计依据 全部见 jit_cache.h. 本文件只放实装.
//
// 内部数据 (file-static):
//   hash_table[JIT_CACHE_SIZE]        ~2MB; 65536 slot 线性探测
//   page_block_head[GUEST_RAM_NPAGES] ~64KB; per-page block list 头 (acq_rel CAS)
//   jit_rcu_slots[MAX_HARTS]          per-hart epoch (_Alignas(64) 防假共享)
//   jit_rcu_global_epoch              global epoch counter (modify-side fetch_add)
//
// memory_order:
//   - status load acquire / store release: install/lookup 配对 happens-before
//     (lookup 看到 COMPILED 时 host_code_ptr 等字段已 visible)
//   - epoch load acquire / store release: EBR critical section 协议
//   - page_block_head CAS acq_rel: 链表头多 producer 间互序
//

#define _POSIX_C_SOURCE 200809L   // sched_yield (jit_rcu_synchronize busy-wait)

#include "jit_cache.h"

#include <sched.h>           // sched_yield
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>          // NULL (stddef.h 也可, 跟 jit_api.h 顶段体例一致)

#include "config.h"          // MAX_HARTS / GUEST_RAM_NPAGES / GUEST_RAM_START
#include "core/cpu.h"        // n_harts (extern; cap-vs-n_harts §15 存在性遍历)


/* ============================================================================
 * 内部常量 (file-static; 不入 config.h, 跟 PLIC stride 宏分布同体例)
 * ============================================================================ */

#define JIT_CACHE_BITS              16U
#define JIT_CACHE_SIZE              (1U << JIT_CACHE_BITS)   /* 65536; uint16_t 索引正好 */
#define JIT_CACHE_MASK              (JIT_CACHE_SIZE - 1U)

#define JIT_PAGE_HEAD_END           0xFFFFU      /* 链尾哨兵; uint16_t 索引最大值 */

#define JIT_LOOKUP_MAX_PROBES       16U          /* 线性探测上限 (装填率 < 50% 时短) */
#define JIT_INSTALL_MAX_PROBES      16U

#define JIT_RCU_INACTIVE            UINT32_MAX   /* local_epoch 编码"出 critical" */


/* ============================================================================
 * 内部数据 (file-static)
 * ============================================================================ */

static jit_cache_entry_t hash_table[JIT_CACHE_SIZE];

static _Atomic uint16_t page_block_head[GUEST_RAM_NPAGES];

/* EBR per-hart epoch slot (对偶 wfi_slots; _Alignas(64) 在第一 member, alignof
 * (slot) = 64, 整 struct 按 cache line 对齐 — 跟 wfi.c wfi_slot_t 同体例). */
typedef struct {
    _Alignas(64) _Atomic uint32_t epoch;
    char _pad[64 - sizeof(_Atomic uint32_t)];
} jit_rcu_slot_t;

static jit_rcu_slot_t  jit_rcu_slots[MAX_HARTS];
static _Atomic uint32_t jit_rcu_global_epoch;


/* ============================================================================
 * jit_cache_hash — Fibonacci hash 散列 (pa_word, regime) → slot idx
 *
 * Fibonacci hash 跟 lrsc.c bucket_lock 同体例; 常数 2654435761u = 2^32/φ:
 *   1. pa_word = (uint32_t)(pa >> 2) — 字粒度对齐, RV32 pa 原本就 32 位
 *   2. pa_word * 2654435761u — uint32_t wrap-around 主散列
 *   3. ^ (uint32_t)regime * 40503u — 另一 Fibonacci 常数 ≈ 2^16/φ; 同 PA 不同
 *      regime 散到不同 idx (regime 3 状态: BARE/SV32_S/SV32_U)
 *   4. >> (32 - JIT_CACHE_BITS) — 取乘积高 16 位作 idx
 *
 * 32-bit wrap-around 关键 (跟 lrsc bucket_lock 同陷阱): pa_word 必须 cast 到
 *   uint32_t 后乘, 不能 (uint64_t)pa_word 乘后 shift — 那会 idx OOB.
 *
 * RV64 切换 (§13 review): pa 升 uint64_t, (uint32_t)cast 截断高 32 位; 撞桶率
 *   上升但 correct (跟 lrsc 同处理).
 * ============================================================================ */
static inline size_t jit_cache_hash(uxlen_t pa, regime_t regime) {
    uint32_t pa_word = (uint32_t)(pa >> 2);
    uint32_t h = pa_word * 2654435761u ^ (uint32_t)regime * 40503u;
    return h >> (32u - JIT_CACHE_BITS);
}


/* ============================================================================
 * lifecycle (对偶 wfi_init/destroy + lrsc_init/destroy)
 *
 * 配对按 cap (dummy.txt §15 lifecycle 必用 cap): hash_table / page_block_head
 * 按 JIT_CACHE_SIZE / GUEST_RAM_NPAGES; jit_rcu_slots 按 MAX_HARTS.
 * ============================================================================ */

int jit_cache_init(void) {
    /* hash_table: 全 EMPTY + next_in_page 哨兵; key 字段清 0 (release store
     * status 给 future race 看到 EMPTY 时其他字段亦清). */
    for (uint32_t i = 0; i < JIT_CACHE_SIZE; i++) {
        hash_table[i].key_pa        = 0;
        hash_table[i].key_regime    = REGIME_BARE;
        hash_table[i].counter       = 0;
        hash_table[i].host_code_ptr = NULL;
        hash_table[i].next_in_page  = JIT_PAGE_HEAD_END;
        atomic_store_explicit(&hash_table[i].status,
                              (uint32_t)JIT_CACHE_EMPTY, memory_order_release);
    }

    /* page_block_head: 全链尾哨兵 (每 page 当前没块) */
    for (uint32_t i = 0; i < GUEST_RAM_NPAGES; i++) {
        atomic_store_explicit(&page_block_head[i],
                              JIT_PAGE_HEAD_END, memory_order_release);
    }

    /* EBR: 全 hart 标 INACTIVE (cap 配对; phantom slot 永远 INACTIVE) +
     * global_epoch = 0. main 还没 spawn hart 线程, 单线程写 atomic 字段 ordering
     * 不重要, release 是 lifecycle 体例统一 (lrsc_init 同形态). */
    for (uint32_t i = 0; i < MAX_HARTS; i++) {
        atomic_store_explicit(&jit_rcu_slots[i].epoch,
                              JIT_RCU_INACTIVE, memory_order_release);
    }
    atomic_store_explicit(&jit_rcu_global_epoch, 0u, memory_order_release);

    return 0;
}

void jit_cache_destroy(void) {
    /* 纯无锁数据结构 (无 mutex / cond / pthread); 进程退出 OS 回收.
     * 写 atomic clear 防 valgrind 噪声 (跟 lrsc_destroy 体例).
     * jit_rcu_slots epoch 清 INACTIVE 是 destroy 后若有意外二次 init 状态 known. */
    for (uint32_t i = 0; i < MAX_HARTS; i++) {
        atomic_store_explicit(&jit_rcu_slots[i].epoch,
                              JIT_RCU_INACTIVE, memory_order_release);
    }
}


/* ============================================================================
 * EBR (jit_rcu_read_lock / read_unlock / synchronize)
 *
 * read_lock / read_unlock 实装作 non-inline (jit_cache.h 不暴露 inline 定义) —
 *   真撞 fast path 瓶颈时考虑 inline 化 (改进项; dispatcher fork 点每块进出各一次).
 * ============================================================================ */

void jit_rcu_read_lock(uint32_t hart_id) {
    /* load global_epoch (acquire) → store local_epoch (release):
     * "我进 critical, 看的是全局 g 这个版本".
     * acquire 保证 modify-side 之前 store (清 status / 链表头清) 对本 reader 可见. */
    uint32_t g = atomic_load_explicit(&jit_rcu_global_epoch,
                                      memory_order_acquire);
    atomic_store_explicit(&jit_rcu_slots[hart_id].epoch, g,
                          memory_order_release);
}

void jit_rcu_read_unlock(uint32_t hart_id) {
    /* store INACTIVE (release): "我出 critical 了".
     * release 让 reader 持引用期间的所有 load (host_code 执行字节读) 跟此 store
     * 形成 happens-before, modify-side 看到 INACTIVE 后可安全 unmap. */
    atomic_store_explicit(&jit_rcu_slots[hart_id].epoch,
                          JIT_RCU_INACTIVE, memory_order_release);
}

void jit_rcu_synchronize(void) {
    /* fetch_add 拿旧值, target = 旧 + 1 (新 epoch); acq_rel 是 RMW 标配:
     *   release 让 modifier 之前的 store (清 status / 链表头清) 对其他 reader
     *     看到新 epoch 后可见;
     *   acquire 让本函数后续 load read-side epoch 时拿最新值. */
    uint32_t target =
        atomic_fetch_add_explicit(&jit_rcu_global_epoch, 1u,
                                  memory_order_acq_rel) + 1u;

    /* 遍历 n_harts (存在性遍历, §15; phantom slot 永远 INACTIVE 走 cap 也不卡,
     * 但按 §15 判据存在性遍历用 n_harts). */
    for (uint32_t i = 0; i < n_harts; i++) {
        for (;;) {
            uint32_t local = atomic_load_explicit(&jit_rcu_slots[i].epoch,
                                                  memory_order_acquire);
            if (local == JIT_RCU_INACTIVE) { break; }   /* hart 已出 critical */
            if (local >= target)           { break; }   /* hart 已重进, 用新 epoch */
            sched_yield();
        }
    }
}


/* ============================================================================
 * jit_cache_lookup — RCU read-side (caller 已调 jit_rcu_read_lock)
 *
 * 线性探测 ≤ JIT_LOOKUP_MAX_PROBES 步; status load acquire 配 install store
 * release 形成 happens-before.
 * ============================================================================ */
jit_cache_entry_t *jit_cache_lookup(uxlen_t pa, regime_t regime) {
    size_t idx = jit_cache_hash(pa, regime);

    for (uint32_t step = 0; step < JIT_LOOKUP_MAX_PROBES; step++) {
        jit_cache_entry_t *e = &hash_table[((uint32_t)idx + step) & JIT_CACHE_MASK];
        uint32_t s = atomic_load_explicit(&e->status, memory_order_acquire);

        if (s == (uint32_t)JIT_CACHE_COMPILED
            && e->key_pa == pa && e->key_regime == regime) {
            return e;
        }
        if (s == (uint32_t)JIT_CACHE_EMPTY) {
            return NULL;   /* 开放寻址 sentinel: 探测停 */
        }
        /* COUNTING / BLACK 继续探测 (同 key 在别槽不可能; COUNTING/BLACK 在别槽
         * 是 hash 冲突, 探测继续) */
    }
    return NULL;
}


/* ============================================================================
 * jit_cache_install — RCU modify-side, atomic CAS 路径无锁
 *
 * 找 EMPTY slot → CAS EMPTY→COUNTING → 填字段 → CAS 挂 page_block_head →
 * store COMPILED (release).
 *
 * page_idx = (pa - GUEST_RAM_START) >> 12; 不在 RAM 区 (caller bug) 兜底不挂链表.
 * ============================================================================ */
int jit_cache_install(uxlen_t pa, regime_t regime, void *host_code_ptr) {
    size_t idx = jit_cache_hash(pa, regime);
    uint32_t slot_idx_u32 = 0;
    jit_cache_entry_t *e = NULL;

    for (uint32_t step = 0; step < JIT_INSTALL_MAX_PROBES; step++) {
        uint32_t slot = ((uint32_t)idx + step) & JIT_CACHE_MASK;
        e = &hash_table[slot];

        uint32_t expected = (uint32_t)JIT_CACHE_EMPTY;
        if (atomic_compare_exchange_strong_explicit(
                &e->status, &expected,
                (uint32_t)JIT_CACHE_COUNTING,
                memory_order_acq_rel, memory_order_acquire)) {
            slot_idx_u32 = slot;
            goto got_slot;
        }
        /* CAS 失败: expected 已被 CAS 更新成当前 status. idempotent check 只能在
         *   COMPILED/BLACK 状态读 key 字段 — 那两状态的 release store 已发布
         *   (install/blacklist 末段 store COMPILED/BLACK release), key 字段 visible.
         * COUNTING 状态下 key 字段 in-flight (别 hart CAS 进 COUNTING 之后填字段
         *   中), 读 key 是 race (TSan 验证). 继续探测下一槽即可. */
        if ((expected == (uint32_t)JIT_CACHE_COMPILED
             || expected == (uint32_t)JIT_CACHE_BLACK)
            && e->key_pa == pa && e->key_regime == regime) {
            return 0;
        }
    }
    return -1;   /* FULL: 探测 N 步都没 EMPTY → caller Flush */

got_slot:
    /* CAS 成功, owner 独占该 slot. 填字段 (非 atomic; release 给 status 时 visible). */
    e->key_pa        = pa;
    e->key_regime    = regime;
    e->counter       = 0;
    e->host_code_ptr = host_code_ptr;

    /* 挂 page_block_head 链表头 (退栈式; page_idx 用 RAM 起点偏移).
     * caller 应已 IS_GPA_RAM 检查 pa 在 RAM 区; 兜底防越界用 page_idx <
     * GUEST_RAM_NPAGES. */
    uint32_t page_idx = (uint32_t)((pa - GUEST_RAM_START) >> 12);
    if (page_idx < GUEST_RAM_NPAGES) {
        uint16_t old_head = atomic_load_explicit(&page_block_head[page_idx],
                                                 memory_order_acquire);
        do {
            e->next_in_page = old_head;
        } while (!atomic_compare_exchange_weak_explicit(
                     &page_block_head[page_idx], &old_head,
                     (uint16_t)slot_idx_u32,
                     memory_order_acq_rel, memory_order_acquire));
    } else {
        /* pa 不在 RAM (兜底; caller 应保证). slot 仍进 COMPILED 状态, 但
         * invalidate_page 找不到此 slot (链表外); flush_all 仍能清. 当前实装
         * 当前阶段没真路径触发, 仅防御. */
        e->next_in_page = JIT_PAGE_HEAD_END;
    }

    /* status COMPILED (release; lookup acquire 后 host_code_ptr 等字段 visible) */
    atomic_store_explicit(&e->status, (uint32_t)JIT_CACHE_COMPILED,
                          memory_order_release);
    return 0;
}


/* ============================================================================
 * jit_cache_set_blacklist — Q11 a 二次 FULL 路径; (PA, regime) 进 BLACK
 *
 * 跟 install 同体例 (CAS EMPTY→COUNTING → 填字段 → 挂链表 → store release),
 * 差异: terminal status = BLACK, host_code_ptr = NULL.
 *
 * 撞已有 COMPILED 同 key 直接返 0 (别 hart 已成功 install, 保留 COMPILED 更合理;
 * BLACK 本意"永远编不出"但别 hart 能编说明这块能编, race 下保留好的). 撞 BLACK
 * 同 key 也返 0 idempotent. jit_entry.cc Q11 a 组合层调本接口.
 * ============================================================================ */
int jit_cache_set_blacklist(uxlen_t pa, regime_t regime) {
    size_t idx = jit_cache_hash(pa, regime);
    uint32_t slot_idx_u32 = 0;
    jit_cache_entry_t *e = NULL;

    for (uint32_t step = 0; step < JIT_INSTALL_MAX_PROBES; step++) {
        uint32_t slot = ((uint32_t)idx + step) & JIT_CACHE_MASK;
        e = &hash_table[slot];

        uint32_t expected = (uint32_t)JIT_CACHE_EMPTY;
        if (atomic_compare_exchange_strong_explicit(
                &e->status, &expected,
                (uint32_t)JIT_CACHE_COUNTING,
                memory_order_acq_rel, memory_order_acquire)) {
            slot_idx_u32 = slot;
            goto got_slot;
        }
        if ((expected == (uint32_t)JIT_CACHE_COMPILED
             || expected == (uint32_t)JIT_CACHE_BLACK)
            && e->key_pa == pa && e->key_regime == regime) {
            return 0;
        }
    }
    return -1;   /* 探测耗尽; caller 通常忽略 (退 interpreter). */

got_slot:
    e->key_pa        = pa;
    e->key_regime    = regime;
    e->counter       = 0;
    e->host_code_ptr = NULL;          /* BLACK 无编译产物 */

    uint32_t page_idx = (uint32_t)((pa - GUEST_RAM_START) >> 12);
    if (page_idx < GUEST_RAM_NPAGES) {
        uint16_t old_head = atomic_load_explicit(&page_block_head[page_idx],
                                                 memory_order_acquire);
        do {
            e->next_in_page = old_head;
        } while (!atomic_compare_exchange_weak_explicit(
                     &page_block_head[page_idx], &old_head,
                     (uint16_t)slot_idx_u32,
                     memory_order_acq_rel, memory_order_acquire));
    } else {
        e->next_in_page = JIT_PAGE_HEAD_END;
    }

    atomic_store_explicit(&e->status, (uint32_t)JIT_CACHE_BLACK,
                          memory_order_release);
    return 0;
}


/* ============================================================================
 * jit_cache_invalidate_page — RCU modify-side, sync grace period
 *
 * 顺 old_head 链表清 status → synchronize → backend.invalidate_block (T2 stub nop).
 * ============================================================================ */
void jit_cache_invalidate_page(uint32_t page_idx) {
    if (page_idx >= GUEST_RAM_NPAGES) { return; }   /* 防御 */

    /* atomic_exchange 拿 old_head + 清头 (acq_rel; 其他 hart install 看到新链头
     * = JIT_PAGE_HEAD_END, 同时本函数拿到旧链头作清空遍历入口). */
    uint16_t old_head = atomic_exchange_explicit(&page_block_head[page_idx],
                                                 JIT_PAGE_HEAD_END,
                                                 memory_order_acq_rel);

    /* 顺链表清 status (release; lookup acquire 之后看 EMPTY 即 miss).
     * 包括 BLACK → EMPTY (Q12 c; SMC / Flush 路径都清). */
    while (old_head != JIT_PAGE_HEAD_END) {
        jit_cache_entry_t *e = &hash_table[old_head];
        uint16_t next = e->next_in_page;
        atomic_store_explicit(&e->status, (uint32_t)JIT_CACHE_EMPTY,
                              memory_order_release);
        old_head = next;
    }

    /* 等所有 hart 出 read critical section, 旧引用 host_code_ptr 不再被任何
     * hart 持有, backend 可安全 unmap host_code mmap 区 (当前 backend stub nop). */
    jit_rcu_synchronize();

    /* 当前 backend stub nop; b_02 真做 emit 后调
     *   const backend_t *be = backend_get_default();
     *   for (清完的每个 slot.host_code_ptr) be->backend_invalidate_block(...);
     * 当前 host_code_ptr 永是 NULL (stub backend.compile_block 返
     * NOT_IMPLEMENTED 不真 install); b_02 真做 emit 时 backend 编 host code
     * 进 mmap RX 段, 这时 backend.invalidate_block 真 unmap. */
}


/* ============================================================================
 * jit_cache_flush_all — RCU modify-side, sync grace period (整表清)
 *
 * 扫整张 hash_table → 扫 page_block_head → synchronize → backend.flush_all
 * (当前 backend stub nop).
 * ============================================================================ */
void jit_cache_flush_all(void) {
    for (uint32_t i = 0; i < JIT_CACHE_SIZE; i++) {
        atomic_store_explicit(&hash_table[i].status,
                              (uint32_t)JIT_CACHE_EMPTY, memory_order_release);
    }
    for (uint32_t i = 0; i < GUEST_RAM_NPAGES; i++) {
        atomic_store_explicit(&page_block_head[i],
                              JIT_PAGE_HEAD_END, memory_order_release);
    }

    jit_rcu_synchronize();

    /* 当前 backend stub nop; b_02 真做后调
     *   const backend_t *be = backend_get_default();
     *   be->backend_flush_all(); */
}
