//
// Created by liujilan on 2026/6/4.
// isa/lrsc —— LR.W / SC.W / reservation 清除函数族 (T1 骨架 stub; T3 主方案 c 填实)。
//
// 顶部接口 doc + 七类清除时机 + T1/T3 划分 见 lrsc.h.
//

#include "lrsc.h"

#include <stdint.h>


/* T1 stub: 全部空体 / 返回中性值.
 *
 * 本文件 6 个函数在 T1 阶段不可达的有 5 个 (lr_w / sc_w / on_store / on_device_write
 * / init+destroy), 给 stub 是为 link 通过 — T1 还没正确性依赖;
 * lrsc_clear_self 是 fence.i 真调点, 但 reservation 字段 T3 才加到 cpu_t,
 * 现在空体是占位真调 — T3 一上字段就生效, fence.c 不动. */

void lrsc_init(void) {
    /* T3: pthread_mutex_init for lrsc_buckets[K] (K=64), Fibonacci hash. */
}

void lrsc_destroy(void) {
    /* T3: pthread_mutex_destroy 对偶. */
}

void lrsc_clear_self(cpu_t *hart) {
    /* T3: atomic_store_explicit(&hart->reservation_addr, INVALID, memory_order_release).
     * T1 字段还没加, 抑制 unused 即可. */
    (void)hart;
}

uxlen_t lrsc_lr_w(cpu_t *hart, uxlen_t pa) {
    /* T3: bucket lock + atomic_load *pa + atomic_store self.reservation = pa & ~3u.
     * T1 不可达 (interpreter 还没 case OP_LR_W); 返 0 是占位. */
    (void)hart;
    (void)pa;
    return 0u;
}

uxlen_t lrsc_sc_w(cpu_t *hart, uxlen_t pa, uxlen_t value) {
    /* T3: bucket lock + check self.reservation == pa & ~3u → atomic_store *pa = value;
     * 扫所有 hart 清匹配 reservation; 返 0=success / 1=fail.
     * T1 不可达; 返 1 (fail) 是更安全的占位 (避免万一被调到时 silently 成功). */
    (void)hart;
    (void)pa;
    (void)value;
    return 1u;
}

void lrsc_on_store(cpu_t *hart, uint32_t pa) {
    /* T3: bucket lock 内扫所有 hart, reservation_addr == (pa & ~3u) 则 atomic_store INVALID.
     * T1 lsu/amo 还没埋 hook (T2 埋), 现在彻底不可达. */
    (void)hart;
    (void)pa;
}

void lrsc_on_device_write(void) {
    /* T3: blanket — 遍历 harts[] 全清 reservation_addr → INVALID.
     * T1 virtio_blk io_worker 还没埋 hook, 不可达. */
}
