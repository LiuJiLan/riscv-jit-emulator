//
// Created by liujilan on 2026/5/28.
// WFI 唤醒框架实现 — file-static per-hart slot 数组 + 5 接口函数。
//
// 接口形态 + 设计意图见 wfi.h; per-hart slot 协议见 dummy.txt §14;
// 线程 lifecycle 见 dummy.txt §12 (init/destroy 配对, 不放线程 spawn/join)。
// 报错风格见 dummy.txt §5。
//

#define _POSIX_C_SOURCE 200809L   // clock_gettime / CLOCK_REALTIME / pthread_cond_timedwait

#include "wfi.h"

#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "config.h"               // MAX_HARTS / WFI_TIMEOUT_NS
#include "debug.h"                // DEBUG_WFI_SLEEP ('w' 进 cond_wait 前打)


// ----------------------------------------------------------------------------
// per-hart slot (dummy.txt §14 体例)
// ----------------------------------------------------------------------------
//
// 每 hart 一对 (mutex, cond) + 64B cache line padding (假共享防御; pthread_mutex_t
// + pthread_cond_t 在 glibc 上加起来已超 64B, padding 后整体 ~128B, _pad 字段
// 保留作"对齐边界"明示)。slot 数组 file-static, 不暴露指针, 接口按 hartid 索引。
typedef struct {
    _Alignas(64) pthread_mutex_t mutex;   /* _Alignas 在第一 member: alignof(slot) = 64,
                                             整个 struct 按 cache line 对齐 (跟 cpu_t.regs
                                             同体例; dummy.txt §14 体例 1) */
    pthread_cond_t  cond;
    char            _pad[64];
} wfi_slot_t;

static wfi_slot_t wfi_slots[MAX_HARTS];


// ----------------------------------------------------------------------------
// lifecycle: wfi_init / wfi_destroy
// ----------------------------------------------------------------------------

int wfi_init(void) {
    for (uint32_t i = 0; i < MAX_HARTS; i++) {
        int rc = pthread_mutex_init(&wfi_slots[i].mutex, NULL);
        if (rc != 0) {
            fprintf(stderr, "wfi_init: pthread_mutex_init(slot[%u].mutex) failed: %s" EOL,
                    i, strerror(rc));
            // 半 init 路径回滚: 前 i 个 mutex / cond 不主动 destroy (跟 clint_init
            // 失败回滚同体例 — main 不会跑 destroy chain, 进程退出 OS 回收资源)。
            return -1;
        }
        rc = pthread_cond_init(&wfi_slots[i].cond, NULL);
        if (rc != 0) {
            fprintf(stderr, "wfi_init: pthread_cond_init(slot[%u].cond) failed: %s" EOL,
                    i, strerror(rc));
            (void)pthread_mutex_destroy(&wfi_slots[i].mutex);  // 当前 i 的 mutex 已 init, 单独 cleanup
            return -1;
        }
    }
    return 0;
}

void wfi_destroy(void) {
    // 调用前置 (main 保证): timer thread / io worker 已 join, clint/plic destroy
    // 已不再调 wfi_kick → 此时无其他线程会 lock slot mutex, destroy 安全。
    for (uint32_t i = 0; i < MAX_HARTS; i++) {
        (void)pthread_cond_destroy(&wfi_slots[i].cond);
        (void)pthread_mutex_destroy(&wfi_slots[i].mutex);
    }
}


// ----------------------------------------------------------------------------
// wfi_wait — 阻塞 hart 线程, 直到 predicate 返 true
// ----------------------------------------------------------------------------
//
// 经典 cond_var pattern (跟 virtio_blk io_worker / uart tx drain 同体例):
//   lock(mutex)
//   while (!pred(closure))
//       cond_timedwait(cond, mutex, WFI_TIMEOUT_NS)
//   unlock(mutex)
//
// 注意 predicate 必须在 lock hold 下调 — 防止 wake 丢失 race:
//   假如 predicate 在 lock 外调: hart 检 false → 决定睡 → 进 cond_wait 之前
//   source 端 atomic_store + lock+signal+unlock → signal 落到无 waiter 上丢失
//   → hart 调 cond_wait 永睡。
//   lock 内 check predicate + cond_wait 原子释放 mutex + 挂起, kick 端 lock 才
//   能 signal, 形成 happens-before 屏障。
//
// timedwait timeout 500ms 是兜底; 正常路径 (kick 接到) 立即唤醒。
// timedwait 用 CLOCK_REALTIME 因为 pthread_cond_init 默认 attr 用 CLOCK_REALTIME
// (POSIX 默认); 不调 pthread_condattr_setclock 切 CLOCK_MONOTONIC, 跟 virtio_blk
// io_worker 同体例。
void wfi_wait(uint32_t hartid, wfi_predicate_fn pred, void *closure) {
    if (hartid >= MAX_HARTS) return;        // 防御; caller 应已 bound check

    wfi_slot_t *s = &wfi_slots[hartid];
    pthread_mutex_lock(&s->mutex);

    while (!pred(closure)) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        // 加 WFI_TIMEOUT_NS, normalize tv_sec / tv_nsec
        ts.tv_nsec += (long)WFI_TIMEOUT_NS;
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec  += ts.tv_nsec / 1000000000L;
            ts.tv_nsec %= 1000000000L;
        }
        DEBUG_WFI_SLEEP();   /* trace 'w' — 进 cond_timedwait 之前; spurious wake +
                                predicate 仍 false 时多次 cond_timedwait → 多次 'w' */
        int rc = pthread_cond_timedwait(&s->cond, &s->mutex, &ts);
        // rc: 0 (signaled) / ETIMEDOUT (兜底) / EINTR (信号; spurious 同处理) —
        // 任何返回都重 check predicate, while loop 已涵盖, 不分流。
        (void)rc;
    }

    pthread_mutex_unlock(&s->mutex);
}


// ----------------------------------------------------------------------------
// wfi_kick / wfi_kick_all — 唤醒
// ----------------------------------------------------------------------------
//
// lock(mutex) 是必须的 (不是为了保护数据, 是为了形成跟 hart 端 predicate check
// 之间的 happens-before; 详 wfi_wait 段注释)。lock 持有时间仅 cond_signal 一瞬,
// 几乎不阻塞。
//
// 空 waitqueue 时近零成本: glibc pthread_cond_signal 实现检 waiter count, 0 时
// 跳过 futex_wake syscall, 仅 1 atomic load + 1 lock/unlock 对 (~50ns)。
//
// 调用约束: 普通线程上下文 (pthread_mutex_* 不是 async-signal-safe; signal
// handler 不能调本函数)。signal handler 触发 SRS 走 timeout 兜底 (500ms) 自愈。
void wfi_kick(uint32_t hartid) {
    if (hartid >= MAX_HARTS) return;

    wfi_slot_t *s = &wfi_slots[hartid];
    pthread_mutex_lock(&s->mutex);
    pthread_cond_signal(&s->cond);
    pthread_mutex_unlock(&s->mutex);
}

// wfi_kick_all — 紧急停机专用: 让所有 hart 尽快看到 SRS (区别于 wfi_kick 的
// "唤醒某个有中断 pending 的 hart")。当前唯一调用方 = runtime_fatal (整机紧急
// 停机)。详 wfi.h 顶段声明注释 + runtime.c runtime_fatal。
//
// 循环用 cap (MAX_HARTS) 不是 n_harts: fatal 是"出大事了", 宁可多 kick phantom
// slot (空 waitqueue 近零成本, glibc 跳 futex syscall; kick 幂等) 也不漏 kick
// 真 hart。正因为用 cap, 本函数理论上不应被正常路径调用 (正常路径只 kick 真实
// hart); 未来若出现正常状态下按 n_harts 的 kick_all 需求, 另分一个函数, 不复用本函数。
void wfi_kick_all(void) {
    // 逐 slot signal (不用单全局 cond_broadcast — 用 per-hart 没意义, 但既然
    // 体例已经 per-hart, kick_all 就是 for 循环)。
    for (uint32_t i = 0; i < MAX_HARTS; i++) {
        wfi_kick(i);
    }
}
