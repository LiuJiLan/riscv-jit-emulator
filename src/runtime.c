//
// Created by liujilan on 2026/5/19.
// runtime 模块实现 — bitmap 化 + 接口函数包协议 (set 用 atomic_fetch_or,
// clear-with-condition 用 CAS-loop, shutdown 蕴含 system_reset 顺序 B)。
// 详 runtime.h 顶段 doc。
//

#include "runtime.h"

_Atomic uint32_t system_reset_signal = 0u;
_Atomic uint32_t shutdown_signal     = 0u;

void system_reset_signal_set_bit(uint32_t mask) {
    atomic_fetch_or_explicit(&system_reset_signal, mask, memory_order_release);
}

void shutdown_signal_set_bit(uint32_t mask) {
    // 顺序 B (SDS 先 SRS 后); 防 race 详 runtime.h doc。
    atomic_fetch_or_explicit(&shutdown_signal,
                             mask,
                             memory_order_release);
    atomic_fetch_or_explicit(&system_reset_signal,
                             SYSRESET_BIT_SHUTDOWN_TRIGGER,
                             memory_order_release);
}

bool system_reset_signal_try_clear_if_shutdown_zero(void) {
    // CAS-loop: atomic { if shutdown==0 then system_reset=0 } 近似。
    //   load shutdown check 0 → CAS system_reset expected → 0;
    //   CAS conflict (别人在写 SRS bit) → expected 自动更新为最新值 retry。
    // race window 下 shutdown 在两次 load 间从 0 变非 0 时 CAS 仍可能成功; 行为
    // 无害 — main 下一 iter 看到 shutdown != 0 自然退出, 走 cleanup 路径。
    uint32_t expected = atomic_load_explicit(&system_reset_signal,
                                             memory_order_acquire);
    for (;;) {
        if (atomic_load_explicit(&shutdown_signal,
                                 memory_order_acquire) != 0u) {
            return false;
        }
        if (atomic_compare_exchange_weak_explicit(
                &system_reset_signal, &expected, 0u,
                memory_order_release, memory_order_acquire)) {
            return true;
        }
        /* expected 已被 atomic_compare_exchange_weak_explicit 更新为最新 SRS, retry */
    }
}
