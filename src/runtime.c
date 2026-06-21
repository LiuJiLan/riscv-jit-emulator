//
// runtime 模块实现 — bitmap 化 + 接口函数包协议 (set 用 atomic_fetch_or,
// clear-with-condition 用 CAS-loop, shutdown 蕴含 system_reset 顺序 B);
// host signal handler + host-side stdin termios raw mode 管理。
// 详 runtime.h 顶段 doc。
//

#define _DEFAULT_SOURCE          // cfmakeraw (glibc BSD-extension; POSIX 不含)

#include "runtime.h"

#include "config.h"               // EOL (项目级 stderr 输出体例)
#include "core/wfi.h"             // wfi_kick_all (runtime_fatal 紧急唤醒所有 hart;
                                  //   runtime → core/wfi 唯一方向依赖, 详 runtime.h
                                  //   runtime_fatal 段 + wfi.h wfi_kick_all 段)

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

_Atomic uint32_t system_reset_signal = 0u;
_Atomic uint32_t shutdown_signal     = 0u;
_Atomic uint32_t external_signal_no  = 0u;

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

void runtime_fatal(const char *where, const char *detail, long bad_value) {
    // emulator 内部一致性违例 → 紧急全机停机 + main 仍可 dump。语义/分层/bit 选择
    // 详 runtime.h runtime_fatal 段。non-_Noreturn / 不 abort (call site 可能持锁)。
    fprintf(stderr, "[runtime] FATAL internal bug at %s: %s (bad value %ld) "
            "(emulator bug, not guest)" EOL, where, detail, bad_value);

    // 走顺序 B (SDS 先 SRS 后) 防 main 看 SRS!=0 走 join 时 SDS 未设 → 线程不退 hang。
    // 两侧都用独立 INTERNAL_FATAL bit (不借道 SHUTDOWN_TRIGGER; 理由见 runtime.h)。
    atomic_fetch_or_explicit(&shutdown_signal,
                             SHUTDOWN_BIT_INTERNAL_FATAL,
                             memory_order_release);
    atomic_fetch_or_explicit(&system_reset_signal,
                             SYSRESET_BIT_INTERNAL_FATAL,
                             memory_order_release);

    // 紧急唤醒所有可能在 WFI cond_wait 的 hart, 让它们立即 re-check predicate
    // (wfi_should_wake 第一条查 SRS != 0 → 退 wfi 退 main loop); fatal 不接受正常
    // shutdown 那 ≤500ms cond_timedwait tail。wfi_kick_all 用 cap, 只碰 wfi slot
    // mutex 不碰 plic/clint 锁 → 持锁 call site 调入无锁环 (详 wfi.h)。
    wfi_kick_all();
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


// ----------------------------------------------------------------------------
// host signal handler (SIGINT / SIGTERM / SIGHUP → shutdown_signal)
// ----------------------------------------------------------------------------
//
// 详 runtime.h 顶段 "external signal handler" 一节。
//
// async-signal-safe 约束 (CLAUDE.md 同 SIGSEGV handler):
//   - atomic_compare_exchange_strong_explicit: lock-free atomic, POSIX 安全
//   - atomic_fetch_or_explicit (内嵌 shutdown_signal_set_bit): 同上
//   - 不调 fprintf / malloc / pthread_* — 此 handler 三函数体内皆无

static void runtime_signal_handler(int signum) {
    // first-wins: 仅 expected=0 时写 signum, 否则 CAS fail 不 retry; 第二次信号
    // 丢弃 signum 但仍走下面 set_bit (fetch_or idempotent 无害)。
    uint32_t expected = 0u;
    (void)atomic_compare_exchange_strong_explicit(
        &external_signal_no, &expected, (uint32_t)signum,
        memory_order_release, memory_order_relaxed);

    // 走顺序 B (SDS bit 先, SRS BIT_SHUTDOWN_TRIGGER 后); main while 看 SRS 退出,
    // SDS-controlled 辅助线程 100ms poll 看 SDS 自然退。
    shutdown_signal_set_bit(SHUTDOWN_BIT_EXTERNAL_SIGNAL);
}

static int runtime_install_one(int sig) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = runtime_signal_handler;
    sigemptyset(&sa.sa_mask);          /* handler 重入安全, 不 block 其他信号 */
    sa.sa_flags = SA_RESTART;          /* read / poll 不返 EINTR, 减少 reader
                                          thread 内部 retry 噪音 (跟 uart_reader_run
                                          现有 EINTR continue 正交, 多一层防护) */
    if (sigaction(sig, &sa, NULL) != 0) {
        fprintf(stderr, "runtime_install_signal_handlers: sigaction(%d) failed: %s" EOL,
                sig, strerror(errno));
        return -1;
    }
    return 0;
}

int runtime_install_signal_handlers(void) {
    /* SIGINT  ^C / 终端中断          → exit 128+2  = 130
       SIGTERM kill 默认 / CLion 停止 → exit 128+15 = 143
       SIGHUP  父进程关闭 / 网页断    → exit 128+1  = 129
       SIGQUIT 不挂 — raw mode 关 ISIG 后 ^\ 进 guest, 由 guest 处理 */
    if (runtime_install_one(SIGINT)  != 0) { return -1; }
    if (runtime_install_one(SIGTERM) != 0) { return -1; }
    if (runtime_install_one(SIGHUP)  != 0) { return -1; }
    return 0;
}

void runtime_restore_signal_handlers(void) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = SIG_DFL;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    /* fail 路径只 fprintf 不传染; POR 收尾本就走单向退出, 这里 fail 不阻塞 cleanup */
    if (sigaction(SIGINT,  &sa, NULL) != 0)
        fprintf(stderr, "runtime_restore_signal_handlers: sigaction(SIGINT)  failed: %s" EOL,  strerror(errno));
    if (sigaction(SIGTERM, &sa, NULL) != 0)
        fprintf(stderr, "runtime_restore_signal_handlers: sigaction(SIGTERM) failed: %s" EOL, strerror(errno));
    if (sigaction(SIGHUP,  &sa, NULL) != 0)
        fprintf(stderr, "runtime_restore_signal_handlers: sigaction(SIGHUP)  failed: %s" EOL,  strerror(errno));
}


// ----------------------------------------------------------------------------
// host-side stdin termios raw mode
// ----------------------------------------------------------------------------
//
// 详 runtime.h 顶段 "host-side stdin raw mode" 节。
//
// state: 单 process 一份 (stdin 是 process 唯一资源); v1 假设单一 caller
// (uart reader thread); 真有第二个 caller 时再升级到 ref count / nested 形态.

static struct {
    struct termios saved_tio;
    int            tio_saved;   /* 0 = 没改 (cooked / 退化路径) / 1 = 已 set raw */
} runtime_stdin_state;

void runtime_stdin_enter_raw(void) {
    runtime_stdin_state.tio_saved = 0;

    /* isatty=0 (pipe/file input, e.g. fixture 跑批) → skip 退化 cooked, 不报错 */
    if (!isatty(STDIN_FILENO)) {
        return;
    }

    if (tcgetattr(STDIN_FILENO, &runtime_stdin_state.saved_tio) != 0) {
        fprintf(stderr, "runtime_stdin_enter_raw: tcgetattr failed (%s); "
                "stdin remains cooked" EOL, strerror(errno));
        return;
    }

    struct termios raw_tio = runtime_stdin_state.saved_tio;
    cfmakeraw(&raw_tio);
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw_tio) != 0) {
        fprintf(stderr, "runtime_stdin_enter_raw: tcsetattr(raw) failed (%s); "
                "stdin remains cooked" EOL, strerror(errno));
        return;
    }

    runtime_stdin_state.tio_saved = 1;
}

void runtime_stdin_exit_raw(void) {
    /* enter 退化路径 (isatty=0 / tcgetattr-tcsetattr fail) tio_saved 仍 0,
       skip restore; abort path (assert/segfault) 走不到此调, user 需 stty sane. */
    if (!runtime_stdin_state.tio_saved) { return; }

    if (tcsetattr(STDIN_FILENO, TCSANOW, &runtime_stdin_state.saved_tio) != 0) {
        fprintf(stderr, "runtime_stdin_exit_raw: tcsetattr(restore) failed (%s); "
                "run 'stty sane' to recover stdin" EOL, strerror(errno));
    }
    runtime_stdin_state.tio_saved = 0;
}
