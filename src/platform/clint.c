//
// CLINT 实现 — mtime / mtimecmp[N] / msip[N] MMIO 寄存器 + bus 注册 + timer
// 辅助线程 (file-static timer_run, 异步累加 atomic clint.mtime)。
//
// 接口形态见 clint.h; 地址布局见 config.h CLINT_* 宏; 注册流程见 platform/bus.h。
// 报错风格见 dummy.txt §5 (clint_init 失败 fprintf "why" + return -1; read/write
// fn 走 dummy.txt §9 "0=成功 / 非0=cause" 接口约定)。
// shared 字段 atomic + monitor 模型 见 dummy.txt §7。
// timer thread spawn / join 协议 (谁 spawn 谁 join) 见 dummy.txt §12。
//

#define _POSIX_C_SOURCE 200809L   // clock_gettime / clock_nanosleep / TIMER_ABSTIME

#include "clint.h"

#include <errno.h>
#include <inttypes.h>   // PRIu64 (timer_log_stop mtime 输出)
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "config.h"          // CLINT_* / MAX_HARTS / TIMEBASE_PER_WAKE / TIMER_WAKE_INTERVAL_NS
#include "core/wfi.h"        // wfi_kick (mtip 0→1 翻转 + msip 写入唤醒 WFI hart)
#include "platform/bus.h"    // mmio_dev_t / bus_register_mmio
#include "riscv.h"           // CAUSE_LOAD/STORE_ACCESS_FAULT
#include "runtime.h"         // system_reset_signal / shutdown_signal


// ----------------------------------------------------------------------------
// CLINT 内部状态 (shared; timer 辅助线程 + dispatcher 主帧跨线程读写)
// ----------------------------------------------------------------------------
//
// 字段类型 _Atomic 是 dummy.txt §7 关键约束 3 — RV spec 允许别的 hart 写另一个
// hart 的 mtimecmp / msip (M-mode IPI / timer broadcast), 即使单 hart 也得
// atomic 满足跨线程 host 内存安全。memory_order_relaxed 起步 (跟 plan §1.9
// SMP 预留同形态); timer thread 写 mtime release, consumer is_clint_timer_pending
// 读 mtime acquire — producer/consumer 配对建立 happens-before, 见 dummy.txt §7
// monitor 模型段。
//
// 静态全局: CLINT 是单例 (整个系统只有一个); 跟 ram_init 的 host mmap 同性质。
// 通过 mmio_dev_t.ctx = &clint 透传给 read/write fn — 风格统一其他多实例 device
// (例 UART), fn 内 (void)ctx; 抑制 unused warning, 但仍接收 ctx 参数, 接口风格
// 一致 (C-style OOP "this 指针")。
//
// timer_thread 字段: pthread_t 句柄, clint_start_timer_thread 写 / clint_join_
// timer_thread 读 (跟 dummy.txt §12 "谁 spawn 谁 join" — 实际 spawn 调用方 =
// main, 但 pthread_t 是 clint 内部状态, 由 clint_* 接口封装)。
// BSS 默认 0 init; spawn fail 时 pthread_create POSIX 规范不修改 thread 参数,
// 仍保持 0 — clint_join_timer_thread 内 pthread_join(0, NULL) glibc 下返 ESRCH
// 容错 (详 clint.h 顶段)。
static struct {
    _Atomic u64_t    mtime;
    _Atomic u64_t    mtimecmps[MAX_HARTS];
    _Atomic u32_t    msip[MAX_HARTS];
    pthread_t        timer_thread;
} clint;


// ----------------------------------------------------------------------------
// per-hart MTIP cache slot (dummy.txt §14)
// ----------------------------------------------------------------------------
//
// 由来: csr_mip_read 合成 MTIP 之前是 is_clint_timer_pending 内 2 atomic_load
// (mtime + mtimecmps[hartid]) + 比较; 升级成 writer 维护 atomic cache 后, reader
// 单 atomic_load(mtip[hartid]), 跟 PLIC ctx_eip 同体例。
//
// writer 三类:
//   - timer thread tick: atomic_fetch_add mtime 后, 对每 hart recompute mtip
//   - guest 写 mtimecmps[i]: 影响 hart i 单 hart, recompute mtip[i]
//   - guest 写 mtime: RV spec 允许 (虽不常用), 影响所有 hart, recompute_all_mtip
//
// race 处理: 严格 per-hart pthread_mutex_t mtip_lock — writer 进锁 → 重 load 两
// inputs → compute → atomic_store → unlock。timer thread + guest writer 都进同
// 把锁, 即使两 writer race 也保证 mtip 最终值反映最新 inputs (dummy.txt §14 体例 5)。
//
// 0→1 翻转处调 wfi_kick(hartid) 唤醒可能在 WFI cond_wait 的 hart。
//
// slot struct: _Alignas(64) + char _pad[64] 是 dummy.txt §14 体例 1 (假共享防御);
// 当前 MAX_HARTS=1 用不上, SMP 上线时直接受益。
typedef struct {
    _Alignas(64) _Atomic u32_t   mtip;  /* 0 / 1: cached (mtime >= mtimecmps[hartid]).
                                            _Alignas 在第一 member: alignof(slot) = 64,
                                            struct 按 cache line 对齐 (cpu_t.regs 同体例;
                                            dummy.txt §14 体例 1) */
    pthread_mutex_t mtip_lock;          /* recompute race 保护 (writer 进锁) */
    char            _pad[64];           /* cache line padding 占位 (dummy.txt §14) */
} clint_per_hart_slot_t;

static clint_per_hart_slot_t clint_per_hart[MAX_HARTS];


// ----------------------------------------------------------------------------
// MTIP cache recompute helpers (dummy.txt §14 体例 5)
// ----------------------------------------------------------------------------
//
// clint_recompute_mtip(hartid) — 重算单 hart 的 mtip cache:
//   lock(mtip_lock) → 重 load mtime + mtimecmps[hartid] → compute new_mtip
//   → 比 old_mtip, 0→1 调 wfi_kick(hartid) → atomic_store mtip → unlock。
//
// 注意 kick 在 atomic_store 之前还是之后: 之后更安全 (避免 hart 醒来 predicate
// re-check 时 mtip 还没更新看不到, 又睡回去 — 不算错但多一次假醒)。但本设计选
// 先 kick 再 store —— 为什么? 因为 atomic_store(release) 跟 hart 端 atomic_load
// (acquire) 配对建立 happens-before, store 是"对外公布"的瞬间; kick 在 store
// 后还是先无所谓 (kick 自带 pthread_mutex 进入 + cond_signal 已经建立 happens-
// before, hart 醒来 re-check 时 atomic_load mtip 必看见新值, 因为 hart 必 lock
// wfi_mutex 之后才 cond_wait, kick 之前必 lock wfi_mutex, 锁就形成 ordering 屏障)。
// 实装上 store 紧跟 kick 也行, 当前选 store 后 kick (更直觉)。
static void clint_recompute_mtip(uint32_t hartid) {
    if (hartid >= MAX_HARTS) return;     /* defensive; 上层调用方应已 bound check */

    clint_per_hart_slot_t *s = &clint_per_hart[hartid];
    pthread_mutex_lock(&s->mtip_lock);

    u64_t    now      = atomic_load_explicit(&clint.mtime,             memory_order_acquire);
    u64_t    cmp      = atomic_load_explicit(&clint.mtimecmps[hartid], memory_order_acquire);
    u32_t    new_mtip = (now >= cmp) ? 1u : 0u;
    u32_t    old_mtip = atomic_load_explicit(&s->mtip, memory_order_relaxed);

    if (old_mtip != new_mtip) {
        atomic_store_explicit(&s->mtip, new_mtip, memory_order_release);
        if (new_mtip == 1u) {
            wfi_kick(hartid);            /* mtip 0→1 → 唤醒可能在 WFI 的 hart */
        }
    }

    pthread_mutex_unlock(&s->mtip_lock);
}

// clint_recompute_all_mtip — for i in 0..n_harts 调上面 (timer thread tick +
// guest 写 mtime 都用); 各 hart 进自己的 mtip_lock, 不同 hart 之间不串行。
// 用 n_harts 不 cap: 只重算存在的 hart, 省 phantom hart 空转 + 避免 spurious kick
// 不存在的 slot (timer thread 每 ms 都调, phantom 多锁多算纯浪费)。
static void clint_recompute_all_mtip(void) {
    for (uint32_t i = 0; i < n_harts; i++) {
        clint_recompute_mtip(i);
    }
}


// ----------------------------------------------------------------------------
// clint_read / clint_write — bus 派发入口
// ----------------------------------------------------------------------------
//
// 接口语义 (dummy.txt §9):
//   返 0    = 成功
//   返非 0  = cause (RV spec exception code); bus 透传给 trap_raise_exception
//
// 当前形态:
//   - size = 4 only; size = 1 / 2 / 8 一律 CAUSE_*_ACCESS_FAULT (跟 QEMU/Spike)
//   - off 必须 4 对齐; 否则 CAUSE_*_ACCESS_FAULT
//   - off 越界 / 区间外 → CAUSE_*_ACCESS_FAULT
//   - msip 写: 只低 1 位有效 (RV spec §3.1.9 MSIP 仅 bit 0 可写)
//   - mtimecmp / mtime 写: 跨写 (低半 → 高半) 瞬态不一致是 guest 责任; 我们不
//     做"原子 64-bit 写"保护
//
// TODO RV64: 真做 size=8 时, 处理跨 mtime / mtimecmp 字段边界的 8B 访问 (例
//            off=CLINT_MTIME_OFF-4 的 8B 访问跨 mtimecmp 末尾 + mtime 开头),
//            当前 RV32 单 sw 4 字节, 不会出现这种情况, 不展开。

static int clint_read(void *ctx, uint32_t off, void *buf, uint32_t size) {
    (void)ctx;
    if (size != 4u) { return CAUSE_LOAD_ACCESS_FAULT; }
    if ((off & 0x3u) != 0u) { return CAUSE_LOAD_ACCESS_FAULT; }

    uint32_t value = 0;
    if (off < (uint32_t)CLINT_MTIMECMP_OFF) {
        // msip[N]: 4 byte/hart
        uint32_t idx = off / 4u;
        // 第 1 类: guest 访问 idx >= n_harts 的 hart = guest bug, 返 access fault
        // (不 silent, 不拖停模拟器); return 在碰字段前 → 无副作用。用 n_harts 不 cap。
        if (idx >= n_harts) { return CAUSE_LOAD_ACCESS_FAULT; }
        value = atomic_load_explicit(&clint.msip[idx], memory_order_relaxed);
    } else if (off < (uint32_t)CLINT_MTIME_OFF) {
        // mtimecmp[N]: 8 byte/hart; 低半 / 高半由 (bo & 0x4) 决定
        uint32_t bo  = off - (uint32_t)CLINT_MTIMECMP_OFF;
        uint32_t idx = bo / 8u;
        if (idx >= n_harts) return CAUSE_LOAD_ACCESS_FAULT;   /* 第 1 类: guest fault, 见 msip read */
        u64_t    v64 = atomic_load_explicit(&clint.mtimecmps[idx], memory_order_relaxed);
        value = (bo & 0x4u) ? (uint32_t)(v64 >> 32) : (uint32_t)v64;
    } else if (off == (uint32_t)CLINT_MTIME_OFF ||
               off == (uint32_t)CLINT_MTIME_OFF + 4u) {
        // mtime 读: acquire 跟 timer thread 的 release-fetch_add 配对 (consumer
        // 看到 producer 的最新写; dummy.txt §7 monitor 模型 memory_order 配对规则)。
        u64_t    v64 = atomic_load_explicit(&clint.mtime, memory_order_acquire);
        value = (off == (uint32_t)CLINT_MTIME_OFF) ? (uint32_t)v64 : (uint32_t)(v64 >> 32);
    } else {
        return CAUSE_LOAD_ACCESS_FAULT;
    }
    memcpy(buf, &value, 4);
    return 0;
}

static int clint_write(void *ctx, uint32_t off, const void *buf, uint32_t size) {
    (void)ctx;
    if (size != 4u) { return CAUSE_STORE_ACCESS_FAULT; }
    if ((off & 0x3u) != 0u) { return CAUSE_STORE_ACCESS_FAULT; }

    uint32_t value;
    memcpy(&value, buf, 4);

    if (off < (uint32_t)CLINT_MTIMECMP_OFF) {
        uint32_t idx = off / 4u;
        // 第 1 类: guest 访问 idx >= n_harts = guest bug, 返 access fault。return 在
        // atomic_store + wfi_kick 之前 → 不会给不存在的 slot 写 msip / kick 空 slot。
        if (idx >= n_harts) { return CAUSE_STORE_ACCESS_FAULT; }
        // msip 只低 1 位有效 (RV spec §3.1.9)
        atomic_store_explicit(&clint.msip[idx], value & 0x1u, memory_order_relaxed);
        // msip 不 cache (已经是 1 atomic load), 但 wake 钩子要直接调 — hart 可能
        // 在 WFI cond_wait 等 MSIP, 写 msip 后必须 kick (dummy.txt §14 / plan)。
        // 注: msip 写 0 (clear) 也调 kick — 无害 (hart 醒来 predicate re-check 看
        // (mie & mip)=0 直接睡回去, 一次假醒可接受); 避免分支判 set/clear。
        wfi_kick(idx);
    } else if (off < (uint32_t)CLINT_MTIME_OFF) {
        uint32_t bo  = off - (uint32_t)CLINT_MTIMECMP_OFF;
        uint32_t idx = bo / 8u;
        // 第 1 类: guest fault; return 在 load-modify-store + clint_recompute_mtip 之前。
        if (idx >= n_harts) { return CAUSE_STORE_ACCESS_FAULT; }
        // 8B 字段拆 RV32 2 条 sw: load-modify-store (atomic 加载现值, 改一半, atomic
        // store 回去); 中间瞬态值不一致是 guest 责任 (见顶部 doc)。
        u64_t    cur = atomic_load_explicit(&clint.mtimecmps[idx], memory_order_relaxed);
        u64_t    nxt = (bo & 0x4u)
                       ? (cur & 0x00000000FFFFFFFFull) | ((uint64_t)value << 32)
                       : (cur & 0xFFFFFFFF00000000ull) | (uint64_t)value;
        atomic_store_explicit(&clint.mtimecmps[idx], nxt, memory_order_relaxed);
        // mtimecmps[idx] 变 → 影响 hart idx 单 hart 的 MTIP, recompute + 可能 kick
        clint_recompute_mtip(idx);
    } else if (off == (uint32_t)CLINT_MTIME_OFF ||
               off == (uint32_t)CLINT_MTIME_OFF + 4u) {
        // mtime 写: guest 主动写 mtime 是少见 (RV spec 允许但不常用); 跟 timer
        // thread 的 atomic_fetch_add 并发时是边界情况, 当前简单 load-modify-store
        // (不做 CAS loop)。memory_order_relaxed 起步, 真撞并发问题再升级 (TODO
        // 单独 RMW 安全审计, 跟 OS 真上才有现实压力)。
        u64_t    cur = atomic_load_explicit(&clint.mtime, memory_order_relaxed);
        u64_t    nxt = (off == (uint32_t)CLINT_MTIME_OFF)
                       ? (cur & 0xFFFFFFFF00000000ull) | (uint64_t)value
                       : (cur & 0x00000000FFFFFFFFull) | ((uint64_t)value << 32);
        atomic_store_explicit(&clint.mtime, nxt, memory_order_relaxed);
        // mtime 变 → 影响所有 hart 的 (mtime >= mtimecmps[i]) 比较, recompute_all
        // (RV spec 允许 guest 写 mtime, 必须全 hart 重算, 不漏)。dummy.txt §14。
        clint_recompute_all_mtip();
    } else {
        return CAUSE_STORE_ACCESS_FAULT;
    }
    return 0;
}


// ----------------------------------------------------------------------------
// timer_run — timer 辅助线程主 routine (file-static; 跟 dummy.txt §7 (b)
// 辅助线程 + monitor 模型对齐, 不持 cpu_t, 只动 shared 字段)
// ----------------------------------------------------------------------------
//
// 跨 system reset 一直跑 (跟真硬件 RTC oscillator 不掉电不停一致); 随 SDS 起停
// (shutdown_signal 非 0 自然退)。clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)
// 绝对时间唤醒避免 nanosleep 相对时间累积 drift。
//
// 错误处理 (跟 dummy.txt §5 报错风格一致):
//   - EINTR: while-retry (Linux 习惯; TIMER_ABSTIME 下极罕见)
//   - 其他 errno: fprintf + cooperative 退出 (shutdown_signal_set_bit(DEVICE_FAIL),
//                  内部按顺序 B 蕴含 SRS BIT_SHUTDOWN_TRIGGER → main 走 cleanup
//                  路径退出 + 此 routine return NULL); 不 abort, 让 main 有机会 dump
//
// trace: timer thread 不打 trace (dummy.txt §7 末段; debug 真要打是临时 fprintf
// 用完删, 不动 debug 模块)。

// 退出前打 mtime — debug 用, 跟 [main] elapsed / [dispatcher] halted 同风格输出到
// stderr, 让肉眼对照 "总累加 = wake 次数 * TIMEBASE_PER_WAKE"。受 DEBUG_CLINT_TIMER_ON
// gate (CMake 非 Release 配置才开; 见 debug.h)。
// 跟 dummy.txt §7 末段 "timer thread 不打 trace" 不冲突 — 那条是说不写 trace
// char-stream (跑期密集 fprintf 干扰), 单点 stop 时一次 fprintf 不属于此范围。
static void timer_log_stop(const char *reason) {
#ifdef DEBUG_CLINT_TIMER_ON
    u64_t    now = atomic_load_explicit(&clint.mtime, memory_order_acquire);
    fprintf(stderr, "[clint timer] stopped (%s): mtime=%" PRIu64 "" EOL, reason, now);
#else
    (void)reason;
#endif
}

static void *timer_run(void *arg) {
    (void)arg;

    struct timespec next_wake;
    if (clock_gettime(CLOCK_MONOTONIC, &next_wake) != 0) {
        fprintf(stderr, "[clint timer] clock_gettime(CLOCK_MONOTONIC) failed: %s" EOL,
                strerror(errno));
        shutdown_signal_set_bit(SHUTDOWN_BIT_DEVICE_FAIL);
        timer_log_stop("clock_gettime fail");
        return NULL;
    }

    // 主循环: SDS=1 继续, SDS=0 退。acquire 跟 main 端 atomic_store(&SDS, 0,
    // release) 配对 (consumer/producer; dummy.txt §7 monitor 模型 memory_order)。
    while (atomic_load_explicit(&shutdown_signal, memory_order_acquire) == 0u) {
        // 累加 TIMER_WAKE_INTERVAL_NS 到 next_wake (normalize tv_sec / tv_nsec)。
        next_wake.tv_nsec += (long)TIMER_WAKE_INTERVAL_NS;
        if (next_wake.tv_nsec >= 1000000000L) {
            next_wake.tv_sec  += next_wake.tv_nsec / 1000000000L;
            next_wake.tv_nsec %= 1000000000L;
        }

        int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wake, NULL);
        if (rc == EINTR) continue;            // Linux 习惯: 信号中断 retry
        if (rc != 0) {
            fprintf(stderr, "[clint timer] clock_nanosleep failed: %s" EOL,
                    strerror(rc));
            shutdown_signal_set_bit(SHUTDOWN_BIT_DEVICE_FAIL);
            timer_log_stop("clock_nanosleep fail");
            return NULL;
        }

        // release 跟 consumer (clint_read mtime) 的 acquire-load 配对, 建立
        // happens-before。is_clint_timer_pending 读 mtip cache, 不直读 mtime;
        // mtip cache 由下面 recompute_all_mtip 维护 (内部 atomic_store release)。
        atomic_fetch_add_explicit(&clint.mtime,
                                  (uint64_t)TIMEBASE_PER_WAKE,
                                  memory_order_release);

        // mtime 涨之后, 对每 hart 重算 MTIP (是 cache; 翻 0→1 时调 wfi_kick
        // 唤醒可能 WFI 的 hart)。每 hart 进自己 mtip_lock, 不互相串行。
        clint_recompute_all_mtip();
    }

    timer_log_stop("SDS!=0");
    return NULL;
}


// ----------------------------------------------------------------------------
// lifecycle: clint_init / clint_reset / clint_destroy
// ----------------------------------------------------------------------------

int clint_init(void) {
    // mtime 初值 0 (time begins from 0; RV spec OK reset state).
    atomic_store_explicit(&clint.mtime, 0, memory_order_relaxed);

    // mtimecmp 初值 UINT64_MAX (= "no timer interrupt scheduled" sentinel).
    // 跟 OpenSBI sbi_timer_init 惯例一致: RV Priv Spec §3.2.1 "MTIP pending
    // whenever mtime ≥ mtimecmp", 若初值 0 + mtime=0 → `0 >= 0` 永远 true →
    // MTIP 一直 spurious set。guest software (SBI / kernel) 显式 csrw 或 MMIO 写
    // mtimecmp 设有意义的值后, is_clint_timer_pending 才返 true。
    //
    // msip 初值 0 (no software interrupt pending).
    for (uint32_t i = 0; i < MAX_HARTS; i++) {
        atomic_store_explicit(&clint.mtimecmps[i], UINT64_MAX, memory_order_relaxed);
        atomic_store_explicit(&clint.msip[i],      0,          memory_order_relaxed);
    }

    // per-hart MTIP cache slot 初始化 (dummy.txt §14)
    // mtip 初值 0 (mtimecmp 是 UINT64_MAX 哨兵, mtime=0 < UINT64_MAX → mtip=0 一致);
    // mtip_lock 调 pthread_mutex_init, 失败走 runtime signal 通道。
    for (uint32_t i = 0; i < MAX_HARTS; i++) {
        atomic_store_explicit(&clint_per_hart[i].mtip, 0u, memory_order_relaxed);
        int rc = pthread_mutex_init(&clint_per_hart[i].mtip_lock, NULL);
        if (rc != 0) {
            fprintf(stderr, "clint_init: pthread_mutex_init(mtip_lock[%u]) failed: %s" EOL,
                    i, strerror(rc));
            // 已 init 的前 i 把锁泄漏 (init 失败路径, main 不会跑 destroy chain);
            // 跟 cpu_create 半 init 失败回滚同体例 — 报错够清楚就行, 不强求 graceful。
            return -1;
        }
    }

    // timer_thread 字段不在此处写 — clint_start_timer 才 pthread_create (dummy.txt
    // §12 "clint_init 是线程无关的, 要有一个专门的行为发出线程")。

    mmio_dev_t dev = {
        .gpa_start = (uint32_t)CLINT_BASE,
        .gpa_end   = (uint32_t)(CLINT_BASE + CLINT_SIZE),
        .ctx       = &clint,         // 单例 device, ctx 接口风格统一占位 (fn 内 (void)ctx;)
        .read      = clint_read,
        .write     = clint_write,
        .name      = "clint",
    };
    if (bus_register_mmio(&dev) != 0) {
        fprintf(stderr, "clint_init: bus_register_mmio failed" EOL);
        return -1;
    }
    return 0;
}

int clint_reset(void) {
    // system reset 每 iter: mtimecmp / msip 清回 init 时的哨兵值; mtime 不动
    // (跟真硬件 RTC oscillator 不掉电不停一致); timer 辅助线程不动 (跨 system
    // reset 持续运行, 随 SDS 才退)。
    for (uint32_t i = 0; i < MAX_HARTS; i++) {
        atomic_store_explicit(&clint.mtimecmps[i], UINT64_MAX, memory_order_relaxed);
        atomic_store_explicit(&clint.msip[i],      0,          memory_order_relaxed);
        // mtip cache 跟 mtimecmps 同步清回 0 (mtimecmp=UINT64_MAX → mtip=0 必然)。
        // 不调 wfi_kick — sleeping hart 已经走 system reset 路径出来了, 不需要从
        // mtip 视角 kick (dummy.txt §14 + plan 注脚)。
        atomic_store_explicit(&clint_per_hart[i].mtip, 0u, memory_order_relaxed);
    }
    return 0;
}

void clint_destroy(void) {
    // 纯模块 cleanup — 不含 pthread_join (timer thread 由 main 调 clint_join_
    // timer 显式回收; dummy.txt §12 谁 spawn 谁 join)。timer thread 必已 join,
    // 故 mtip_lock 此时无 writer, 销毁安全。
    atomic_store_explicit(&clint.mtime, 0, memory_order_relaxed);
    for (uint32_t i = 0; i < MAX_HARTS; i++) {
        atomic_store_explicit(&clint.mtimecmps[i], 0, memory_order_relaxed);
        atomic_store_explicit(&clint.msip[i],      0, memory_order_relaxed);
        // 销毁 per-hart mtip_lock (dummy.txt §14 + §12 init/destroy 配对)。
        // 必须在 timer_thread join 之后调 (timer thread tick 调 recompute_mtip 持锁);
        // main.c 退出顺序: clint_join_timer_thread → 各 destroy chain → clint_destroy,
        // 即此时 timer thread 已退, 无 race。
        (void)pthread_mutex_destroy(&clint_per_hart[i].mtip_lock);
    }
}


// ----------------------------------------------------------------------------
// thread lifecycle: clint_start_timer_thread / clint_join_timer_thread (dummy.txt §12)
// ----------------------------------------------------------------------------

void clint_start_timer_thread(void) {
    // 前置 check: 若 SDS 已非 0 (前面有别的 init 失败, 通过 runtime signal 通道
    // 提前通知"别再 spawn"), 跳过 pthread_create。当前 main flow 下 spawn 时
    // SDS 必=0, check 是死代码; 但跟"错误处理统一走 SRS/SDS 通道"原则一致,
    // 给未来"前面有别的 init 失败也 set SDS bit" 场景留路。
    if (atomic_load_explicit(&shutdown_signal, memory_order_acquire) != 0u) {
        return;
    }

    int rc = pthread_create(&clint.timer_thread, NULL, timer_run, NULL);
    if (rc != 0) {
        fprintf(stderr, "clint_start_timer_thread: pthread_create failed: %s" EOL,
                strerror(rc));
        // 错误走 runtime signal 通道 — shutdown_signal_set_bit 内部按顺序 B 蕴含
        // SRS BIT_SHUTDOWN_TRIGGER, main while 因 SRS 非 0 自然不进, 走 cleanup
        // 路径 (runtime.h doc + dummy.txt §12 "SDS 蕴含 SRS")。
        shutdown_signal_set_bit(SHUTDOWN_BIT_DEVICE_FAIL);
        // clint.timer_thread 保持 BSS 0 init (POSIX 7.2 pthread_create fail 不
        // 修改 thread 参数); clint_join_timer_thread 内 pthread_join(0, NULL)
        // glibc 下返 ESRCH 容错 — 不需要额外 track "是否 spawn 成功"。
    }
}

void clint_join_timer_thread(void) {
    // 调用前置: main 已 shutdown_signal_set_bit(NORMAL_EXIT); 否则 pthread_join
    // 永远 block (timer thread 在 while(SDS) 内永不退)。
    //
    // 两种 case 都安全 fprintf 不 fatal:
    //   1. 正常 case (spawn 成功 + thread 已退): pthread_join 返 0
    //   2. spawn fail case (clint.timer_thread = 0): pthread_join 在 glibc/musl
    //      下返 ESRCH "No such process", fprintf 一行不 fatal — 跟 spawn fail
    //      路径已经 fprintf 过的"pthread_create failed" 是双线索, 信息冗余
    //      但不影响正确性。已在退出路径, 不阻挡 destroy chain + return 0。
    int rc = pthread_join(clint.timer_thread, NULL);
    if (rc != 0) {
        fprintf(stderr, "clint_join_timer_thread: pthread_join failed: %s" EOL,
                strerror(rc));
    }
}


// ----------------------------------------------------------------------------
// 中断 pending 语义查询 (csr.c csr_mip_read 合成路径用)
//
// 接口约定见 clint.h 顶段; hartid 越界返 0 防御; 内部 atomic_load_explicit 跨
// hart 安全 (单 hart 时编译为 plain load, 零开销)。memory_order_acquire 跟
// timer thread 的 release-fetch_add 配对 (producer/consumer; dummy.txt §7
// monitor 模型)。
// ----------------------------------------------------------------------------

int is_clint_msip_pending(uint32_t hartid) {
    if (hartid >= MAX_HARTS) { return 0; }
    return atomic_load_explicit(&clint.msip[hartid], memory_order_acquire) ? 1 : 0;
}

int is_clint_timer_pending(uint32_t hartid) {
    if (hartid >= MAX_HARTS) { return 0; }
    // 单 atomic_load: writer (timer thread / clint_write mtime|mtimecmp)
    // 都接了 clint_recompute_mtip 钩子维护 mtip cache, reader 不再每次比 mtime/mtimecmps,
    // 跟 PLIC ctx_eip 同体例 (dummy.txt §14 + trade_off_log §T.6 同源思路)。
    // acquire 跟 recompute 内 atomic_store release 配对建立 happens-before。
    return atomic_load_explicit(&clint_per_hart[hartid].mtip, memory_order_acquire) ? 1 : 0;
}

// csr.c csr_time/timeh_read 调; consumer 接口对偶 is_clint_*_pending (acquire 跟
// timer thread release-fetch_add mtime 配对; dummy.txt §7 monitor 模型)。
uint64_t clint_read_mtime(void) {
    return atomic_load_explicit(&clint.mtime, memory_order_acquire);
}
