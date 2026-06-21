//
// jit/smc.c —— SMC chain 隐式触发路径实装 (SIGSEGV handler + page_dirty bitmap +
// mprotect API + dispatcher 顶扫 consume).
//
// 协议 / 顺序 / async-signal-safe 约束详 smc.h 顶段 doc.
//

#define _GNU_SOURCE          /* SA_SIGINFO / siginfo_t (POSIX 2001 默认开, 多余但
                                跟既有 src 文件体例对齐) */

#include "jit/smc.h"

#include <errno.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>           /* fprintf — 仅 smc_init / smc_destroy 用, handler 内不调 */
#include <string.h>          /* memset / strerror */
#include <sys/mman.h>        /* mprotect / PROT_READ / PROT_WRITE */
#include <unistd.h>          /* _exit (handler 兜底用; async-signal-safe) */

#include "config.h"          /* GUEST_RAM_START / GUEST_RAM_SIZE / GUEST_RAM_NPAGES */
#include "debug.h"           /* EOL (跟既有 src 体例) */
#include "platform/ram.h"    /* host_ram_base (handler 内反查 host_addr → page_idx) */


// ============================================================================
// file-static 状态
// ============================================================================

/* page_dirty bitmap: per-page 1 bit, GUEST_RAM_NPAGES / 64 个 uint64_t.
 *
 * 当前 GUEST_RAM_NPAGES = 32768 (128MB / 4KB) → 32768/64 = 512 个 _Atomic
 * uint64_t = 4096 byte, file-static 0 初始化. atomic_fetch_or (release) /
 * atomic_fetch_and (acq_rel) lock-free, SIGSEGV handler 内可调. */
static _Atomic uint64_t page_dirty[GUEST_RAM_NPAGES / 64];

/* sigaction 老 handler 保存, smc_destroy 恢复 (跟 runtime_restore_signal_handlers
 * 体例; 进程退出时 OS 也会自动还, 这一步主要是 lifecycle 对称 + valgrind clean). */
static struct sigaction old_sigsegv;


// ============================================================================
// SIGSEGV handler (async-signal-safe; smc.h 顶段 doc 严格遵守清单)
// ============================================================================

static void smc_signal_handler(int sig, siginfo_t *si, void *uctx) {
    (void)sig;
    (void)uctx;

    /* fault_addr 落 host_ram 区判定: 无符号下溢比较 (跟 IS_GPA_RAM 体例), 一步
     * sub + 一步 cmp. host_ram_base 一旦 ram_init 完成就不变, handler 内裸读 OK. */
    uintptr_t fault = (uintptr_t)si->si_addr;
    uintptr_t base  = (uintptr_t)host_ram_base;
    uintptr_t off   = fault - base;          /* 下溢成大值时下方 cmp 会 false */
    if (off >= (uintptr_t)GUEST_RAM_SIZE) {
        /* 真异常 (非 SMC) — fault_addr 不在 guest RAM 区. async-signal-safe
         * 不能调 fprintf / abort (abort 内部 raise + cleanup 不全 safe);
         * 用 _exit (POSIX async-signal-safe 白名单). exit code 139 = 128 + 11
         * (SIGSEGV 标准). */
        _exit(139);
    }

    uint32_t page_idx = (uint32_t)(off >> 12);

    /* plan §1.17.3 SMP release 顺序:
     *   1. atomic_fetch_or page_dirty (release)  ← bitmap 先标
     *   2. smc_unprotect_page (mprotect R/W)     ← 然后开锁让 store 成功
     *
     * release 语义: 让 dispatcher 顶扫 atomic_load (acquire) 看到 bit set 时,
     * 之前所有内存修改 (无, 但保 happens-before 不让编译器乱序) visible. */
    atomic_fetch_or_explicit(&page_dirty[page_idx / 64u],
                             (uint64_t)1u << (page_idx % 64u),
                             memory_order_release);

    smc_unprotect_page(page_idx);

    /* return → kernel 重执行触发 fault 的 host store 指令; 此时 page 已 R/W,
     * store 成功 (这一次的 store 不进 page_dirty 通路, 但 page 已标 dirty,
     * dispatcher 下次顶扫会 invalidate JIT 块, 后续重 compile 时再重 mprotect). */
}


// ============================================================================
// mprotect API (caller: jit_cache_install / smc_signal_handler)
// ============================================================================

void smc_protect_page(uint32_t page_idx) {
    /* host_addr = host_ram_base + page_idx * 4096; mprotect 要 page-aligned,
     * mmap 起点 + page_idx * 4KB 天然对齐 (mmap 返 page-aligned 起点). */
    void *host = (uint8_t *)host_ram_base + (size_t)page_idx * 4096u;
    /* PROT_READ only — guest 读代码段 OK (interpret / JIT walker fetch 都是
     * load), 写触发 SIGSEGV → handler. PROT_EXEC 不要 (guest RAM 在 host 不
     * 需 exec, JIT host code 在 asmjit JitRuntime 自管 RX 段). */
    if (mprotect(host, 4096u, PROT_READ) != 0) {
        /* T1.a 起步: 非 handler context, 可 fprintf; 真撞算半 fatal 但不 abort
         * (跟 ram_init 失败逻辑对齐, 让 main 处理). */
        fprintf(stderr, "smc_protect_page(%u) mprotect failed: %s" EOL,
                page_idx, strerror(errno));
    }
}

void smc_unprotect_page(uint32_t page_idx) {
    void *host = (uint8_t *)host_ram_base + (size_t)page_idx * 4096u;
    /* PROT_READ | PROT_WRITE — 恢复 mmap 初值 (ram.c mmap 初始 PROT_READ |
     * PROT_WRITE). handler 内调时不可 fprintf — 但 mprotect 失败几乎不可能
     * (page-aligned + 来自合法 mmap 区 + 不调 fail 跟踪), 静默 (void)cast. */
    (void)mprotect(host, 4096u, PROT_READ | PROT_WRITE);
}


// ============================================================================
// dispatcher 顶扫 consume (caller: dispatcher 主循环顶)
// ============================================================================

bool smc_consume_dirty(uint32_t *out_page_idx) {
    /* 简单 linear scan — 32768 page / 64 = 512 word, cold path (dispatcher 顶
     * 每 iter 调一次, 99% 情况整数组全 0 顺扫不 hit). 真撞 perf 退化加快路 hint
     * (eg. atomic dirty_word_count counter). */
    for (size_t w = 0; w < (size_t)GUEST_RAM_NPAGES / 64u; w++) {
        uint64_t v = atomic_load_explicit(&page_dirty[w], memory_order_acquire);
        while (v != 0u) {
            int b = __builtin_ctzll(v);
            uint64_t mask = (uint64_t)1u << b;
            /* atomic_fetch_and 返原值; 若原值这 bit = 1 表示本 hart 真清掉 (CAS
             * 赢家), 返 true; 若 = 0 表示别 hart 已 consume, 继续局部 v 扫下个 bit. */
            uint64_t prev = atomic_fetch_and_explicit(&page_dirty[w],
                                                       ~mask,
                                                       memory_order_acq_rel);
            if (prev & mask) {
                *out_page_idx = (uint32_t)(w * 64u + (uint32_t)b);
                return true;
            }
            /* lost — 清 v 内同 bit 继续 (避免重扫已被别 hart consume 的 bit) */
            v &= ~mask;
        }
    }
    return false;
}


// ============================================================================
// lifecycle (main POR / teardown 一次配对)
// ============================================================================

int smc_init(void) {
    /* page_dirty 已 static 0 初始化, 不需显式 clear; lifecycle 对称 + 防多次
     * smc_init 调时残留 (但 main 只调一次, 防御性而已). */
    for (size_t w = 0; w < (size_t)GUEST_RAM_NPAGES / 64u; w++) {
        atomic_store_explicit(&page_dirty[w], 0u, memory_order_relaxed);
    }

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = smc_signal_handler;
    sigemptyset(&sa.sa_mask);            /* handler 重入安全, 不 block 其他信号 */
    sa.sa_flags = SA_SIGINFO;            /* 用 sa_sigaction 路径, 需要 siginfo_t.si_addr */
    if (sigaction(SIGSEGV, &sa, &old_sigsegv) != 0) {
        fprintf(stderr, "smc_init: sigaction(SIGSEGV) failed: %s" EOL,
                strerror(errno));
        return -1;
    }
    return 0;
}

void smc_destroy(void) {
    /* 恢复 SIGSEGV old handler (跟 runtime_restore_signal_handlers 体例 — fail
     * 只 fprintf 不传染, POR 收尾本就单向退出, fail 不阻塞 cleanup). */
    if (sigaction(SIGSEGV, &old_sigsegv, NULL) != 0) {
        fprintf(stderr, "smc_destroy: sigaction(SIGSEGV) failed: %s" EOL,
                strerror(errno));
    }
}
