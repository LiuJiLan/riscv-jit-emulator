//
// Created by liujilan on 2026/5/16.
// CLINT 实现 — mtime / mtimecmp[N] / msip[N] MMIO 寄存器 + bus 注册。
//
// 接口形态见 clint.h; 地址布局见 config.h CLINT_* 宏; 注册流程见 platform/bus.h。
// 报错风格见 dummy.txt §5 (clint_init 失败 fprintf "why" + return -1; read/write
// fn 走 dummy.txt §9 "0=成功 / 非0=cause" 接口约定)。
// shared 字段 atomic 见 dummy.txt §7 (T5 timer 辅助线程跨线程读写, 必须 atomic;
// MAX_HARTS=1 单 hart 时编译为 plain load/store, 零开销)。
//
// T1 阶段语义边界: read/write 路径走通, 跨写 mtimecmp 中间瞬间值不一致是 guest
// 软件责任 (跟 QEMU 一致, 不做"原子 64-bit 写"保护); mtime 写啥读啥, 不真自动
// 推进 (那是 T5 方案 A: dispatcher.total_count 映射 / 方案 C: timer 辅助线程
// 异步累加 — 两条都不在 T1)。msip 写低 1 位 + 读, 不真触发 M-mode soft interrupt
// (mip.MSIP 联动留 T2 + T4)。
//

#include "clint.h"

#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config.h"          // CLINT_* / MAX_HARTS
#include "platform/bus.h"    // mmio_dev_t / bus_register_mmio
#include "riscv.h"           // CAUSE_LOAD/STORE_ACCESS_FAULT


// ----------------------------------------------------------------------------
// CLINT 内部状态 (shared; T5 多线程 timer 真接时跨线程读写)
// ----------------------------------------------------------------------------
//
// 字段类型 _Atomic 是 dummy.txt §7 关键约束 3 — RV spec 允许别的 hart 写另一个
// hart 的 mtimecmp / msip (M-mode IPI / timer broadcast), 即使 T1 单 hart 也得
// atomic 满足跨线程 host 内存安全。memory_order_relaxed 起步 (跟 plan §1.9
// SMP 预留同形态), fence 按需加。
//
// 静态全局: CLINT 是单例 (整个系统只有一个); 跟 ram_init 的 host mmap 同性质。
// 通过 mmio_dev_t.ctx = &clint 透传给 read/write fn — 风格统一其他多实例 device
// (例 UART), fn 内 (void)ctx; 抑制 unused warning, 但仍接收 ctx 参数, 接口风格
// 一致 (C-style OOP "this 指针")。
static struct {
    _Atomic uint64_t mtime;
    _Atomic uint64_t mtimecmps[MAX_HARTS];
    _Atomic uint32_t msip[MAX_HARTS];
} clint;


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
// TODO RV64: 未来真做 size=8 时, 处理跨 mtime / mtimecmp 字段边界的 8B 访问
//            (例 off=CLINT_MTIME_OFF-4 的 8B 访问跨 mtimecmp 末尾 + mtime 开头),
//            当前 RV32 单 sw 4 字节, 不会出现这种情况, 不展开。

static int clint_read(void *ctx, uint32_t off, void *buf, uint32_t size) {
    (void)ctx;
    if (size != 4u) return CAUSE_LOAD_ACCESS_FAULT;
    if ((off & 0x3u) != 0u) return CAUSE_LOAD_ACCESS_FAULT;

    uint32_t value = 0;
    if (off < (uint32_t)CLINT_MTIMECMP_OFF) {
        // msip[N]: 4 byte/hart
        uint32_t idx = off / 4u;
        if (idx >= MAX_HARTS) return CAUSE_LOAD_ACCESS_FAULT;
        value = atomic_load_explicit(&clint.msip[idx], memory_order_relaxed);
    } else if (off < (uint32_t)CLINT_MTIME_OFF) {
        // mtimecmp[N]: 8 byte/hart; 低半 / 高半由 (bo & 0x4) 决定
        uint32_t bo  = off - (uint32_t)CLINT_MTIMECMP_OFF;
        uint32_t idx = bo / 8u;
        if (idx >= MAX_HARTS) return CAUSE_LOAD_ACCESS_FAULT;
        uint64_t v64 = atomic_load_explicit(&clint.mtimecmps[idx], memory_order_relaxed);
        value = (bo & 0x4u) ? (uint32_t)(v64 >> 32) : (uint32_t)v64;
    } else if (off == (uint32_t)CLINT_MTIME_OFF ||
               off == (uint32_t)CLINT_MTIME_OFF + 4u) {
        uint64_t v64 = atomic_load_explicit(&clint.mtime, memory_order_relaxed);
        value = (off == (uint32_t)CLINT_MTIME_OFF) ? (uint32_t)v64 : (uint32_t)(v64 >> 32);
    } else {
        return CAUSE_LOAD_ACCESS_FAULT;
    }
    memcpy(buf, &value, 4);
    return 0;
}

static int clint_write(void *ctx, uint32_t off, const void *buf, uint32_t size) {
    (void)ctx;
    if (size != 4u) return CAUSE_STORE_ACCESS_FAULT;
    if ((off & 0x3u) != 0u) return CAUSE_STORE_ACCESS_FAULT;

    uint32_t value;
    memcpy(&value, buf, 4);

    if (off < (uint32_t)CLINT_MTIMECMP_OFF) {
        uint32_t idx = off / 4u;
        if (idx >= MAX_HARTS) return CAUSE_STORE_ACCESS_FAULT;
        // msip 只低 1 位有效 (RV spec §3.1.9)
        atomic_store_explicit(&clint.msip[idx], value & 0x1u, memory_order_relaxed);
    } else if (off < (uint32_t)CLINT_MTIME_OFF) {
        uint32_t bo  = off - (uint32_t)CLINT_MTIMECMP_OFF;
        uint32_t idx = bo / 8u;
        if (idx >= MAX_HARTS) return CAUSE_STORE_ACCESS_FAULT;
        // 8B 字段拆 RV32 2 条 sw: load-modify-store (atomic 加载现值, 改一半, atomic
        // store 回去); 中间瞬态值不一致是 guest 责任 (见顶部 doc)。
        uint64_t cur = atomic_load_explicit(&clint.mtimecmps[idx], memory_order_relaxed);
        uint64_t nxt = (bo & 0x4u)
                       ? (cur & 0x00000000FFFFFFFFull) | ((uint64_t)value << 32)
                       : (cur & 0xFFFFFFFF00000000ull) | (uint64_t)value;
        atomic_store_explicit(&clint.mtimecmps[idx], nxt, memory_order_relaxed);
    } else if (off == (uint32_t)CLINT_MTIME_OFF ||
               off == (uint32_t)CLINT_MTIME_OFF + 4u) {
        uint64_t cur = atomic_load_explicit(&clint.mtime, memory_order_relaxed);
        uint64_t nxt = (off == (uint32_t)CLINT_MTIME_OFF)
                       ? (cur & 0xFFFFFFFF00000000ull) | (uint64_t)value
                       : (cur & 0x00000000FFFFFFFFull) | ((uint64_t)value << 32);
        atomic_store_explicit(&clint.mtime, nxt, memory_order_relaxed);
    } else {
        return CAUSE_STORE_ACCESS_FAULT;
    }
    return 0;
}


// ----------------------------------------------------------------------------
// clint_init — 初始化 + 注册到 bus
// ----------------------------------------------------------------------------

int clint_init(void) {
    // mtime 初值 0 (time begins from 0; RV spec OK reset state).
    atomic_store_explicit(&clint.mtime, 0, memory_order_relaxed);

    // mtimecmp 初值 UINT64_MAX (= "no timer interrupt scheduled" sentinel).
    // 跟 OpenSBI sbi_timer_init 惯例一致: RV Priv Spec §3.2.1 "MTIP pending
    // whenever mtime ≥ mtimecmp", 若初值 0 + mtime=0 → `0 >= 0` 永远 true →
    // MTIP spurious set 整个 T2 阶段。guest software (SBI / kernel) 显式 csrw
    // 或 MMIO 写 mtimecmp 设有意义的值后, clint_timer_pending 才返 true。
    //
    // msip 初值 0 (no software interrupt pending).
    for (uint32_t i = 0; i < MAX_HARTS; i++) {
        atomic_store_explicit(&clint.mtimecmps[i], UINT64_MAX, memory_order_relaxed);
        atomic_store_explicit(&clint.msip[i],      0,          memory_order_relaxed);
    }

    mmio_dev_t dev = {
        .gpa_start = (uint32_t)CLINT_BASE,
        .gpa_end   = (uint32_t)(CLINT_BASE + CLINT_SIZE),
        .ctx       = &clint,         // 单例 device, ctx 接口风格统一占位 (fn 内 (void)ctx;)
        .read      = clint_read,
        .write     = clint_write,
        .name      = "clint",
    };
    if (bus_register_mmio(&dev) != 0) {
        fprintf(stderr, "clint_init: bus_register_mmio failed\n");
        return -1;
    }
    return 0;
}


// ----------------------------------------------------------------------------
// 中断 pending 语义查询 (csr.c csr_mip_read 合成路径用)
//
// 接口约定见 clint.h 顶段; hartid 越界返 0 防御; 内部 atomic_load_explicit 跨
// hart 安全 (单 hart 时编译为 plain load, 零开销)。memory_order_acquire 跟
// dummy.txt §7 跨线程读取异步源约定一致 (T5 timer 辅助线程 release-store
// mtime 之后, dispatcher 这边 acquire-load 看到新值)。
// ----------------------------------------------------------------------------

int clint_msip_pending(uint32_t hartid) {
    if (hartid >= MAX_HARTS) return 0;
    return atomic_load_explicit(&clint.msip[hartid], memory_order_acquire) ? 1 : 0;
}

int clint_timer_pending(uint32_t hartid) {
    if (hartid >= MAX_HARTS) return 0;
    uint64_t now = atomic_load_explicit(&clint.mtime,             memory_order_acquire);
    uint64_t cmp = atomic_load_explicit(&clint.mtimecmps[hartid], memory_order_acquire);
    return (now >= cmp) ? 1 : 0;
}


// ----------------------------------------------------------------------------
// T3 临时 mtime 步进源 (T5 timer 辅助线程上线时 grep "mtime_t3_temp" 清三点)
//
// 接口语义见 clint.h 顶段。注: RV Priv Spec §3.2.1 mtime 由 rtc_toggle 驱动,
// 跟 guest 指令执行**异步**; 本 setter "1 指令 = 1 tick" 强行同步是 T3 临时桥,
// T5 走方案 C 独立 timer 辅助线程才跟 spec 异步语义对齐.
//
// memory_order_relaxed 起步 (跟 clint_read/write mtime 路径同序; T5 timer 辅助
// 线程上线时改 release-store, 让 dispatcher acquire-load 看到 happens-before).
// ----------------------------------------------------------------------------
void clint_set_mtime_t3_temp(uint64_t v) {
    atomic_store_explicit(&clint.mtime, v, memory_order_relaxed);
}
