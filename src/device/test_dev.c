//
// Created by liujilan on 2026/5/24.
// test_dev 实现 — sifive_test 兼容外设的两入口 fanout (SET/CLEAR → PLIC).
//
// 接口形态 + monitor 模型 + lifecycle 见 test_dev.h 顶段 doc。
// 地址布局见 config.h TEST_DEV_* 宏。
// 报错风格见 dummy.txt §5; "0=成功 / 非0=cause" 接口约定见 dummy.txt §9。
//

#include "test_dev.h"

#include <limits.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config.h"          // TEST_DEV_*
#include "platform/bus.h"    // mmio_dev_t / bus_register_mmio
#include "platform/plic.h"   // device_set_pending / device_clear_pending
#include "riscv.h"           // CAUSE_LOAD/STORE_ACCESS_FAULT
#include "runtime.h"         // system_reset_signal / shutdown_signal


// ----------------------------------------------------------------------------
// file-static state — sifive_test FINISHER 解析后的单个 main-readable 字段
// ----------------------------------------------------------------------------
//
// _Atomic int + INT32_MIN sentinel "未设置": shutdown 是一次性事件, 多 hart 同时
// 写 magic 时走 first-writer-wins 语义 — 第一个 CAS 抢占 exit_code 的 hart 决定退
// 出码, 后续 hart 写 silent ignore (跟 QEMU last-writer-wins-via-BQL 不同, 我们的
// 行为更地道: shutdown 一次性事件不该被后续 hart 覆盖).
//
// 实装: test_dev_write PASS/FAIL 路径走 atomic_compare_exchange_strong (expected=
// INT32_MIN, desired=code). CAS 成才调 shutdown_signal_set_bit(NORMAL_EXIT); 失败
// 说明已有 hart 抢过, 直接返不动. test_dev_reset (RESET-and-continue 后) 把
// sentinel 还原成 INT32_MIN 让下次 magic 还能抢. test_dev_get_exit_code 把 sentinel
// 还原成 0 兜底 (main 端 "没跑到 FINISHER" 视图跟改前兼容).
//
// reset_req 字段已废除 — reset 触发改用 SRS bit SYSRESET_BIT_TEST_RESET 表达。
#define TEST_DEV_EXIT_SENTINEL  INT32_MIN
static _Atomic int test_dev_exit_code = TEST_DEV_EXIT_SENTINEL;


// ----------------------------------------------------------------------------
// test_dev_read / test_dev_write — bus 派发入口
// ----------------------------------------------------------------------------
//
// 接口语义 (dummy.txt §9):
//   返 0    = 成功; read 路径返字段值 / write 路径执行 fanout
//   返非 0  = cause (RV spec exception code); bus 透传给 trap_raise_exception
//
// fault 政策 (跟 plic_read/write 一致):
//   - size != 4 / off 未对齐 → CAUSE_*_ACCESS_FAULT
//   - 其他 reserved offset → silent ignore (R 返 0 / W 丢弃)
//
// 无锁 — test_dev 无内部状态, fanout 调 plic.device_set/clear_pending, plic 内 rwlock
// 已封。read 全部返 0 (finisher 读语义未定义, 简化全 0)。

static int test_dev_read(void *ctx, uint32_t off, void *buf, uint32_t size) {
    (void)ctx;
    if (size != 4u) { return CAUSE_LOAD_ACCESS_FAULT; }
    if ((off & 0x3u) != 0u) { return CAUSE_LOAD_ACCESS_FAULT; }

    uint32_t value = 0;
    memcpy(buf, &value, 4);
    return 0;
}

static int test_dev_write(void *ctx, uint32_t off, const void *buf, uint32_t size) {
    (void)ctx;
    if (size != 4u) { return CAUSE_STORE_ACCESS_FAULT; }
    if ((off & 0x3u) != 0u) { return CAUSE_STORE_ACCESS_FAULT; }

    uint32_t value;
    memcpy(&value, buf, 4);

    if (off == (uint32_t)TEST_DEV_SET_OFF) {
        device_set_pending(value);
    } else if (off == (uint32_t)TEST_DEV_CLEAR_OFF) {
        device_clear_pending(value);
    } else if (off == (uint32_t)TEST_DEV_SIFIVE_OFF) {
        /* sifive_test FINISHER: cmd = low 16; arg = high 16 (FAIL exit code).
           PASS/FAIL → CAS exit_code from sentinel → code (first-writer-wins);
           成才调 shutdown_signal_set_bit(NORMAL_EXIT). CAS 失败说明已有 hart 抢过,
           ignore (跟 QEMU last-writer-wins-via-BQL 不同; 我们语义更地道 — shutdown
           一次性事件不该被后续 hart 覆盖).
           RESET → system_reset_signal_set_bit(TEST_RESET) (非 ABORT, main try_clear
           continue, SDS 不动 — timer / uart reader 跨 reset 一直跑, 跟真硬件 RTC
           oscillator + UART pin always-on 一致). RESET 不抢 exit_code (RESET-and-
           continue 路径, exit_code 由后续 PASS/FAIL 决定; test_dev_reset 把 sentinel
           还原).
           内存序: CAS release-acquire (CAS 成 release, 失败 acquire); 跟 shutdown_
           signal_set_bit 内 atomic_fetch_or release 共同建立 happens-before; main while
           读 SRS acquire 退出后再 atomic_load exit_code (acquire), 跨线程可见. */
        uint32_t cmd = value & 0xFFFFu;
        uint32_t arg = (value >> 16) & 0xFFFFu;
        int desired;
        switch (cmd) {
          case TEST_DEV_FINISHER_PASS:
          case TEST_DEV_FINISHER_FAIL:
            desired = (cmd == TEST_DEV_FINISHER_PASS) ? 0 : (int)arg;
            int expected = TEST_DEV_EXIT_SENTINEL;
            if (atomic_compare_exchange_strong_explicit(&test_dev_exit_code,
                                                       &expected, desired,
                                                       memory_order_release,
                                                       memory_order_acquire)) {
                shutdown_signal_set_bit(SHUTDOWN_BIT_NORMAL_EXIT);
            }
            /* CAS 失败: 已有 hart 抢过, ignore */
            break;
          case TEST_DEV_FINISHER_RESET:
            system_reset_signal_set_bit(SYSRESET_BIT_TEST_RESET);
            break;
          /* 其他 cmd: silent (兼容未来扩展) */
        }
    }
    /* 其他 reserved offset silent ignore */
    return 0;
}


// ----------------------------------------------------------------------------
// lifecycle: test_dev_init / test_dev_reset / test_dev_destroy
// ----------------------------------------------------------------------------

int test_dev_init(void) {
    mmio_dev_t dev = {
        .gpa_start = (uint32_t)TEST_DEV_BASE,
        .gpa_end   = (uint32_t)(TEST_DEV_BASE + TEST_DEV_SIZE),
        .ctx       = NULL,                /* 无状态 — bus 不解读, callback 内 (void)ctx; */
        .read      = test_dev_read,
        .write     = test_dev_write,
        .name      = "test_dev",
    };
    if (bus_register_mmio(&dev) != 0) {
        fprintf(stderr, "test_dev_init: bus_register_mmio failed" EOL);
        return -1;
    }
    return 0;
}

int test_dev_reset(void) {
    /* system reset 每 iter (含 RESET cmd 触发的 continue path): exit_code 还原
       sentinel 让 RESET-and-continue 后下次 PASS/FAIL CAS 还能抢. atomic_store
       release 跟 test_dev_write CAS / test_dev_get_exit_code acquire 配套 (虽然
       reset 串行调时无并发, atomic 体例统一). */
    atomic_store_explicit(&test_dev_exit_code, TEST_DEV_EXIT_SENTINEL,
                          memory_order_release);
    return 0;
}

void test_dev_destroy(void) {
    /* 无资源 cleanup; 跟 plic_destroy / clint_destroy 同 void 签名留作 lifecycle 对称 */
}


// ----------------------------------------------------------------------------
// 外部接口: main 端读取 exit_code
// ----------------------------------------------------------------------------
//
// 调用约束: main 在 while 退出之后 (atomic_load SRS 非 0 acquire 之后) 调用; 单次
// 读取, 无并发。
//
// reset_req 字段 + test_dev_consume_reset_request 已废除 — reset 触发改用 SRS bit
// SYSRESET_BIT_TEST_RESET 表达, main 直接读 SRS bit, 不需要 consume 式接口。

int test_dev_get_exit_code(void) {
    int v = atomic_load_explicit(&test_dev_exit_code, memory_order_acquire);
    return (v == TEST_DEV_EXIT_SENTINEL) ? 0 : v;
}
