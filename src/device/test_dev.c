//
// Created by liujilan on 2026/5/24.
// test_dev 实现 — sifive_test 兼容外设的两入口 fanout (SET/CLEAR → PLIC).
//
// 接口形态 + monitor 模型 + lifecycle 见 test_dev.h 顶段 doc。
// 地址布局见 config.h TEST_DEV_* 宏。
// 报错风格见 dummy.txt §5; "0=成功 / 非0=cause" 接口约定见 dummy.txt §9。
//

#include "test_dev.h"

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
// plain int 不 _Atomic — 跨线程可见性靠 SRS/SDS 的 atomic release-acquire 同步:
//   - test_dev_write 内先写 exit_code (plain), 然后 shutdown_signal_set_bit 内
//     atomic_fetch_or release; main while 读 SRS (acquire) 退出后再读 exit_code,
//     plain 字段通过 atomic release-acquire 边界跨线程可见 (dummy.txt §7 monitor 模型)
//   - 唯一写入点 = test_dev_write (hart 主线程); 唯一读入点 = main while 之后 (同线程,
//     但跨 atomic 边界); 不存在并发写
//
// 默认 0 (BSS) — PASS exit_code=0 是恒等, FAIL 路径写入新值。RESET 路径不写 exit_code
// (返 0 默认, 因为 RESET 是 reset-and-continue, 真退出时按当时是否还有 PASS/FAIL 决定)。
//
// reset_req 字段已废除 — reset 触发改用 SRS bit SYSRESET_BIT_TEST_RESET 表达。
static int test_dev_exit_code = 0;


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
    if (size != 4u)              return CAUSE_LOAD_ACCESS_FAULT;
    if ((off & 0x3u) != 0u)      return CAUSE_LOAD_ACCESS_FAULT;

    uint32_t value = 0;
    memcpy(buf, &value, 4);
    return 0;
}

static int test_dev_write(void *ctx, uint32_t off, const void *buf, uint32_t size) {
    (void)ctx;
    if (size != 4u)              return CAUSE_STORE_ACCESS_FAULT;
    if ((off & 0x3u) != 0u)      return CAUSE_STORE_ACCESS_FAULT;

    uint32_t value;
    memcpy(&value, buf, 4);

    if (off == (uint32_t)TEST_DEV_SET_OFF) {
        device_set_pending(value);
    } else if (off == (uint32_t)TEST_DEV_CLEAR_OFF) {
        device_clear_pending(value);
    } else if (off == (uint32_t)TEST_DEV_SIFIVE_OFF) {
        /* sifive_test FINISHER: cmd = low 16; arg = high 16 (FAIL exit code).
           PASS/FAIL → shutdown_signal_set_bit(NORMAL_EXIT) (顺序 B 蕴含 SRS
           BIT_SHUTDOWN_TRIGGER; ABORT_MASK 命中, main cleanup + return exit_code).
           RESET → system_reset_signal_set_bit(TEST_RESET) (非 ABORT, main try_clear
           continue, SDS 不动 — timer / uart reader 跨 reset 一直跑, 跟真硬件 RTC
           oscillator + UART pin always-on 一致).
           写入顺序: 先 plain 字段 (exit_code), 后 set_bit 内 atomic_fetch_or release —
           release-acquire 跟 main while 读取建立 happens-before, plain 字段可见. */
        uint32_t cmd = value & 0xFFFFu;
        uint32_t arg = (value >> 16) & 0xFFFFu;
        switch (cmd) {
          case TEST_DEV_FINISHER_PASS:
            test_dev_exit_code = 0;
            shutdown_signal_set_bit(SHUTDOWN_BIT_NORMAL_EXIT);
            break;
          case TEST_DEV_FINISHER_FAIL:
            test_dev_exit_code = (int)arg;
            shutdown_signal_set_bit(SHUTDOWN_BIT_NORMAL_EXIT);
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
        fprintf(stderr, "test_dev_init: bus_register_mmio failed\n");
        return -1;
    }
    return 0;
}

int test_dev_reset(void) {
    /* system reset 每 iter (含 RESET cmd 触发的 continue path): exit_code 清.
       不持锁 — 跟 plic_reset / uart_reset 内"system reset 时 dispatcher 主帧停, 无并发"
       同前提. */
    test_dev_exit_code = 0;
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
    return test_dev_exit_code;
}
