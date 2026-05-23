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
// file-static state — sifive_test FINISHER 解析后的两个 main-readable 字段
// ----------------------------------------------------------------------------
//
// plain int 不 _Atomic — 跨线程可见性靠 SRS/SDS 的 atomic release-acquire 同步:
//   - test_dev_write 内先写 exit_code / reset_req (plain), 然后 atomic_store SRS/SDS
//     (release); main while 读 SRS (acquire) 退出后再读 exit_code / reset_req, plain
//     字段通过 atomic release-acquire 边界跨线程可见 (dummy.txt §7 monitor 模型)
//   - 唯一写入点 = test_dev_write (hart 主线程); 唯一读入点 = main while 之后 (同线程,
//     但跨 atomic 边界); 不存在并发写
//
// 默认 0 (BSS) — PASS exit_code=0 是恒等, FAIL/RESET 路径写入新值.
static int test_dev_exit_code = 0;
static int test_dev_reset_req = 0;


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
           PASS/FAIL 设 SRS=0 + SDS=0 → main while 退 → cleanup → return exit_code.
           RESET 设 SRS=0 + reset_req=1, SDS 不动 (跨 reset 一直跑) → main 判
           reset_req 走 SR continue.
           注意写入顺序: 先 plain 字段 (exit_code/reset_req), 后 atomic SRS/SDS (release) —
           release-acquire 跟 main while 读取建立 happens-before, 让 plain 字段可见. */
        uint32_t cmd = value & 0xFFFFu;
        uint32_t arg = (value >> 16) & 0xFFFFu;
        switch (cmd) {
          case TEST_DEV_FINISHER_PASS:
            test_dev_exit_code = 0;
            atomic_store_explicit(&system_reset_signal, 0, memory_order_release);
            atomic_store_explicit(&shutdown_signal,     0, memory_order_release);
            break;
          case TEST_DEV_FINISHER_FAIL:
            test_dev_exit_code = (int)arg;
            atomic_store_explicit(&system_reset_signal, 0, memory_order_release);
            atomic_store_explicit(&shutdown_signal,     0, memory_order_release);
            break;
          case TEST_DEV_FINISHER_RESET:
            test_dev_reset_req = 1;
            atomic_store_explicit(&system_reset_signal, 0, memory_order_release);
            /* SDS 不动 — RESET 是 warm reset 语义, timer / uart reader thread 跨 reset
               一直跑 (跟真硬件 RTC oscillator + UART pin always-on 一致). */
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
    /* system reset 每 iter (含 RESET cmd 触发的 continue path): exit_code / reset_req 清.
       不持锁 — 跟 plic_reset / uart_reset 内"system reset 时 dispatcher 主帧停, 无并发"
       同前提. */
    test_dev_exit_code = 0;
    test_dev_reset_req = 0;
    return 0;
}

void test_dev_destroy(void) {
    /* 无资源 cleanup; 跟 plic_destroy / clint_destroy 同 void 签名留作 lifecycle 对称 */
}


// ----------------------------------------------------------------------------
// 外部接口: main 端读取 exit_code / consume reset_request
// ----------------------------------------------------------------------------
//
// 调用约束: main 在 while 退出之后 (atomic_load SRS=0 acquire 之后) 调用; 单次读取,
// 无并发. consume_reset 读后清字段 — 避免如果 main while 再进 (RESET continue 之后
// dispatcher 再 tri-fault 退) 把上次 RESET 的残留误当作新 reset.

int test_dev_get_exit_code(void) {
    return test_dev_exit_code;
}

int test_dev_consume_reset_request(void) {
    int r = test_dev_reset_req;
    test_dev_reset_req = 0;
    return r;
}
