//
// Created by liujilan on 2026/5/24.
// test_dev — sifive_test 兼容外设, 跟 QEMU virt sifive_test 同地址 (0x00100000)。
//
// 职责: 三个 MMIO 入口:
//   - TEST_DEV_SIFIVE_OFF (off 0x00) — sifive_test FINISHER; W 解析三 magic (PASS=
//     0x5555 / FAIL=0x3333 / RESET=0x7777), cmd = value & 0xFFFF, arg = (value >> 16)。
//     PASS/FAIL → shutdown_signal_set_bit(SHUTDOWN_BIT_NORMAL_EXIT) (内部按顺序 B
//     蕴含 SRS BIT_SHUTDOWN_TRIGGER; ABORT_MASK 命中, main 走 cleanup + return
//     exit_code)。
//     RESET → system_reset_signal_set_bit(SYSRESET_BIT_TEST_RESET) (非 ABORT, main
//     走 try_clear continue, timer / uart reader 跨 reset 一直跑)。
//   - TEST_DEV_SET_OFF / TEST_DEV_CLEAR_OFF (off 0x40 / 0x44) — fixture 通过 sw
//     触发 device_set/clear_pending fanout, 走 PLIC 完整路径。
//
// 地址布局 / 寄存器形态 见 config.h TEST_DEV_* 宏 (含 TEST_DEV_FINISHER_PASS /
// FAIL / RESET 三 magic 宏)。bus 接口形态 (read/write 返 cause / 0=成功) 见
// platform/bus.h + dummy.txt §9。
//
// ----------------------------------------------------------------------------
// monitor 模型 (dummy.txt §7) — test_dev 不是 monitor (退化情况)
// ----------------------------------------------------------------------------
//
// CLINT    = "monitor + timer 辅助线程"
// PLIC     = "monitor 但无线程"
// UART     = "monitor + reader 辅助线程"
// test_dev = **退化 monitor** — exit_code 只由 hart 主线程通过 MMIO 写; main 读取
//            走 SRS/SDS atomic happens-before 边界 (release-acquire 同步, plain int
//            跨线程仍可见); 不持锁; SET/CLEAR fanout 透传 plic 内 rwlock。
//            (reset_req 字段 + consume_reset_request 入口已废除 — reset 触发改用
//            SRS bit SYSRESET_BIT_TEST_RESET 表达, main 直接读 SRS bit。)
//
// dummy.txt §12 "谁 spawn 谁 join" 协议**不适用** (无线程); §7 monitor 范式只用
// happens-before 边界 (SRS/SDS atomic) 而非完整 monitor 锁。
//
// ----------------------------------------------------------------------------
// 三函数 lifecycle (跟 plic / clint 同形态; reset/destroy 真工作量 = 0, 留作 lifecycle
// 对称)
// ----------------------------------------------------------------------------
//
// test_dev_init    — POR 一次性 (main 入口, plic_init 之后): 填 mmio_dev_t + 调
//                    bus_register_mmio; 失败 -1 (跟 plic_init 同形态; dummy.txt §5)。
//                    顺序 plic_init 之后 — test_dev 写 SET/CLEAR 直接调 plic.device_set/
//                    clear_pending, plic 必须先 ready (rwlock_init + plic_ctx_map 已填)。
//
// test_dev_reset   — system reset 每 iter (main while 顶段): no-op (无内部状态); 返 0
//                    保持 lifecycle 函数签名对称 (跟 plic_reset / clint_reset 同签名)。
//
// test_dev_destroy — POR 收尾 (main 末段): no-op (无内部状态; bus 未来加 unregister
//                    时这里调); 函数留作 lifecycle 对称 (跟 plic_destroy / clint_destroy
//                    同 void 签名)。
//

#ifndef DEVICE_TEST_DEV_H
#define DEVICE_TEST_DEV_H

int  test_dev_init(void);
int  test_dev_reset(void);
void test_dev_destroy(void);

/* main 端读取入口 — 返 exit code (PASS=0 / FAIL=arg / 默认 0).
   main return 时用; SRS/SDS atomic release-acquire 同步, 跟 main while 退出顺序对齐. */
int  test_dev_get_exit_code(void);

#endif //DEVICE_TEST_DEV_H
