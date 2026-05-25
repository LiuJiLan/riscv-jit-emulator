//
// Created by liujilan on 2026/5/19.
// runtime 模块 — host-side 生命周期信号 (degenerate monitor; src/dummy.txt §7)。
//
// 两个 atomic uint32_t bitmap, 0 = "允许执行", 非 0 = "触发对应停机路径"。
// (注: 极性跟早期单 bit 形态相反 — 早期 1=允许 / 0=触发, bitmap 化后改 0=允许 /
// 非0=触发, 这样多个事件源 atomic_fetch_or 各自的 bit 互不覆盖。)
//
// system_reset_signal bit 分配:
//   bit 0  = SHUTDOWN_TRIGGER  shutdown_signal 蕴含 system_reset (协议固化在
//                              shutdown_signal_set_bit 内部, 顺序 B); ABORT
//   bit 2  = DEVICE_FAIL       HART 外 SRS-controlled 设备故障 (当前占位, 没真
//                              SRS-controlled 设备); ABORT
//   bit 16 = HART_MDT          HART M-mode double fault (mstatus.MDT 触发, NMI
//                              不实现, 按 QEMU 实践 abort); ABORT
//   bit 24 = TEST_RESET        0x7777 sifive_test reset; 非 ABORT, main 走
//                              system_reset continue 路径
//   ABORT_MASK = bit0 | bit2 | bit16 — 命中其中任一 bit 时 main 走 cleanup +
//   return exit_code; 不命中 (即 TEST_RESET) 时 main 走 try_clear + continue。
//   未来新增"允许 system reset" 的 bit 放 ABORT_MASK 外即可。
//
// shutdown_signal bit 分配:
//   bit 0  = NORMAL_EXIT       test_dev 0x5555/0x3333 / main 顶层正常退出 (语义:
//                              "由整个模拟器决定退出", 不是设备出问题不得不退)
//   bit 2  = DEVICE_FAIL       受 SDS 控制设备故障 (timer / uart reader /
//                              virtio_blk io_worker spawn fail 或运行中异常)
//
// 写路径协议:
//   - set 路径 — atomic_fetch_or RMW, 不允许覆盖别人 bit。HART (M-mode double
//     fault) 只能写 SYSRESET_BIT_HART_MDT (跨 bit 隔离); SDS-controlled 设备只
//     能写 SHUTDOWN_BIT_DEVICE_FAIL; test_dev / main 写 SHUTDOWN_BIT_NORMAL_EXIT
//     和 SYSRESET_BIT_TEST_RESET。
//   - shutdown 蕴含 system_reset — shutdown_signal_set_bit(mask) 内部按 *顺序 B*
//     实现:
//       1) atomic_fetch_or(&shutdown_signal,     mask)
//       2) atomic_fetch_or(&system_reset_signal, SYSRESET_BIT_SHUTDOWN_TRIGGER)
//     反向不要求 (SRS 不必触发 shutdown)。
//   - 顺序 B (SDS 先 SRS 后) 防 race: 若反过来 (SRS 先 SDS 后), main 在 race
//     window 看 SRS != 0 + BIT_SHUTDOWN_TRIGGER 命中 ABORT_MASK → 走 cleanup
//     join, 此时 SDS 还是 0, thread 不退, main hang 在 pthread_join。顺序 B
//     下 SDS 先 set, race window 内 main 还在 dispatcher; window 后 SRS set,
//     main 退 while 时 SDS 已生效, cleanup join 不卡。
//
// 清路径协议:
//   - 仅 main 端 system_reset_signal_try_clear_if_shutdown_zero (CAS-loop); 其他
//     路径不清。语义 = atomic { if shutdown==0 then system_reset=0 }。
//   - 由于 shutdown / system_reset 是两个独立 _Atomic 变量, 跨变量原子做不到, 用
//     CAS-loop 近似: race window 下 shutdown 在两次 load 间从 0 变非 0, CAS 后
//     main 下一 iter 看到 shutdown != 0 自然退出, 行为无害。
//
// degenerate monitor — 字段无跨字段一致性问题, 但 set/clear 路径不直接 extern
// atomic_fetch_or; 通过接口函数包协议 ("SDS 蕴含 SRS" 顺序写死在 set 函数里),
// 跟 dummy.txt §7 monitor 模型一致。
//
// memory_order: set 路径 release (producer 通知); main while load acquire
// (consumer 看到); CAS 路径 release-on-success / acquire-on-fail (标配 RMW)。
//
// 模块 scope: 仅这两 bitmap + 接口函数 + bit/mask 常量。未来 user 端 reset 按钮
// 等 source 加新 SRS bit 时, 此 header 同步加常量 + ABORT_MASK 评估即可。
//

#ifndef RUNTIME_H
#define RUNTIME_H

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

// ----------------------------------------------------------------------------
// system_reset_signal bit 编码
// ----------------------------------------------------------------------------
#define SYSRESET_BIT_SHUTDOWN_TRIGGER  (1u << 0)
#define SYSRESET_BIT_DEVICE_FAIL       (1u << 2)
#define SYSRESET_BIT_HART_MDT          (1u << 16)
#define SYSRESET_BIT_TEST_RESET        (1u << 24)

#define SYSRESET_ABORT_MASK            (SYSRESET_BIT_SHUTDOWN_TRIGGER | \
                                        SYSRESET_BIT_DEVICE_FAIL      | \
                                        SYSRESET_BIT_HART_MDT)

// ----------------------------------------------------------------------------
// shutdown_signal bit 编码
// ----------------------------------------------------------------------------
#define SHUTDOWN_BIT_NORMAL_EXIT       (1u << 0)
#define SHUTDOWN_BIT_DEVICE_FAIL       (1u << 2)

extern _Atomic uint32_t system_reset_signal;
extern _Atomic uint32_t shutdown_signal;

// ----------------------------------------------------------------------------
// 接口函数 (协议固化在内部, caller 只传 bit mask)
// ----------------------------------------------------------------------------

// 设 system_reset_signal bit (atomic_fetch_or release; 不覆盖别人 bit)。
// caller 例:
//   - HART M-mode double fault → system_reset_signal_set_bit(SYSRESET_BIT_HART_MDT);
//   - 0x7777 sifive_test reset → system_reset_signal_set_bit(SYSRESET_BIT_TEST_RESET).
void system_reset_signal_set_bit(uint32_t mask);

// 设 shutdown_signal bit, 内部按顺序 B 同步设 SYSRESET_BIT_SHUTDOWN_TRIGGER
// ("shutdown 蕴含 system_reset" 协议)。caller 例:
//   - test_dev 0x5555/0x3333    → shutdown_signal_set_bit(SHUTDOWN_BIT_NORMAL_EXIT);
//   - timer/uart/virtio_blk fail → shutdown_signal_set_bit(SHUTDOWN_BIT_DEVICE_FAIL).
void shutdown_signal_set_bit(uint32_t mask);

// 仅 main 端调: atomic { if shutdown==0 then system_reset=0 } CAS-loop。
// 返 true  = 清成功 (system_reset 已 0; main while 可 continue 进下一 iter);
// 返 false = shutdown 非 0 (main 应走 cleanup chain 退出, 不 continue)。
bool system_reset_signal_try_clear_if_shutdown_zero(void);

#endif //RUNTIME_H
