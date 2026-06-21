//
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
//   bit 17 = INTERNAL_FATAL    emulator 内部一致性违例 (runtime_fatal; 反查表 -1
//                              等真 emulator bug, 跟 guest 无关); ABORT。独立 bit
//                              (不借道 SHUTDOWN_TRIGGER) 让 SRS join 时一眼看到自己
//                              出了大问题, 详 runtime_fatal 段。
//   bit 18 = HART_SPAWN_FAIL   per-hart pthread_create 失败 (main 端 set); ABORT。
//                              已 spawn 的 hart 看 SRS 退 dispatcher → join 完 →
//                              cleanup chain; 跟 HART_MDT 同 hart-level bit 段。
//   bit 24 = TEST_RESET        0x7777 sifive_test reset; 非 ABORT, main 走
//                              system_reset continue 路径
//   ABORT_MASK = bit0 | bit2 | bit16 | bit17 | bit18 — 命中其中任一 bit 时 main 走
//   cleanup + return exit_code; 不命中 (即 TEST_RESET) 时 main 走 try_clear + continue。
//   未来新增"允许 system reset" 的 bit 放 ABORT_MASK 外即可。
//
// shutdown_signal bit 分配:
//   bit 0  = NORMAL_EXIT       test_dev 0x5555/0x3333 / main 顶层正常退出 (语义:
//                              "由整个模拟器决定退出", 不是设备出问题不得不退)
//   bit 2  = DEVICE_FAIL       受 SDS 控制设备故障 (timer / uart reader /
//                              virtio_blk io_worker spawn fail 或运行中异常)
//   bit 4  = EXTERNAL_SIGNAL   外部 host signal 触发停机 (SIGINT/SIGTERM/SIGHUP);
//                              语义 = "外部介入, 不是模拟器自己决定退出"; 配套
//                              external_signal_no 字段携带 signum, main 末段
//                              按 POSIX 惯例返 128 + signum (130/143/129)
//   bit 6  = INTERNAL_FATAL    emulator 内部一致性违例 (runtime_fatal); 跟 SRS 侧
//                              同名 bit 配套, 走顺序 B (详 runtime_fatal 段)
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
// external signal handler (host SIGINT/SIGTERM/SIGHUP → shutdown):
//   - runtime_install_signal_handlers 一次性安装 sigaction (SIGINT/SIGTERM/SIGHUP
//     都接同一 handler; SA_RESTART; sa_mask 空 handler 重入安全)
//   - handler 内只做两件事 (async-signal-safe; CLAUDE.md 同 SIGSEGV handler 约束:
//     不 fprintf / 不 malloc / 不 pthread_*; 仅 atomic_compare_exchange +
//     shutdown_signal_set_bit, 后者内部纯 atomic_fetch_or 也安全):
//     1) external_signal_no CAS first-wins (expected 0 → store signum), 第二个
//        信号丢弃 signum 但仍走 set_bit (idempotent fetch_or 无害); first-wins
//        保证 "user ^C 后又按 ^C, 仍报 130 而不是被后一个信号改成别的 exit"
//     2) shutdown_signal_set_bit(SHUTDOWN_BIT_EXTERNAL_SIGNAL) 走顺序 B
//        (SDS bit 先, SRS BIT_SHUTDOWN_TRIGGER 后), 让 main while 退出 +
//        SDS-controlled 辅助线程 100ms poll 自然退
//   - SIGKILL / SIGSTOP 不可 catch (POSIX 硬约束); SIGQUIT 不挂 (raw mode 关
//     ISIG 后 ^\ 字节进 guest, 由 guest 处理)
//   - runtime_restore_signal_handlers POR 收尾对偶 (sa_handler=SIG_DFL), 让
//     main 退出后子进程 / 后续 process state 干净
//   - signal 在多线程下默认 deliver 到任意未 mask 的线程; handler 内只动 atomic,
//     不依赖具体线程, 不需要 pthread_sigmask 引导
//
// host-side stdin raw mode (line-buffered → char-by-char):
//   - runtime 持有 host stdin termios 配置 — termios 是 process 唯一资源, 不是
//     UART guest 模型内部状态; 归 runtime 跟 SDS/SRS / signal handler 同质
//     (都是 host process 级生命周期管理)。
//   - runtime_stdin_enter_raw: isatty 检 + tcgetattr 备份 + cfmakeraw +
//     tcsetattr; isatty=0 (pipe/file input) 或 tcsetattr fail 一律退化 cooked
//     (fprintf 一行不 fatal, plan tcsetattr fail 选项 A)
//   - runtime_stdin_exit_raw: if (raw 已 set) tcsetattr restore; restore fail
//     时 fprintf 提示 user `stty sane` 救活
//   - v1 简化: 假设只一个 caller (UART reader thread); 真有第二个 caller 时
//     再升级到 ref count / nested 形态
//   - cfmakeraw 关 ICANON / ECHO / ISIG / IEXTEN / OPOST / INPCK / ICRNL /
//     IXON / ...: 字符即来即收 (不等回车), 不 echo, ^C 字节直接进 read 不发
//     SIGINT (跟上方 external signal handler 正交 — raw mode 下 SIGINT 不会
//     来, handler 是 raw mode 没生效时的 fallback)
//   - abort path (assert / segfault) 不上 atexit restore — atexit handler 内
//     tcsetattr 是 async-signal-unsafe; trade-off 接受, user 需 `stty sane`
//
// 模块 scope: 仅这两 bitmap + signal 字段 + termios state + 接口函数 + bit/mask
// 常量。未来 user 端 reset 按钮等 source 加新 SRS bit 时, 此 header 同步加
// 常量 + ABORT_MASK 评估即可。
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
#define SYSRESET_BIT_INTERNAL_FATAL    (1u << 17)   /* emulator 内部一致性违例 (runtime_fatal); ABORT */
#define SYSRESET_BIT_HART_SPAWN_FAIL   (1u << 18)   /* per-hart pthread_create 失败; main 端 set; ABORT */
#define SYSRESET_BIT_TEST_RESET        (1u << 24)

#define SYSRESET_ABORT_MASK            (SYSRESET_BIT_SHUTDOWN_TRIGGER | \
                                        SYSRESET_BIT_DEVICE_FAIL      | \
                                        SYSRESET_BIT_HART_MDT         | \
                                        SYSRESET_BIT_INTERNAL_FATAL   | \
                                        SYSRESET_BIT_HART_SPAWN_FAIL)

// ----------------------------------------------------------------------------
// shutdown_signal bit 编码
// ----------------------------------------------------------------------------
#define SHUTDOWN_BIT_NORMAL_EXIT       (1u << 0)
#define SHUTDOWN_BIT_DEVICE_FAIL       (1u << 2)
#define SHUTDOWN_BIT_EXTERNAL_SIGNAL   (1u << 4)
#define SHUTDOWN_BIT_INTERNAL_FATAL    (1u << 6)   /* emulator 内部一致性违例 (runtime_fatal) */

extern _Atomic uint32_t system_reset_signal;
extern _Atomic uint32_t shutdown_signal;

// 携带 host signal 触发时的 signum (0 = 未触发; 非 0 = POSIX signum, e.g.
// SIGINT=2 / SIGTERM=15 / SIGHUP=1)。CAS first-wins, 二次信号不覆盖; main 末段
// 按 EXTERNAL_SIGNAL bit 命中读此字段 + 返 128 + signum。详上方 doc。
extern _Atomic uint32_t external_signal_no;

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

// runtime_fatal — emulator 内部一致性违例的紧急全机停机 (唯一的"跨类例外"包装)。
//
// 语义: 当 emulator 内部检测到不该发生的状态 (例如 PLIC 反查表 plic_ctx_to_hartid /
// plic_ctx_map 读到 -1 = 存在的 hart/ctx 丢了硬连线 = 真 emulator bug, 不是 guest
// 的错), 调本函数 → fprintf 诊断 + 全机停机 + main 还能走 cleanup/dump 定位。
//
// **唯一可由 SRS 类设备 (dispatcher/hart/trap) 和 SDS 类设备 (timer/uart/virtio)
// 都直接调用的停机函数** — 这是有意的跨类例外, 不违反 dummy.txt §7 monitor 分层:
//   - 正常生命周期分层 (SRS 类 = 每 system reset 起停 / SDS 类 = 跨 reset 持续) 约束
//     的是"正常运行"; fatal 是"异常退出", 跟"你属于哪条生命周期"正交 — 内部 bug 可能
//     发生在任何模块。
//   - monitor 封装原则贯彻到底: 调用方只调 runtime_fatal, 拿到契约"全机停 + main
//     dump", **不需要也不知道**内部戳了 SDS 还是 SRS (consumer/producer 接口)。
//
// 走 SDS 路径 (顺序 B): set SHUTDOWN_BIT_INTERNAL_FATAL + SYSRESET_BIT_INTERNAL_FATAL。
// SRS/SDS 各用独立 INTERNAL_FATAL bit (不借道 SHUTDOWN_TRIGGER) 的理由: 原理上 SRS
// 会观察 SDS, 但流程是"SRS 先检查自己没问题, 才在清空自己前检查 SDS"。FATAL 的语义是
// 这个错误严重到 SRS 自己就该知道出了大问题, 不能"觉得自己没问题、等到 SDS 语义才停";
// 所以 SRS 侧必须有独立 INTERNAL_FATAL bit, join 时一眼看到自己出了大问题。
//
// 之后再调 wfi_kick_all 紧急唤醒所有可能在 WFI 的 hart (fatal "别睡了立刻停", 不接受
// 正常 shutdown 那 ≤500ms cond_timedwait tail; runtime → core/wfi 方向依赖, 见 runtime.c)。
//
// **non-_Noreturn / 不 abort**: 某些 call site (plic.c recompute_ctx_eip) 持 plic.lock
// wrlock 调入, _Noreturn/abort 会带锁跳走死锁。本函数只 fprintf + set bit + kick + 返回;
// call site 负责在调用后安全 return / 跳过副作用 (防用 -1 二次越界索引)。
//
// where:     caller 函数名 (诊断用; **推荐直接传 `__func__`** 让编译器维护, 防
//            rename 时 hardcode 字符串漏改导致 fatal 日志骗人)。
// detail:    出事的具体上下文 (例 "ctx_to_hartid[ctx_id] < 0"; 跟 where 拼成完整线索)。
// bad_value: 读到的非法值 (例 -1 / 越界 idx)。
void runtime_fatal(const char *where, const char *detail, long bad_value);

// 仅 main 端调: atomic { if shutdown==0 then system_reset=0 } CAS-loop。
// 返 true  = 清成功 (system_reset 已 0; main while 可 continue 进下一 iter);
// 返 false = shutdown 非 0 (main 应走 cleanup chain 退出, 不 continue)。
bool system_reset_signal_try_clear_if_shutdown_zero(void);

// 一次性安装 SIGINT / SIGTERM / SIGHUP handler (sigaction, SA_RESTART); main
// POR 段调一次。详顶段 "external signal handler" 节。失败 -1 / 成功 0。
int  runtime_install_signal_handlers(void);

// POR 收尾对偶: 三个 signal 恢复 SIG_DFL。
void runtime_restore_signal_handlers(void);

// host stdin termios 切 raw mode (line-buffered → char-by-char); caller 是
// 想拿 raw 字节流的模块 (当前 = uart reader thread). 详顶段 "host-side stdin
// raw mode" 节. 内部 silent 退化 (isatty=0 / tcsetattr fail 一律 fprintf 不
// fatal), 不返 status — caller 不分支.
void runtime_stdin_enter_raw(void);

// 对偶: 恢复进入 raw 前 termios. 若没 enter (raw 退化跑了 cooked 路径), 此调
// no-op. fail 时 fprintf 提示 `stty sane` 救活.
void runtime_stdin_exit_raw(void);

#endif //RUNTIME_H
