//
// UART (ns16550a 兼容) — TX → host stdout / RX ← host stdin; 接 PLIC source 10.
//
// 跟 QEMU virt machine + Linux earlycon=uart8250 + OpenSBI 默认 console 一致;
// 字节访问 (size = 1 only; size != 1 → CAUSE_*_ACCESS_FAULT, 跟 PLIC/test_dev
// 4B 访问区分)。地址布局 / 寄存器形态 见 config.h UART_* 宏。bus 接口形态见
// platform/bus.h + dummy.txt §9。
//
// ----------------------------------------------------------------------------
// monitor 模型 (dummy.txt §7) — UART 是 "monitor + reader + tx_drain 双辅助线程"
// ----------------------------------------------------------------------------
//
// monitor 范式四态:
//   CLINT      = "monitor + timer 辅助线程"            (mtime 按固定步长 fetch_add 推进; host 时钟只定唤醒节奏, 非墙钟值)
//   UART       = "monitor + reader + tx_drain 双线程"   (RX stdin / TX stdout 都异步)
//   virtio-blk = "monitor + io_worker 辅助线程"         (异步 pread/pwrite + 触发 IRQ)
//   PLIC       = "monitor 但无辅助线程"                 (atomic 字段做 hot path 优化)
//   test_dev   = 不是 monitor                          (无内部状态, 纯 fanout + CAS exit_code)
//
// reader 辅助线程职责 (uart_reader_run, 实装在 uart.c):
//   - 周期 poll(STDIN_FILENO, POLLIN, 100ms) — 100ms timeout 用于 cooperative shutdown
//     (CLAUDE.md "Do not pthread_cancel/pthread_kill" 禁; shutdown_signal 检测路径)
//   - 命中 POLLIN → read 1 byte → 加锁 push RX FIFO + 重算 device_line
//   - FIFO 满则 silent 丢字节 (跟 ns16550a overrun 简化, 不真模 LSR.OE)
//   - EOF / shutdown_signal 非 0 → 自然退出
//
// tx_drain 辅助线程职责 (uart_tx_drain_run, 实装在 uart.c):
//   - cond_timedwait(tx_not_empty, UART_TX_DRAIN_INTERVAL_MS=10ms 兜底) cooperative
//     shutdown 模型; hart 写 THR cond_signal 立即 wake (主路径), 10ms 是 SDS check 节奏
//   - wake 后锁内拷整 FIFO 到 stack buf + reset tx_count=0 + 清 acked + 重算
//     device_line (让 ETBEI 中断 fire); 释放锁 → 锁外 write(STDOUT_FILENO, buf, n)
//     一次 batch (跟 _IONBF 每字节 syscall 比 syscall 数量级降)
//   - 高吞吐 (hart 高频写) 自然 batch (write syscall 间隙 hart 灌满 FIFO);
//     低吞吐 (interactive typing) 退化一字节一 write (可预料可接受, 延迟近 0)
//   - shutdown drain 协议: 主循环退出后 (SDS != 0) 进残余 drain 段, 不丢字节
//     (保 [perf] / [main] 末段输出完整 — 跟 virtio_blk worker 丢残余请求语义不同)
//   - uart_join_tx_thread 入口 cond_broadcast 唤醒立即跳出 timeout (不等 10ms)
//
// dummy.txt §12 "谁 spawn 谁 join" 协议适用 — uart_start_rx_thread /
// uart_join_rx_thread + uart_start_tx_thread / uart_join_tx_thread
// 两对接口暴露给 main, 跟 clint_start_timer_thread / clint_join_timer_thread 同
// 形态; 都受 SDS 控制跨 system reset 一直跑。
//
// ----------------------------------------------------------------------------
// 读写抽象 (monitor + pthread_mutex_t)
// ----------------------------------------------------------------------------
//
// UART 内部用 pthread_mutex_t 单锁 (不 rwlock — 跟 PLIC 区分): UART 状态写改占比
// 高 (reader thread push / hart MMIO 读 RBR 改 head / hart MMIO 写改 IER/LCR/...);
// 没有 PLIC csr_mip_read 那种"高频纯读"路径, rwlock 优势退化, mutex 更直白。
//
// file-static helper uart_lock / uart_unlock 包装, 不对外暴露 (调用方都是 uart.c
// 内部函数; putchar 在锁外执行避免 host I/O 在锁内造成的潜在延迟)。
//
// ----------------------------------------------------------------------------
// 字段模型 vs ns16550a spec — 简化点 + 真实装点
// ----------------------------------------------------------------------------
//
// TX 真 FIFO 反压 (UART_FIFO_SIZE 默认 128, ≥ 16550A baseline 16 — 软件按 DTS
// fifo-size batch, 不告知 emulator 容量软件按 16 走 dead capacity 不影响行为):
//   - LSR.THRE = (tx_count < UART_FIFO_SIZE) 真反映 (FIFO 未满)
//   - LSR.TEMT = (tx_count == 0) 真反映 (FIFO 空)
//   - hart 写 THR: 入 tx_fifo ring + cond_signal(tx_not_empty); 满则 silent drop
//     字节 (跟真 16550A FIFO 满写 THR 字节丢一致, 跟 RX silent drop 同体例).
//     drain thread 异步消费 (锁外 write(STDOUT_FILENO, batch)).
//
// RX 真 FIFO (源 = stdin reader thread, 真异步 buffer 需求, 容量同 TX):
//   - LSR.DR = (rx_cnt > 0)
//   - RX trigger level 简化 = 1 (FCR bit 7:6 写入 silent accept; 不真模 1/4/8/14)
//   - 读 RBR pop 一字节; 空时返 0 (无 LSR.OE overrun 真模 — 简化丢字节)
//
// 其他寄存器:
//   - IIR readout 按 spec §10.2 优先级合成 (RX > THRE; bit 7:6=11 报告 FIFO enabled)
//   - FCR 写入 silent accept (trigger level / FIFO reset 简化不真实装)
//   - LCR.DLAB=1 时 off 0/1 路由到 DLL/DLM (silent accept; 不影响 TX/RX)
//   - MCR / MSR (modem 全 0) / SCR (scratch pad) 直接 store/load
//
// device_line 拉法 (同步驱动, 跟 PLIC 套契约):
//   - 任何修改 IER / rx_cnt / tx_count / acked 的路径调完调 uart_compute_device_line_locked
//   - 计算 (ier.ERBFI && rx_cnt > 0) || (ier.ETBEI && tx_count == 0 && !acked)
//   - 跟前值不同时, 调 plic.device_set/clear_pending(UART_PLIC_IRQ)
//   - TX drain thread drain 完 tx_count → 0 后清 acked + compute, 让 ETBEI 中断
//     重新 fire 给 hart (软件 IRQ-driven driver "buffer drained 可续写" 路径)
//
// ----------------------------------------------------------------------------
// host-side stdin raw mode (line-buffered → char-by-char)
// ----------------------------------------------------------------------------
//
// UART 不持有 host stdin termios 状态 — termios 是 process 唯一资源, 归
// runtime 管理 (跟 SDS/SRS / signal handler 同 host-side process 级生命周期;
// 详 runtime.h 顶段 "host-side stdin raw mode" 节). UART 只是 raw 字节流的
// 消费方, 在 reader thread spawn / join 时各调一次 runtime 接口.
//
// 切入点:
//   - uart_start_rx_thread spawn 前 → runtime_stdin_enter_raw()
//   - uart_join_rx_thread  join 后 → runtime_stdin_exit_raw()
//
// 退化路径 (isatty=0 / tcsetattr fail) 由 runtime 端内部 silent 处理, UART
// 不分支 — reader thread 拿到的字节流仍能跑 (只是 line-buffered 体验降级).
//
// ----------------------------------------------------------------------------
// 五函数 lifecycle
// ----------------------------------------------------------------------------
//
// uart_init                — POR 一次性 (main 入口, test_dev_init 之后): 字段 0 init +
//                            pthread_mutex_init + bus 注册; **不发线程** (跟 clint_init
//                            同体例 — spawn 由专门函数). 失败 -1 (dummy.txt §5).
//
// uart_reset               — system reset 每 iter (main while 顶段): 清 8 寄存器字段 +
//                            RX FIFO head/tail/cnt + device_line; lock 不动 (基础设施);
//                            reader_thread 不动 (受 SDS 跨 reset 一直跑, 跟 CLINT timer
//                            同形态; dummy.txt §12).
//
// uart_destroy             — POR 收尾 (main 末段): pthread_mutex_destroy; reader thread
//                            已由 uart_join_rx_thread 收回, 这里只清 mutex.
//
// uart_start_rx_thread     — main POR spawn (clint_start_timer_thread 之后, 进 while
//                                之前); SDS 必须 = 0; spawn fail set DEVICE_FAIL 让 while 不进.
//
// uart_join_rx_thread      — POR 退出段 (main while 外, 跟 clint_join_timer_thread 同
//                                位置): main 已 shutdown_signal_set_bit(NORMAL_EXIT) 之后调;
//                                reader loop 自然退出 → pthread_join 不永远 block.
//
// uart_start_tx_thread   — main POR spawn (uart_start_rx_thread 之后);
//                                体例同 reader (SDS check + spawn fail set DEVICE_FAIL).
//
// uart_join_tx_thread    — POR 退出段 (uart_join_rx_thread 之后); 入口
//                                cond_broadcast(tx_not_empty) 唤醒 drain thread 立即跳出
//                                cond_timedwait (不等 10ms tail timeout) 后再 pthread_join.
//

#ifndef DEVICE_UART_H
#define DEVICE_UART_H

int  uart_init(void);
int  uart_reset(void);
void uart_destroy(void);

void uart_start_rx_thread  (void);
void uart_join_rx_thread   (void);

void uart_start_tx_thread(void);
void uart_join_tx_thread (void);

#endif //DEVICE_UART_H
