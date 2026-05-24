//
// Created by liujilan on 2026/5/24.
// UART (ns16550a 兼容) — TX → host stdout / RX ← host stdin; 接 PLIC source 10.
//
// 跟 QEMU virt machine + Linux earlycon=uart8250 + OpenSBI 默认 console 一致;
// 字节访问 (size = 1 only; size != 1 → CAUSE_*_ACCESS_FAULT, 跟 PLIC/test_dev
// 4B 访问区分)。地址布局 / 寄存器形态 见 config.h UART_* 宏。bus 接口形态见
// platform/bus.h + dummy.txt §9。
//
// ----------------------------------------------------------------------------
// monitor 模型 (dummy.txt §7) — UART 是 "monitor + reader 辅助线程"
// ----------------------------------------------------------------------------
//
// monitor 范式四态:
//   CLINT      = "monitor + timer 辅助线程"     (mtime 由 host wall clock 推进)
//   UART       = "monitor + reader 辅助线程"    (RX 源 = host stdin, blocking read)
//   virtio-blk = "monitor + io_worker 辅助线程" (异步 pread/pwrite + 触发 IRQ)
//   PLIC       = "monitor 但无辅助线程"         (atomic 字段直接做 hot path 优化)
//   test_dev   = 不是 monitor                   (无内部状态, 纯 fanout)
//
// reader 辅助线程职责 (uart_reader_run, 实装在 uart.c):
//   - 周期 poll(STDIN_FILENO, POLLIN, 100ms) — 100ms timeout 用于 cooperative shutdown
//     (CLAUDE.md "Do not pthread_cancel/pthread_kill" 禁; shutdown_signal 检测路径)
//   - 命中 POLLIN → read 1 byte → 加锁 push RX FIFO + 重算 device_line
//   - FIFO 满则 silent 丢字节 (跟 ns16550a overrun 简化, 不真模 LSR.OE)
//   - EOF / shutdown_signal=0 → 自然退出
//
// dummy.txt §12 "谁 spawn 谁 join" 协议适用 — uart_start_reader_thread /
// uart_join_reader_thread 暴露给 main, 跟 clint_start_timer_thread /
// clint_join_timer_thread 同形态; 受 SDS 控制跨 system reset 一直跑。
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
// TX 简化 (host stdout/pipe 已自带 buffer; FIFO 模拟 ROI 低):
//   - LSR.THRE = 1 永远 (THR 永远 empty; 写 THR 直接 putchar)
//   - LSR.TEMT = 1 永远 (shift register 永远 empty; host stdout 瞬间消化)
//   - 写 THR 副作用: 若 IER.ETBEI=1, device_line 仍维持 (THRE 没变); 读 IIR 后才清
//
// RX 真实装 (源 = stdin reader thread, 真异步 buffer 需求):
//   - 16 B RX FIFO (circular buffer, head/tail/cnt)
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
//   - 任何修改 IER 或 rx_cnt 的路径调完调 uart_compute_device_line_locked
//   - 计算 (ier.ERBFI && rx_cnt > 0) || (ier.ETBEI && THRE永远=1)
//   - 跟前值不同时, 调 plic.device_set/clear_pending(UART_PLIC_IRQ)
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
//                            已由 uart_join_reader_thread 收回, 这里只清 mutex.
//
// uart_start_reader_thread — main POR spawn (clint_start_timer_thread 之后, 进 while
//                            之前); SDS 必须已 set 1 (跟 clint_start_timer_thread 同前提).
//                            spawn 失败按 dummy.txt §5 fprintf + set SRS=0 + SDS=0 让
//                            while 不进; 不分 error path, destroy chain 在 while 外写一次.
//
// uart_join_reader_thread  — POR 退出段 (main while 外, 跟 clint_join_timer_thread 同
//                            位置): SDS=0 之后调; reader loop 自然退出 → pthread_join
//                            不永远 block. spawn fail path 防御 (reader_thread_started=0
//                            时 fprintf 一行 skip 不 fatal).
//

#ifndef DEVICE_UART_H
#define DEVICE_UART_H

int  uart_init(void);
int  uart_reset(void);
void uart_destroy(void);

void uart_start_reader_thread(void);
void uart_join_reader_thread (void);

#endif //DEVICE_UART_H
