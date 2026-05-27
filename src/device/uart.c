//
// Created by liujilan on 2026/5/24.
// UART (ns16550a) 实现 — 8 寄存器 byte-access + RX FIFO + reader 辅助线程 +
// bus 注册 + 同步驱动 device_line 拉法。
//
// 接口形态 + monitor 模型 + 字段对应 spec + 5 函数 lifecycle 见 uart.h 顶段 doc。
// 地址布局见 config.h UART_* 宏。报错风格见 dummy.txt §5; "0=成功 / 非0=cause"
// 接口约定见 dummy.txt §9。thread 生命周期 (谁 spawn 谁 join) 见 dummy.txt §12。
//

#define _POSIX_C_SOURCE 200809L   // poll / read / ssize_t / clock_gettime / write

#include "uart.h"

#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "config.h"          // UART_* / UART_PLIC_IRQ / UART_FIFO_SIZE / UART_TX_DRAIN_INTERVAL_MS
#include "platform/bus.h"    // mmio_dev_t / bus_register_mmio
#include "platform/plic.h"   // device_set_pending / device_clear_pending
#include "riscv.h"           // CAUSE_LOAD/STORE_ACCESS_FAULT
#include "runtime.h"         // shutdown_signal / system_reset_signal

// ns16550a baseline FIFO 容量 (软件 ns16550a driver 默认按这个 batch). UART_FIFO_SIZE
// 实际容量必须 >= 这个值, 不然 emulator 表现比 16550A 还小, 软件 batch 16 字节就会
// 撞反压 (跟 spec 不符). _Static_assert C11, 项目目标 C11.
#define UART_16550A_FIFO_SIZE  16U
_Static_assert(UART_FIFO_SIZE >= UART_16550A_FIFO_SIZE,
               "UART_FIFO_SIZE must be >= 16550A baseline (16 bytes)");


// ----------------------------------------------------------------------------
// UART 内部状态 (单例 file-static; reader / tx_drain thread + dispatcher 主帧跨线程读写)
// ----------------------------------------------------------------------------
//
// 字段 NOT _Atomic — UART 走 pthread_mutex_t (uart_lock / uart_unlock 包装), 锁本身
// 是 happens-before 边界, _Atomic 在锁内冗余 (跟 PLIC 同思路, 区分点是 mutex 不
// rwlock — UART 状态写改占比高, 没有 PLIC csr_mip_read 那种热读路径)。
//
// reader_thread / tx_drain_thread: pthread_t 句柄, BSS 0 init; spawn fail 时 POSIX 不
// 修改 thread 参数, pthread_join(0) glibc 下返 ESRCH 容错 (跟 clint.timer_thread 同
// 体例; dummy.txt §12 + clint.h 顶段)。
//
// tx_not_empty: pthread_cond_t, hart 入 TX FIFO 后 cond_signal, drain thread 立即
// wake 处理; drain thread 在 cond_timedwait 10ms 兜底循环里 (高吞吐场景下 hart
// 高频写 + drain write syscall 间隙 = batch 自然形成; 低吞吐场景退化一字节一
// write, 可接受). uart_join_tx_thread 入口走 cond_broadcast 唤醒退出.

#define UART_IER_ERBFI   0x01u    /* bit 0: enable RX data available interrupt */
#define UART_IER_ETBEI   0x02u    /* bit 1: enable THR empty interrupt */
#define UART_IER_MASK    0x0Fu    /* IER 低 4 bit 有效 */

#define UART_LCR_DLAB    0x80u    /* bit 7: Divisor Latch Access */

#define UART_LSR_DR      0x01u    /* RX data ready (随 rx_cnt > 0) */
#define UART_LSR_THRE    0x20u    /* THR empty — 真反映 (tx_count < UART_FIFO_SIZE) */
#define UART_LSR_TEMT    0x40u    /* shift reg empty — 真反映 (tx_count == 0); 软件可读这个判"全部物理发送完成" */

#define UART_IIR_NO_INT  0x01u    /* bit 0: 1 = no interrupt pending */
#define UART_IIR_RX_DATA 0x04u    /* bit 3:1 = 010 (RX data available) */
#define UART_IIR_THR_EMP 0x02u    /* bit 3:1 = 001 (THR empty) */
#define UART_IIR_FIFO_ON 0xC0u    /* bit 7:6 = 11 (FIFO enabled) */

static struct {
    /* 8 寄存器物理字段; LSR/IIR 是 readout 时合成不是字段 */
    uint8_t  dll, dlm;          /* baud divisor (silent accept; 不影响行为) */
    uint8_t  ier;               /* interrupt enable (bit 0 ERBFI / bit 1 ETBEI) */
    uint8_t  fcr;               /* 软件最后写入的 FCR; trigger level 简化不读 */
    uint8_t  lcr;               /* LCR (bit 7 = DLAB) */
    uint8_t  mcr;
    uint8_t  scr;               /* scratch pad; 无副作用 */

    /* RX FIFO circular buffer (reader thread push / hart MMIO RBR read pop).
       容量 UART_FIFO_SIZE (默认 128 > 16550A baseline 16; 软件按 DTS fifosize batch,
       超过 baseline 时是 dead capacity 不影响行为). index 用 uint16_t (UART_FIFO_SIZE
       128 = 2^7, uint8_t 装得下但 head==SIZE 边界用 uint16_t 更安全). */
    uint8_t  rx_fifo[UART_FIFO_SIZE];
    uint16_t rx_head;
    uint16_t rx_tail;
    uint16_t rx_cnt;

    /* TX FIFO circular buffer (hart MMIO THR write push / tx_drain thread pop → host
       stdout). 容量同 RX (单 UART_FIFO_SIZE 宏 RX/TX 共用); 满则 silent drop 字节
       (跟 RX silent drop 同体例, 跟真 16550A FIFO 满写 THR 字节丢一致). LSR.THRE
       真反映 (tx_count < UART_FIFO_SIZE); LSR.TEMT 真反映 (tx_count == 0). */
    uint8_t  tx_fifo[UART_FIFO_SIZE];
    uint16_t tx_head;
    uint16_t tx_tail;
    uint16_t tx_count;

    /* device_line cache — 同步驱动: 任何改 ier / rx_cnt / tx_count / acked 的路径
       后调 uart_compute_device_line_locked, 跟前值不同时调 plic.device_set/
       clear_pending */
    uint8_t  device_line;

    /* RX 中断"读 IIR 不清"语义 vs TX 中断"读 IIR 清"语义的实现 — IIR readout 副作用.
       drain thread drain 完 tx_count→0 时也清 acked, 让 ETBEI 中断 fire (queue 真空了, hart
       该收到"buffer drained 可以续写"中断). */
    uint8_t  thr_empty_pending_acked;  /* 0=THR empty 中断未被读 IIR 确认; 1=已 ack 不再 fire */

    pthread_t        reader_thread;
    pthread_t        tx_drain_thread;
    pthread_cond_t   tx_not_empty;     /* hart 入 queue → cond_signal; drain thread cond_timedwait */
    pthread_mutex_t  lock;
} uart;


// ----------------------------------------------------------------------------
// monitor 锁 helper (file-static; 跟 plic_rdlock/wrlock 同体例但 mutex 形态)
// ----------------------------------------------------------------------------

static void uart_lock  (void) { (void)pthread_mutex_lock  (&uart.lock); }
static void uart_unlock(void) { (void)pthread_mutex_unlock(&uart.lock); }


// ----------------------------------------------------------------------------
// device_line 计算 helper (file-static; "_locked" 后缀; 调用方持锁)
// ----------------------------------------------------------------------------
//
// 计算 = (ier.ERBFI && rx_cnt > 0) ||
//        (ier.ETBEI && tx_count == 0 && !thr_empty_pending_acked)
//
// 跟前值不同时, 调 plic.device_set_pending / device_clear_pending(UART_PLIC_IRQ).
// plic.device_set/clear_pending 内部有自己的 rwlock, 锁嵌套顺序 = uart.lock → plic.lock
// (单向, 不会跟 plic 内调 uart 反向 — plic 不知道 uart 存在)。
//
// 注 1: thr_empty_pending_acked 字段为 "读 IIR 清 THR empty 中断" 语义服务 — ns16550a
// spec §10.2: 读 IIR 时, 如果当前 report 是 THR empty (bit 3:1 = 001), 该中断
// 被认为 "acknowledged", 下次 readout 不再 report, 直到再次写 THR 后才重启 (写 THR
// 时清 acked 标记)。RX data available 中断没有这层 "读 IIR 清", 只能由 pop RBR 清。
//
// 注 2: thr_int 增加 (tx_count == 0) 条件是真 TX FIFO 反压模型的必然 — 真硬件
// THR empty 中断只在 FIFO 真空时 fire (跟"TX 永远 empty" 假装模型不同). drain
// thread drain 完 queue 后 (锁内 reset tx_count + 清 acked) 再调本函数, 触发
// plic.device_set_pending 让 hart 收到中断, 续写新 batch.

static void uart_compute_device_line_locked(void) {
    uint8_t rx_int  = (uart.ier & UART_IER_ERBFI) && (uart.rx_cnt > 0);
    uint8_t thr_int = (uart.ier & UART_IER_ETBEI) &&
                      (uart.tx_count == 0) &&
                      !uart.thr_empty_pending_acked;
    uint8_t want    = (rx_int || thr_int) ? 1u : 0u;

    if (want == uart.device_line) return;
    uart.device_line = want;
    if (want) device_set_pending  ((uint32_t)UART_PLIC_IRQ);
    else      device_clear_pending((uint32_t)UART_PLIC_IRQ);
}


// ----------------------------------------------------------------------------
// IIR readout helper (file-static; "_locked"; 调用方持锁)
// ----------------------------------------------------------------------------
//
// ns16550a IIR 优先级 (spec §10.2):
//   - RX data available  (bit 3:1 = 010): 当 ier.ERBFI && rx_cnt > 0; 优先级最高;
//     不被读 IIR 清 — 只能由 pop RBR (rx_cnt 减到 0) 清
//   - THR empty          (bit 3:1 = 001): 当 ier.ETBEI && THRE=1; 优先级次之;
//     被读 IIR 清 (set thr_empty_pending_acked=1; 下次 readout 不再 report)
//   - 无 pending         (bit 0 = 1):     无上述
//
// bit 7:6 永远报告 11 (FIFO enabled; 我们直接当 16550 mode 而不是 8250 legacy)
//
// 副作用: 调用 (即读 IIR 寄存器) 时, 若当前 report 是 THR empty, 拉低 acked 字段;
// 调用方读取返回值后, 必须调 uart_compute_device_line_locked 让 device_line 跟着变.

static uint8_t uart_iir_readout_locked(void) {
    uint8_t bits;
    if ((uart.ier & UART_IER_ERBFI) && uart.rx_cnt > 0) {
        bits = UART_IIR_RX_DATA;
    } else if ((uart.ier & UART_IER_ETBEI) &&
               (uart.tx_count == 0) &&
               !uart.thr_empty_pending_acked) {
        bits = UART_IIR_THR_EMP;
        /* 副作用: ack 掉 THR empty 中断, 下次 readout 不再 report */
        uart.thr_empty_pending_acked = 1;
    } else {
        bits = UART_IIR_NO_INT;
    }
    return UART_IIR_FIFO_ON | bits;
}


// ----------------------------------------------------------------------------
// uart_read / uart_write — bus 派发入口 (size=1 only)
// ----------------------------------------------------------------------------
//
// 接口语义 (dummy.txt §9):
//   返 0    = 成功; read 路径返寄存器值 / write 路径修改字段
//   返非 0  = cause (RV spec exception code); bus 透传给 trap_raise_exception
//
// fault 政策 (跟 ns16550a 标准 byte-access 一致):
//   - size != 1 → CAUSE_*_ACCESS_FAULT (8 寄存器都 1B 字段; 半字 / 字访问非合法)
//   - off >= 8 (region 内 reserved [8, UART_SIZE)) → silent ignore (R 返 0 / W 丢弃)
//   - off < 8 reserved (无) — 8 寄存器全部有定义形态
//
// off 0/1 路由: LCR.DLAB=1 时 → DLL/DLM (silent baud divisor accept, 不影响行为);
//               DLAB=0 时 → RBR(R) / THR(W) (RBR pop FIFO; THR putchar 即时)。

static int uart_read(void *ctx, uint32_t off, void *buf, uint32_t size) {
    (void)ctx;
    if (size != 1u) return CAUSE_LOAD_ACCESS_FAULT;
    if (off >= 8u) {
        *(uint8_t *)buf = 0;
        return 0;
    }

    uint8_t v = 0;
    uart_lock();
    switch (off) {
      case UART_REG_RBR_THR:
        if (uart.lcr & UART_LCR_DLAB) {
            v = uart.dll;
        } else {
            /* RBR: pop RX FIFO; 空则返 0 (不真模 LSR.OE overrun) */
            if (uart.rx_cnt > 0) {
                v = uart.rx_fifo[uart.rx_tail];
                uart.rx_tail = (uint16_t)((uart.rx_tail + 1u) % UART_FIFO_SIZE);
                uart.rx_cnt--;
                uart_compute_device_line_locked();  /* rx_cnt 变 → 可能清 RX 中断 */
            }
        }
        break;

      case UART_REG_IER:
        v = (uart.lcr & UART_LCR_DLAB) ? uart.dlm : uart.ier;
        break;

      case UART_REG_IIR_FCR:
        v = uart_iir_readout_locked();              /* 副作用: ack THR empty */
        uart_compute_device_line_locked();          /* ack 后 device_line 重算 */
        break;

      case UART_REG_LCR: v = uart.lcr; break;
      case UART_REG_MCR: v = uart.mcr; break;

      case UART_REG_LSR:
        v = (uart.rx_cnt > 0                ? UART_LSR_DR   : 0u) |
            (uart.tx_count < UART_FIFO_SIZE ? UART_LSR_THRE : 0u) |
            (uart.tx_count == 0             ? UART_LSR_TEMT : 0u);
        break;

      case UART_REG_MSR: v = 0; break;              /* modem 全 0; CTS/DSR/RI/DCD 未连 */
      case UART_REG_SCR: v = uart.scr; break;
    }
    uart_unlock();

    *(uint8_t *)buf = v;
    return 0;
}

static int uart_write(void *ctx, uint32_t off, const void *buf, uint32_t size) {
    (void)ctx;
    if (size != 1u) return CAUSE_STORE_ACCESS_FAULT;
    if (off >= 8u)  return 0;                       /* reserved silent ignore */

    uint8_t v = *(const uint8_t *)buf;

    /* THR write — 字节入 TX FIFO (drain thread 异步消费). FIFO 满则 silent drop
       (跟 RX silent drop 同体例, 跟真 16550A FIFO 满写 THR 字节丢一致). 真硬件
       polled driver 会先读 LSR.THRE 再写, 不会盲写; 自家 demo 或 fixture 写法
       不查 LSR 时, 满字节丢. cond_signal 仅在真入队时调省一次系统调用. */
    if (off == UART_REG_RBR_THR) {
        uart_lock();
        if (uart.lcr & UART_LCR_DLAB) {
            uart.dll = v;
        } else {
            if (uart.tx_count < UART_FIFO_SIZE) {
                uart.tx_fifo[uart.tx_head] = v;
                uart.tx_head = (uint16_t)((uart.tx_head + 1u) % UART_FIFO_SIZE);
                uart.tx_count++;
                pthread_cond_signal(&uart.tx_not_empty);
            }
            /* 写 THR 清 acked 让下次 queue 空时 THR empty 中断重新 fire (即使本次
               丢字节也清 — hart 是否丢看 LSR.THRE, IIR ack 状态机独立). */
            uart.thr_empty_pending_acked = 0;
            uart_compute_device_line_locked();
        }
        uart_unlock();
        return 0;
    }

    uart_lock();
    switch (off) {
      case UART_REG_IER:
        if (uart.lcr & UART_LCR_DLAB) {
            uart.dlm = v;
        } else {
            uart.ier = v & UART_IER_MASK;           /* 低 4 bit 有效; 高位丢 */
            /* IER 写后 THR empty 中断重置 (8250 spec: ETBEI 从 0 → 1 时立即 fire);
               简单做法 — 任何 IER 写都清 acked, 让 device_line 按当前 IER+THRE/RX 重算. */
            uart.thr_empty_pending_acked = 0;
            uart_compute_device_line_locked();
        }
        break;

      case UART_REG_IIR_FCR:
        /* FCR 写: trigger level (bit 7:6) / FIFO reset (bit 1:2) silent accept; 不真实装.
           bit 0 (FIFO enable) 也 silent accept — 我们永远以 16550 FIFO mode 运作.
           简化: 只保存字段方便软件回读 (虽然 FCR is W-only, 但保存无害). */
        uart.fcr = v;
        break;

      case UART_REG_LCR: uart.lcr = v; break;
      case UART_REG_MCR: uart.mcr = v; break;

      case UART_REG_LSR: /* RO; silent ignore */ break;
      case UART_REG_MSR: /* RO; silent ignore */ break;

      case UART_REG_SCR: uart.scr = v; break;
    }
    uart_unlock();
    return 0;
}


// ----------------------------------------------------------------------------
// reader 辅助线程 — RX 源 = stdin
// ----------------------------------------------------------------------------
//
// 周期 poll(STDIN, POLLIN, 100ms): 100 ms timeout 用于 cooperative shutdown 检测
// (CLAUDE.md "Do not suggest pthread_cancel/pthread_kill" — 否决积极取消,
// shutdown_signal 非 0 + 短 poll timeout 是合规路径)。
//
// 命中 POLLIN → read 1 byte → 加锁 push RX FIFO + 重算 device_line. FIFO 满则 silent
// 丢字节 (不真模 ns16550a overrun LSR.OE; 留 long-term TODO).
//
// EOF (read==0): 自然退出. 已 push 到 FIFO 的字节仍可被 hart pop, 不主动清.
// 错误处理: EINTR retry; 其他 errno → fprintf + break (跟 clint timer_run 同体例).
// trace: 不打 char-stream trace (dummy.txt §7 末段; 真要调试用临时 fprintf).

static void *uart_reader_run(void *arg) {
    (void)arg;

    while (atomic_load_explicit(&shutdown_signal, memory_order_acquire) == 0u) {
        struct pollfd pfd = { .fd = STDIN_FILENO, .events = POLLIN, .revents = 0 };
        int rc = poll(&pfd, 1, 100);            /* 100 ms timeout cooperative shutdown */
        if (rc < 0) {
            if (errno == EINTR) continue;
            fprintf(stderr, "[uart reader] poll failed: %s\n", strerror(errno));
            break;
        }
        if (rc == 0) continue;                  /* timeout — 再判 SDS */
        if (!(pfd.revents & POLLIN)) continue;

        uint8_t c;
        ssize_t n = read(STDIN_FILENO, &c, 1);
        if (n == 0) break;                      /* EOF; stdin closed / pipe 跑完 */
        if (n < 0) {
            if (errno == EINTR) continue;
            fprintf(stderr, "[uart reader] read failed: %s\n", strerror(errno));
            break;
        }

        uart_lock();
        if (uart.rx_cnt < UART_FIFO_SIZE) {
            uart.rx_fifo[uart.rx_head] = c;
            uart.rx_head = (uint16_t)((uart.rx_head + 1u) % UART_FIFO_SIZE);
            uart.rx_cnt++;
            uart_compute_device_line_locked();  /* rx_cnt 变 → 可能 set RX 中断 */
        }
        /* FIFO 满: silent drop; 真模 LSR.OE 留 long-term TODO */
        uart_unlock();
    }
    return NULL;
}


// ----------------------------------------------------------------------------
// tx_drain 辅助线程 — TX FIFO → host stdout
// ----------------------------------------------------------------------------
//
// 异步 TX 模型: hart 写 THR (uart_write) → 入 tx_fifo + cond_signal(tx_not_empty);
// drain thread cond_timedwait(UART_TX_DRAIN_INTERVAL_MS) 兜底 + signal 立即 wake.
// 每次 wake 后, 锁内拷整 fifo 到 stack buf + reset 三字段 + 清 acked + 重算
// device_line (让 ETBEI 中断 fire); 锁外一次 write(STDOUT_FILENO, buf, n) batch.
//
// 节流模型 (跟 virtio_blk io_worker 同体例 — 选项 B):
//   - 高吞吐 (hart 高频写, e.g. bad apple play 60KB/帧): drain thread write syscall
//     间隙里 hart 持续灌 FIFO, 一次 batch 写出大块字节; write syscall 时间决定
//     batch 大小. syscall 数远低于 _IONBF.
//   - 低吞吐 (interactive typing): 退化成一字节一 write (可预料可接受;
//     interactive 延迟近 0).
//   - cond_timedwait 10ms 兜底主要为 SDS check (signal 才是主路径).
//
// shutdown drain 协议 (跟 virtio_blk worker 丢残余请求不同):
//   - 外层 while (SDS == 0) 退出后, 进残余 drain 段, 不丢字节 — 保 [perf] /
//     [main] 末段输出完整.
//   - uart_join_tx_thread 入口先 cond_broadcast 唤醒 drain thread (不等
//     10ms timeout); join 后才走 destroy chain.
//
// 真硬件对应: 真 ns16550a FIFO 满时 hart 写 THR 字节丢 (uart_write 那里已实现);
// drain thread 类比物理 wire (按 baud rate 串行移出, 我们 emulator 不节流跑最快).

static void *uart_tx_drain_run(void *arg) {
    (void)arg;

    /* batch 临时 buffer (锁外 write 用; 单线程不需 lock) */
    uint8_t batch[UART_FIFO_SIZE];

    while (atomic_load_explicit(&shutdown_signal, memory_order_acquire) == 0u) {
        uart_lock();
        /* 等 fifo 非空或 SDS 触发; cond_timedwait 兜底周期 cooperative shutdown */
        while (uart.tx_count == 0 &&
               atomic_load_explicit(&shutdown_signal, memory_order_acquire) == 0u) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_nsec += (long)(UART_TX_DRAIN_INTERVAL_MS * 1000000L);
            if (ts.tv_nsec >= 1000000000L) {
                ts.tv_sec  += ts.tv_nsec / 1000000000L;
                ts.tv_nsec %= 1000000000L;
            }
            int rc = pthread_cond_timedwait(&uart.tx_not_empty, &uart.lock, &ts);
            if (rc == ETIMEDOUT) continue;       /* 重检 tx_count + SDS */
            if (rc == 0) continue;               /* signaled 或 spurious; 重检 */
            /* 真异常 (rc != 0 且 != ETIMEDOUT) — 走 cleanup */
            fprintf(stderr, "[uart tx_drain] cond_timedwait failed: %s\n",
                    strerror(rc));
            uart_unlock();
            shutdown_signal_set_bit(SHUTDOWN_BIT_DEVICE_FAIL);
            return NULL;
        }

        /* 退内层 — 要么 tx_count > 0 要么 SDS 触发. 两种情况都进 drain 路径
           (SDS 触发时 tx_count 可能 0, 拷 0 字节也 OK; 之后外层 while 退出
           走残余 drain 段时是真正剩余字节路径). */
        size_t n = uart.tx_count;
        for (size_t i = 0; i < n; i++) {
            batch[i] = uart.tx_fifo[uart.tx_tail];
            uart.tx_tail = (uint16_t)((uart.tx_tail + 1u) % UART_FIFO_SIZE);
        }
        uart.tx_count = 0;
        /* drain 完 tx_count→0 时清 acked, 让 THR empty 中断重新 fire 给 hart
           (queue 真空了, 软件 IRQ-driven driver 该收到 "buffer drained 可以续写"). */
        uart.thr_empty_pending_acked = 0;
        uart_compute_device_line_locked();
        uart_unlock();

        /* 锁外 write — write syscall 时间是 batch 形成的关键 (期间 hart 可继续
           灌 FIFO, drain thread 顶 while 再次拿锁时 tx_count 可能又非 0). */
        if (n > 0) {
            size_t written = 0;
            while (written < n) {
                ssize_t w = write(STDOUT_FILENO, batch + written, n - written);
                if (w < 0) {
                    if (errno == EINTR) continue;
                    fprintf(stderr, "[uart tx_drain] write failed: %s\n",
                            strerror(errno));
                    shutdown_signal_set_bit(SHUTDOWN_BIT_DEVICE_FAIL);
                    return NULL;
                }
                if (w == 0) {
                    /* POSIX: write 0 写 0 字节正常返回, 但持续 0 算异常 — short
                       write 形态. stdout 一般不撞, 防御性走 fail 路径. */
                    fprintf(stderr, "[uart tx_drain] write returned 0 (short write)\n");
                    shutdown_signal_set_bit(SHUTDOWN_BIT_DEVICE_FAIL);
                    return NULL;
                }
                written += (size_t)w;
            }
        }
    }

    /* 退出主循环后 — shutdown 路径残余 drain. 不丢字节 (保 [perf] / [main] 末段
       输出完整, 跟 virtio_blk worker 丢残余请求语义不同 — 字节流不可丢).
       此时 main 已 set SDS, hart 不再写新字节进 fifo (cleanup 顺序: shutdown_signal_
       set_bit → join 顺序 timer → reader → tx_drain → virtio_blk; reader 退出后
       hart 也不再跑, 但 hart 可能在 set SDS 那瞬间还有最后几个 THR 写未 drain). */
    uart_lock();
    if (uart.tx_count > 0) {
        size_t n = uart.tx_count;
        for (size_t i = 0; i < n; i++) {
            batch[i] = uart.tx_fifo[uart.tx_tail];
            uart.tx_tail = (uint16_t)((uart.tx_tail + 1u) % UART_FIFO_SIZE);
        }
        uart.tx_count = 0;
        uart.thr_empty_pending_acked = 0;
        uart_compute_device_line_locked();
        uart_unlock();

        size_t written = 0;
        while (written < n) {
            ssize_t w = write(STDOUT_FILENO, batch + written, n - written);
            if (w < 0) {
                if (errno == EINTR) continue;
                fprintf(stderr, "[uart tx_drain] tail-drain write failed: %s\n",
                        strerror(errno));
                /* shutdown 路径已在, 不再 set DEVICE_FAIL — 仅 log */
                break;
            }
            if (w == 0) break;
            written += (size_t)w;
        }
    } else {
        uart_unlock();
    }

    return NULL;
}


// ----------------------------------------------------------------------------
// lifecycle: uart_init / uart_reset / uart_destroy
// ----------------------------------------------------------------------------

int uart_init(void) {
    /* 字段 0 init — BSS 已 0, 显式 memset lifecycle 可读 (跟 plic_init / clint_init
       同体例). 留意: 这一处会清 reader_thread / tx_drain_thread 句柄, 必须在两个
       spawn 之前调 (init 在 main POR 段, spawn 在 main POR 后段, 顺序天然正确). */
    memset(&uart, 0, sizeof(uart));

    int rc = pthread_mutex_init(&uart.lock, NULL);
    if (rc != 0) {
        fprintf(stderr, "uart_init: pthread_mutex_init failed: %s\n", strerror(rc));
        return -1;
    }

    rc = pthread_cond_init(&uart.tx_not_empty, NULL);
    if (rc != 0) {
        fprintf(stderr, "uart_init: pthread_cond_init failed: %s\n", strerror(rc));
        (void)pthread_mutex_destroy(&uart.lock);
        return -1;
    }

    mmio_dev_t dev = {
        .gpa_start = (uint32_t)UART_BASE,
        .gpa_end   = (uint32_t)(UART_BASE + UART_SIZE),
        .ctx       = &uart,                 /* 单例 device; fn 内 (void)ctx; */
        .read      = uart_read,
        .write     = uart_write,
        .name      = "uart",
    };
    if (bus_register_mmio(&dev) != 0) {
        fprintf(stderr, "uart_init: bus_register_mmio failed\n");
        (void)pthread_cond_destroy(&uart.tx_not_empty);
        (void)pthread_mutex_destroy(&uart.lock);
        return -1;
    }
    return 0;
}

int uart_reset(void) {
    /* system reset 每 iter: 8 寄存器字段 + RX/TX FIFO + device_line 清; 锁 + reader /
       tx_drain thread 不动 (基础设施 / 受 SDS 跨 reset 一直跑, 跟 CLINT timer 同形态).
       device_line 通过 compute helper 清 (因 ier=0 后计算结果必为 0; 顺便通知
       plic.device_clear_pending 让 PLIC 端 device_line 也清).

       TX FIFO 清: drain thread 此刻可能在 (a) cond_timedwait 锁外等 — main 直接拿锁
       清字段, drain wake 后见 tx_count=0 继续等, 安全; (b) 锁内拷 buf — main 等锁,
       drain 拷完释放后 main 拿锁清, drain 锁外 write 已拷 buf 内容, write 完成无新
       字节, safe; (c) 锁外 write — main 拿锁清, drain write 完顶 while 进锁见
       tx_count=0 wait, safe. 已入 queue 但还没 write 出去的字节会丢, 跟 system reset
       语义一致 (跟 shutdown 路径"必须保字节" 不同, 见 uart_tx_drain_run 末段). */
    uart_lock();
    uart.dll = uart.dlm = 0;
    uart.ier = uart.fcr = uart.lcr = uart.mcr = uart.scr = 0;
    memset(uart.rx_fifo, 0, sizeof(uart.rx_fifo));
    uart.rx_head = uart.rx_tail = uart.rx_cnt = 0;
    memset(uart.tx_fifo, 0, sizeof(uart.tx_fifo));
    uart.tx_head = uart.tx_tail = uart.tx_count = 0;
    uart.thr_empty_pending_acked = 0;
    uart_compute_device_line_locked();
    uart_unlock();
    return 0;
}

void uart_destroy(void) {
    /* reader thread + tx_drain thread 已由 main uart_join_*_thread 收回; 这里只清
       cond + mutex. 跟 plic_destroy / clint_destroy 同 lifecycle 对称. */
    (void)pthread_cond_destroy(&uart.tx_not_empty);
    (void)pthread_mutex_destroy(&uart.lock);
}


// ----------------------------------------------------------------------------
// thread lifecycle: uart_start_rx_thread / uart_join_rx_thread (dummy.txt §12)
// ----------------------------------------------------------------------------
//
// 跟 clint_start_timer_thread / clint_join_timer_thread 同体例:
//   - spawn 前 check SDS (前面有别的 init 失败已 set SDS=0 时 skip spawn, 给"统一
//     走 runtime signal 通道" 留路; 当前 main flow 下 SDS 必 1, check 是死代码)
//   - spawn fail: fprintf + set SRS=0 + SDS=0 (一对两 flag) 让 main while 不进
//   - reader_thread 字段不 track started flag — BSS 0 init, POSIX 7.2 pthread_create
//     fail 不修改 thread 参数; pthread_join(0) glibc 下返 ESRCH 容错, fprintf 一行
//     不 fatal (跟 clint.timer_thread 字段同形态)

void uart_start_rx_thread(void) {
    if (atomic_load_explicit(&shutdown_signal, memory_order_acquire) != 0u) {
        return;
    }

    /* host stdin raw mode 委托 runtime — termios 是 process 唯一资源, 归
       runtime 管 (跟 SDS/SRS / signal handler 同 host-side process 级生命周期).
       runtime_stdin_enter_raw 内 silent 退化 (isatty=0 / tcsetattr fail 一律
       fprintf 不 fatal), 不 propagate 给 reader thread 的 spawn fail 路径. */
    runtime_stdin_enter_raw();

    int rc = pthread_create(&uart.reader_thread, NULL, uart_reader_run, NULL);
    if (rc != 0) {
        fprintf(stderr, "uart_start_rx_thread: pthread_create failed: %s\n",
                strerror(rc));
        shutdown_signal_set_bit(SHUTDOWN_BIT_DEVICE_FAIL);
    }
}

void uart_join_rx_thread(void) {
    /* 调用前置: main 已 shutdown_signal_set_bit(NORMAL_EXIT); reader loop 自然退出.
       spawn fail case: uart.reader_thread 保持 BSS 0, pthread_join 返 ESRCH, fprintf
       一行不 fatal (跟 clint_join_timer_thread 同形态). */
    int rc = pthread_join(uart.reader_thread, NULL);
    if (rc != 0) {
        fprintf(stderr, "uart_join_rx_thread: pthread_join failed: %s\n",
                strerror(rc));
    }

    /* host stdin termios restore 委托 runtime (跟 spawn 前 enter_raw 对偶).
       enter 退化路径 (isatty=0 / tcsetattr fail) 内部 tio_saved=0, exit 自然 no-op. */
    runtime_stdin_exit_raw();
}


// ----------------------------------------------------------------------------
// thread lifecycle: uart_start_tx_thread / uart_join_tx_thread (dummy.txt §12)
// ----------------------------------------------------------------------------
//
// 跟 reader thread 同 spawn/join 体例; 区别:
//   - 异常路径调 shutdown_signal_set_bit(DEVICE_FAIL) — 走 runtime 顺序 B 协议
//     (SDS bit fetch_or + 蕴含 SRS BIT_SHUTDOWN_TRIGGER 一并设).
//   - join 入口先 cond_broadcast 唤醒可能在 cond_timedwait 内的 drain thread,
//     不等 10ms timeout (broadcast 对 zero waiters 是 no-op, 不需持锁).

void uart_start_tx_thread(void) {
    if (atomic_load_explicit(&shutdown_signal, memory_order_acquire) != 0u) {
        return;
    }

    int rc = pthread_create(&uart.tx_drain_thread, NULL, uart_tx_drain_run, NULL);
    if (rc != 0) {
        fprintf(stderr, "uart_start_tx_thread: pthread_create failed: %s\n",
                strerror(rc));
        shutdown_signal_set_bit(SHUTDOWN_BIT_DEVICE_FAIL);
    }
}

void uart_join_tx_thread(void) {
    /* 唤醒可能在 cond_timedwait 内的 drain thread, 跳过 10ms tail timeout. 持锁
       与否对 broadcast 行为无影响 (POSIX 不要求持锁 broadcast); 不持锁省一次
       acquire/release pair. */
    (void)pthread_cond_broadcast(&uart.tx_not_empty);

    int rc = pthread_join(uart.tx_drain_thread, NULL);
    if (rc != 0) {
        fprintf(stderr, "uart_join_tx_thread: pthread_join failed: %s\n",
                strerror(rc));
    }
}
