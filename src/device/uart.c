//
// Created by liujilan on 2026/5/24.
// UART (ns16550a) 实现 — 8 寄存器 byte-access + RX FIFO + reader 辅助线程 +
// bus 注册 + 同步驱动 device_line 拉法。
//
// 接口形态 + monitor 模型 + 字段对应 spec + 5 函数 lifecycle 见 uart.h 顶段 doc。
// 地址布局见 config.h UART_* 宏。报错风格见 dummy.txt §5; "0=成功 / 非0=cause"
// 接口约定见 dummy.txt §9。thread 生命周期 (谁 spawn 谁 join) 见 dummy.txt §12。
//

#define _POSIX_C_SOURCE 200809L   // poll / read / ssize_t

#include "uart.h"

#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "config.h"          // UART_* / UART_PLIC_IRQ / UART_RX_FIFO_CAP
#include "platform/bus.h"    // mmio_dev_t / bus_register_mmio
#include "platform/plic.h"   // device_set_pending / device_clear_pending
#include "riscv.h"           // CAUSE_LOAD/STORE_ACCESS_FAULT
#include "runtime.h"         // shutdown_signal / system_reset_signal


// ----------------------------------------------------------------------------
// UART 内部状态 (单例 file-static; reader thread + dispatcher 主帧跨线程读写)
// ----------------------------------------------------------------------------
//
// 字段 NOT _Atomic — UART 走 pthread_mutex_t (uart_lock / uart_unlock 包装), 锁本身
// 是 happens-before 边界, _Atomic 在锁内冗余 (跟 PLIC 同思路, 区分点是 mutex 不
// rwlock — UART 状态写改占比高, 没有 PLIC csr_mip_read 那种热读路径)。
//
// reader_thread: pthread_t 句柄, BSS 0 init; spawn fail 时 POSIX 不修改 thread 参数,
// pthread_join(0) glibc 下返 ESRCH 容错 (跟 clint.timer_thread 同体例; dummy.txt §12
// + clint.h 顶段)。

#define UART_IER_ERBFI   0x01u    /* bit 0: enable RX data available interrupt */
#define UART_IER_ETBEI   0x02u    /* bit 1: enable THR empty interrupt */
#define UART_IER_MASK    0x0Fu    /* IER 低 4 bit 有效 */

#define UART_LCR_DLAB    0x80u    /* bit 7: Divisor Latch Access */

#define UART_LSR_DR      0x01u    /* RX data ready (随 rx_cnt) */
#define UART_LSR_THRE    0x20u    /* THR empty (永远 1) */
#define UART_LSR_TEMT    0x40u    /* shift reg empty (永远 1) */
#define UART_LSR_BASE    (UART_LSR_THRE | UART_LSR_TEMT)   /* = 0x60 */

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

    /* RX FIFO 16 B circular buffer (reader thread push / hart MMIO RBR read pop) */
    uint8_t  rx_fifo[UART_RX_FIFO_CAP];
    uint8_t  rx_head;
    uint8_t  rx_tail;
    uint8_t  rx_cnt;

    /* device_line cache — 同步驱动: 任何改 ier / rx_cnt 的路径后调
       uart_compute_device_line_locked, 跟前值不同时调 plic.device_set/clear_pending */
    uint8_t  device_line;

    /* RX 中断"读 IIR 不清"语义 vs TX 中断"读 IIR 清"语义的实现 — IIR readout 副作用 */
    uint8_t  thr_empty_pending_acked;  /* 0=THR empty 中断未被读 IIR 确认; 1=已 ack 不再 fire */

    pthread_t        reader_thread;
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
// 计算 = (ier.ERBFI && rx_cnt > 0) || (ier.ETBEI && !thr_empty_pending_acked)
//
// 跟前值不同时, 调 plic.device_set_pending / device_clear_pending(UART_PLIC_IRQ).
// plic.device_set/clear_pending 内部有自己的 rwlock, 锁嵌套顺序 = uart.lock → plic.lock
// (单向, 不会跟 plic 内调 uart 反向 — plic 不知道 uart 存在)。
//
// 注: thr_empty_pending_acked 字段是为 "读 IIR 清 THR empty 中断" 语义服务 — ns16550a
// spec §10.2: 读 IIR 时, 如果当前 report 是 THR empty (bit 3:1 = 001), 该中断
// 被认为 "acknowledged", 下次 readout 不再 report, 直到再次写 THR 后才重启 (写 THR
// 时清 acked 标记)。RX data available 中断没有这层 "读 IIR 清", 只能由 pop RBR 清。

static void uart_compute_device_line_locked(void) {
    uint8_t rx_int  = (uart.ier & UART_IER_ERBFI) && (uart.rx_cnt > 0);
    uint8_t thr_int = (uart.ier & UART_IER_ETBEI) && !uart.thr_empty_pending_acked;
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
    } else if ((uart.ier & UART_IER_ETBEI) && !uart.thr_empty_pending_acked) {
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
                uart.rx_tail = (uart.rx_tail + 1u) % UART_RX_FIFO_CAP;
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
        v = UART_LSR_BASE | (uart.rx_cnt > 0 ? UART_LSR_DR : 0u);
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

    /* THR write 特例 — putchar 必须在锁外执行 (host I/O 在锁内会延阻其他 hart MMIO
       访问; stdout 自身 thread-safe 不需 uart.lock 保护). 把字段更新跟 putchar 拆开. */
    if (off == UART_REG_RBR_THR) {
        uart_lock();
        if (uart.lcr & UART_LCR_DLAB) {
            uart.dll = v;
            uart_unlock();
        } else {
            /* 写 THR 重启 THR empty 中断 fire (清 acked); 副作用: device_line 重算 */
            uart.thr_empty_pending_acked = 0;
            uart_compute_device_line_locked();
            uart_unlock();
            /* putchar 锁外; stdout 已 _IONBF 全局 (main setvbuf), 不需 fflush */
            putchar((int)v);
        }
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
// shutdown_signal=0 + 短 poll timeout 是合规路径)。
//
// 命中 POLLIN → read 1 byte → 加锁 push RX FIFO + 重算 device_line. FIFO 满则 silent
// 丢字节 (不真模 ns16550a overrun LSR.OE; 留 long-term TODO).
//
// EOF (read==0): 自然退出. 已 push 到 FIFO 的字节仍可被 hart pop, 不主动清.
// 错误处理: EINTR retry; 其他 errno → fprintf + break (跟 clint timer_run 同体例).
// trace: 不打 char-stream trace (dummy.txt §7 末段; 真要调试用临时 fprintf).

static void *uart_reader_run(void *arg) {
    (void)arg;

    while (atomic_load_explicit(&shutdown_signal, memory_order_acquire)) {
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
        if (uart.rx_cnt < UART_RX_FIFO_CAP) {
            uart.rx_fifo[uart.rx_head] = c;
            uart.rx_head = (uart.rx_head + 1u) % UART_RX_FIFO_CAP;
            uart.rx_cnt++;
            uart_compute_device_line_locked();  /* rx_cnt 变 → 可能 set RX 中断 */
        }
        /* FIFO 满: silent drop; 真模 LSR.OE 留 long-term TODO */
        uart_unlock();
    }
    return NULL;
}


// ----------------------------------------------------------------------------
// lifecycle: uart_init / uart_reset / uart_destroy
// ----------------------------------------------------------------------------

int uart_init(void) {
    /* 字段 0 init — BSS 已 0, 显式 memset lifecycle 可读 (跟 plic_init / clint_init
       同体例). 留意: 这一处会清 reader_thread 句柄, 必须在 uart_start_reader_thread
       之前调 (init 在 main POR 段, spawn 在 main POR 后段, 顺序天然正确). */
    memset(&uart, 0, sizeof(uart));

    int rc = pthread_mutex_init(&uart.lock, NULL);
    if (rc != 0) {
        fprintf(stderr, "uart_init: pthread_mutex_init failed: %s\n", strerror(rc));
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
        (void)pthread_mutex_destroy(&uart.lock);
        return -1;
    }
    return 0;
}

int uart_reset(void) {
    /* system reset 每 iter: 8 寄存器字段 + RX FIFO + device_line 清; 锁 + reader
       thread 不动 (基础设施 / 受 SDS 跨 reset 一直跑, 跟 CLINT timer 同形态).
       device_line 通过 compute helper 清 (因 ier=0 后计算结果必为 0; 顺便通知
       plic.device_clear_pending 让 PLIC 端 device_line 也清). */
    uart_lock();
    uart.dll = uart.dlm = 0;
    uart.ier = uart.fcr = uart.lcr = uart.mcr = uart.scr = 0;
    memset(uart.rx_fifo, 0, sizeof(uart.rx_fifo));
    uart.rx_head = uart.rx_tail = uart.rx_cnt = 0;
    uart.thr_empty_pending_acked = 0;
    uart_compute_device_line_locked();
    uart_unlock();
    return 0;
}

void uart_destroy(void) {
    /* reader thread 已由 main uart_join_reader_thread 收回; 这里只清 mutex.
       跟 plic_destroy / clint_destroy 同 lifecycle 对称. */
    (void)pthread_mutex_destroy(&uart.lock);
}


// ----------------------------------------------------------------------------
// thread lifecycle: uart_start_reader_thread / uart_join_reader_thread (dummy.txt §12)
// ----------------------------------------------------------------------------
//
// 跟 clint_start_timer_thread / clint_join_timer_thread 同体例:
//   - spawn 前 check SDS (前面有别的 init 失败已 set SDS=0 时 skip spawn, 给"统一
//     走 runtime signal 通道" 留路; 当前 main flow 下 SDS 必 1, check 是死代码)
//   - spawn fail: fprintf + set SRS=0 + SDS=0 (一对两 flag) 让 main while 不进
//   - reader_thread 字段不 track started flag — BSS 0 init, POSIX 7.2 pthread_create
//     fail 不修改 thread 参数; pthread_join(0) glibc 下返 ESRCH 容错, fprintf 一行
//     不 fatal (跟 clint.timer_thread 字段同形态)

void uart_start_reader_thread(void) {
    if (atomic_load_explicit(&shutdown_signal, memory_order_acquire) == 0) {
        return;
    }

    int rc = pthread_create(&uart.reader_thread, NULL, uart_reader_run, NULL);
    if (rc != 0) {
        fprintf(stderr, "uart_start_reader_thread: pthread_create failed: %s\n",
                strerror(rc));
        atomic_store_explicit(&system_reset_signal, 0, memory_order_release);
        atomic_store_explicit(&shutdown_signal,     0, memory_order_release);
    }
}

void uart_join_reader_thread(void) {
    /* 调用前置: main 已 atomic_store(&shutdown_signal, 0); reader loop 自然退出.
       spawn fail case: uart.reader_thread 保持 BSS 0, pthread_join 返 ESRCH, fprintf
       一行不 fatal (跟 clint_join_timer_thread 同形态). */
    int rc = pthread_join(uart.reader_thread, NULL);
    if (rc != 0) {
        fprintf(stderr, "uart_join_reader_thread: pthread_join failed: %s\n",
                strerror(rc));
    }
}
