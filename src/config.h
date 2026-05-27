//
// Created by liujilan on 2026/4/29.
// 全局编译期宏。
//
// 本文件只放编译期常量(#define)。
// 运行期变量(host_ram_base / host_ram_base_adjusted / 未来其他 ram 派生量)由 ram_init 初始化, 暴露在 ram.h —— 不放这里。
//
//

#ifndef CONFIG_H
#define CONFIG_H

// guest 物理地址空间内, RAM 区域的起点 GPA。
// 0x80000000 为 RISC-V常用的内存起点
#define GUEST_RAM_START   0x80000000UL

// guest RAM 大小(字节)。128MB。
// page on demand, 名义大不会占物理;
// 留足空间,后期不用回头改。
#define GUEST_RAM_SIZE    (128UL * 1024 * 1024)

// ----------------------------------------------------------------------------
// TLB / ASID 配置
// ----------------------------------------------------------------------------
//
// TLB_ASID_BITS = 4 → ASIDLEN = 4。Sv32 规范: ASIDMAX ≤ 9, 我们选 4 < 9, 合法。
// guest 看到的 ASIDLEN 就是 4 位; csr.c 的 satp 写 helper 必须做 WARL 截断, 详见
// dummy.txt §3 (satp 合法性契约)。
#define TLB_ASID_BITS     4U
#define ASID_MAX          (1U << TLB_ASID_BITS)         /* = 16 */
#define ASID_MASK         (ASID_MAX - 1U)               /* = 0xF, fast path 用 */

// 单套叶 TLB 的 entry 数 (direct-mapped index)。
// 64 entry × 16 B/entry = 1 KB / 套, cache line 友好。
// set-associative 是改进项, 现阶段 direct-mapped 足够。
#define TLB_INDEX_BITS    6U
#define TLB_NUM_ENTRIES   (1U << TLB_INDEX_BITS)        /* = 64 */

// ----------------------------------------------------------------------------
// IALIGN (指令地址对齐)
// ----------------------------------------------------------------------------
//
// RV spec: IALIGN 是 hart 级常量(单位:位), 由是否实现 C / Zca 扩展决定:
//   - 实现 C/Zca → IALIGN = 16(所有 PC 必须 2 字节对齐)
//   - 不实现 C   → IALIGN = 32(所有 PC 必须 4 字节对齐)
// misa.C 决定 IALIGN; 多数实现把 misa.C 写死, IALIGN 是编译期常量。
// 本项目: misa.C 强制为 1(decode 已含 RVC 路径), IALIGN 固定 16。
// 动态切 misa 是远期议题, 真要做时再回来动这里 + 加运行时切换路径。
//
// 实际效果: jal / branch (imm[0]=0 编码强制) + jalr (& ~1u 强制 mask LSB) 路径下,
// (target & IALIGN_MASK) 永远 = 0, instruction-address-misaligned 异常 (cause 0)
// 在本项目中是 dead code; 但代码里仍写检查 + 调 trap_raise_exception 占位, 保结构
// 完整, 与 spec 语义对齐, 也方便日后切 IALIGN=32 时只改宏不改逻辑。
#define IALIGN            16U
#define IALIGN_BYTES      (IALIGN / 8U)        /* = 2 */
#define IALIGN_MASK       (IALIGN_BYTES - 1U)  /* = 1; (target & IALIGN_MASK) != 0 → 不对齐 */

// ----------------------------------------------------------------------------
// 软边界: 单 block 最大指令数 (interpreter / 未来 translator 共用)
// ----------------------------------------------------------------------------
//
// plan §1.23.2 软边界初版默认 64;[32, 128] 区间都合理, 性能数据出来后再调。
// interpreter / translator 各自循环维护自己的计数器, 共享同一个上限常量。
// 当前用途:fixture 写错时 (例如忘 ecall 收尾, 或 branch 死循环) 失控保护;
// 未来 OS 场景下还要叠加跨 4K page 检查 + 真边界 op (硬边界由
// is_block_boundary_inst 处理), 这两条是另两个独立的 block 截断条件。
#define BLOCK_INST_LIMIT  64U

// ----------------------------------------------------------------------------
// hart 数量
// ----------------------------------------------------------------------------
//
// v1 单 hart; SMP 真接时改这个宏, clint.msip[] / mtimecmps[] 等数组自动跟随。
// 不放运行期变量 (跟 GUEST_RAM_* 同性质 — 编译期已知, 数组形态稳定)。
#define MAX_HARTS         1U

// ----------------------------------------------------------------------------
// CLINT (Core-Local Interruptor) MMIO 地址布局
// ----------------------------------------------------------------------------
//
// 跟 QEMU virt machine + SiFive E31/U54 Core Complex Manual 一致 (不跟 ACLINT spec)。
//
//   +0x0000   msip[hart]      4 byte/hart   M-mode software interrupt (仅低 1 位有效)
//   +0x4000   mtimecmp[hart]  8 byte/hart   per-hart timer compare value
//   +0xBFF8   mtime           8 byte global timer counter
#define CLINT_BASE          0x02000000UL
#define CLINT_SIZE          0x00010000UL
#define CLINT_MSIP_OFF      0x0000UL
#define CLINT_MTIMECMP_OFF  0x4000UL
#define CLINT_MTIME_OFF     0xBFF8UL

// ----------------------------------------------------------------------------
// TIMEBASE / timer 辅助线程参数 (timer 辅助线程异步累加 atomic clint.mtime)
// ----------------------------------------------------------------------------
//
// 10 MHz 跟 QEMU virt machine (guest dtb timebase-frequency 复用); 1ms 是 Linux
// nanosleep 精度甜点 + 跟 HZ=1000 / FreeRTOS 默认 tick 匹配。RV mtime 跟传统 wall
// clock RTC 是两个东西 (mtime = 高频 monotonic counter, 重启清零; wall clock 走
// SBI / virtio-rtc, 本项目当前不涉及)。
#define TIMEBASE_FREQ_HZ        10000000UL   /* guest 视角: mtime tick 频率 (10 MHz, 100ns/tick) */
#define TIMER_WAKE_INTERVAL_NS  1000000UL    /* host 实现: timer 辅助线程 nanosleep 周期 (1 ms) */

// WFI cond_timedwait 兜底周期 (015; core/wfi.c). 防 wfi_kick 漏调时永睡 + 兜底
// SRS 传播 (signal handler 路径不能 kick, 走 SRS+timeout 自愈)。500ms 是 wfi 私有
// 数 (比 UART_TX_DRAIN_INTERVAL_MS=10ms 长得多 — wfi 真在睡, 不需要 IO 那种快响应);
// kick 工作正常时基本不触发, 误漏时人感知前自愈。
#define WFI_TIMEOUT_NS          500000000UL  /* 500 ms */

// 一次唤醒 fetch_add 量 (编译期常量; 当前配置 = 10000)。
// TIMEBASE_FREQ_HZ * TIMER_WAKE_INTERVAL_NS / 1e9, 解耦 guest 视角频率 vs host
// 调度周期: guest 看到 mtime tick 单位 (100ns) 跟 host nanosleep 周期 (1ms)
// 互不绑定, 改其一不影响其二语义。
#define TIMEBASE_PER_WAKE       (TIMEBASE_FREQ_HZ * TIMER_WAKE_INTERVAL_NS / 1000000000ULL)

// ----------------------------------------------------------------------------
// PLIC (Platform-Level Interrupt Controller) MMIO 地址布局
// ----------------------------------------------------------------------------
//
// 跟 QEMU virt machine + SiFive PLIC 一致 (RV PLIC spec v1.0.0 兼容)。
//
//   +0x000000   priority[N]     4 byte/source       per-source priority
//   +0x001000   pending[]       1 bit/source        per-source pending bit
//                               (32 sources / 4-byte word; source 0 = word0.bit0 保留)
//   +0x002000   enable[ctx][]   1 bit/source/ctx    per-ctx per-source enable
//                               (0x80 = 128 B/ctx, 32 src/word, 32 word/ctx → 1024 src max)
//   +0x200000   ctx[ctx]        0x1000 B/ctx        per-ctx control:
//                                                     +0x000 threshold
//                                                     +0x004 claim / complete
//
// source 0 永远保留 (RV PLIC spec: source_id 0 = no IRQ, 用 dtb interrupts 引用时
// 跳过)。N_SOURCES 跟 QEMU virt 默认对齐 (96, 含 source 0; 实际可用 1..N_SOURCES-1)。
//
// N_CONTEXTS: 当前每 hart 2 context (M + S), 跟 dtb interrupts-extended 标准约定一致;
// MU-only / MSU 等 misa-aware 动态 context 划法是 long-term TODO, v1 简化。
#define PLIC_BASE            0x0C000000UL
#define PLIC_SIZE            0x00600000UL              /* 6 MB; QEMU virt 标准 */
#define PLIC_PRIORITY_OFF    0x000000UL
#define PLIC_PENDING_OFF     0x001000UL
#define PLIC_ENABLE_OFF      0x002000UL
#define PLIC_ENABLE_STRIDE   0x80UL                    /* 128 B/ctx (1024 src max) */
#define PLIC_CONTEXT_OFF     0x200000UL
#define PLIC_CONTEXT_STRIDE  0x1000UL                  /* 4 KB/ctx; +0 threshold, +4 claim/complete */
#define PLIC_N_SOURCES       96U                       /* 跟 QEMU virt 默认对齐; src 0 保留 */
#define PLIC_N_CONTEXTS      (MAX_HARTS * 2U)          /* M + S 双 context per hart; v1 = 2 */

// ----------------------------------------------------------------------------
// test_dev (sifive_test 兼容外设) MMIO 地址布局
// ----------------------------------------------------------------------------
//
// 跟 QEMU virt machine + sifive_test 同 (base = 0x00100000, 4 KB region)。
//
//   +0x00      sifive_test FINISHER         (W: 三 magic 解析 PASS/FAIL/RESET; R 返 0)
//                                            cmd = value & 0xFFFF; arg = (value >> 16) & 0xFFFF
//                                            cmd=0x5555 PASS  → main return 0
//                                            cmd=0x3333 FAIL  → main return arg
//                                            cmd=0x7777 RESET → main while continue (跨 reset
//                                                              timer/uart reader thread 跑)
//   +0x40      TEST_DEV_SET                 (W value=source_id → device_set_pending)
//   +0x44      TEST_DEV_CLEAR               (W value=source_id → device_clear_pending)
//   其他 off   silent ignore                (R 返 0 / W 丢弃, 跟 plic 内 reserved 体例)
//
// fixture 端通过 sw TEST_DEV_SET_OFF / TEST_DEV_CLEAR_OFF 触发 PLIC fanout; sw
// TEST_DEV_SIFIVE_OFF 上报 exit code / 触发 system reset.
#define TEST_DEV_BASE             0x00100000UL
#define TEST_DEV_SIZE             0x00001000UL          /* 4 KB */
#define TEST_DEV_SIFIVE_OFF       0x00UL                /* finisher: PASS/FAIL/RESET 三 magic */
#define TEST_DEV_SET_OFF          0x40UL                /* W → device_set_pending(value) */
#define TEST_DEV_CLEAR_OFF        0x44UL                /* W → device_clear_pending(value) */

/* sifive_test FINISHER cmd magic (low 16 bit of write value); QEMU virt sifive_test 兼容 */
#define TEST_DEV_FINISHER_PASS    0x5555U
#define TEST_DEV_FINISHER_FAIL    0x3333U
#define TEST_DEV_FINISHER_RESET   0x7777U

// ----------------------------------------------------------------------------
// UART (ns16550a 兼容) MMIO 地址布局
// ----------------------------------------------------------------------------
//
// 跟 QEMU virt machine + Linux earlycon=uart8250 + OpenSBI 默认 console 一致
// (base 0x10000000, 8 寄存器 byte-access, size = 0x100 留 reg-shift 扩展空间)。
//
//   +0x0  RBR (R) / THR (W) / DLL (DLAB=1)
//   +0x1  IER       / DLM (DLAB=1)
//   +0x2  IIR (R)   / FCR (W)
//   +0x3  LCR  (含 DLAB bit 7)
//   +0x4  MCR
//   +0x5  LSR  (R)
//   +0x6  MSR  (R)
//   +0x7  SCR
//
// 访问宽度: 1 byte only (8250 spec; size != 1 → CAUSE_*_ACCESS_FAULT)。
// PLIC source_id: 跟 QEMU virt 一致 (UART0 = 10)。
//
// FIFO 容量: UART_FIFO_SIZE 单宏 RX/TX 共用 (默认 128, 超 16550A baseline 16);
// 寄存器接口不变 (软件按 fifosize 走, ns16550 标准 DTS binding 只 fifo-size 一个
// 属性, 不分 RX/TX). 软件不通过 DTS 显式 fifo-size 时按默认 16 处理, emulator
// 内部容量更大只是 dead capacity (不影响行为); 软件配 DTS fifo-size=128 才用满.
// uart.c 加 _Static_assert(UART_FIFO_SIZE >= 16) 兜底 16550A 兼容性.
//
// TX drain thread 兜底周期: cond_timedwait wake interval, 主要为 SDS check 节奏
// (hart 入 queue cond_signal 立即 wake 是主路径). 10 ms 足够 (interactive 字节
// 最大延迟人不可感; 高吞吐场景下 hart 高频写 → write syscall 间隙 batch 自然).
#define UART_BASE                 0x10000000UL
#define UART_SIZE                 0x00000100UL     /* 256 B 留 reg-shift 扩展 */
#define UART_REG_RBR_THR          0x0UL            /* RBR(R) / THR(W) / DLL(DLAB=1) */
#define UART_REG_IER              0x1UL            /* IER     / DLM(DLAB=1) */
#define UART_REG_IIR_FCR          0x2UL            /* IIR(R)  / FCR(W) */
#define UART_REG_LCR              0x3UL            /* LCR (bit 7 = DLAB) */
#define UART_REG_MCR              0x4UL
#define UART_REG_LSR              0x5UL            /* LSR (R; THRE/TEMT/DR) */
#define UART_REG_MSR              0x6UL            /* MSR (R) */
#define UART_REG_SCR              0x7UL
#define UART_PLIC_IRQ             10U              /* 跟 QEMU virt UART0 一致 */
#define UART_FIFO_SIZE            128U             /* RX/TX 共用容量; ≥ 16550A baseline 16 */
#define UART_TX_DRAIN_INTERVAL_MS 10U              /* drain thread cond_timedwait 兜底周期 (SDS check) */

// ----------------------------------------------------------------------------
// virtio-mmio block device (legacy v1.0 + DeviceID=2)
// ----------------------------------------------------------------------------
//
// 跟 QEMU virt machine virtio-mmio.0 对齐 (base 0x10001000 紧邻 UART, IRQ 1)。
// legacy v1.0 寄存器布局 (Version=1, 单 QueuePFN 单 queue_align); modern v1.1
// 三 PFN 形态属未来工作。
//
// 访问宽度: 4 byte only (含 Config space; legacy 推荐 4B align; size != 4 →
// CAUSE_*_ACCESS_FAULT, 跟 PLIC/test_dev 同形态)。
//
// 后端 = host file (pread/pwrite + image_fd; mmap/fsync 都属未来工作);
// IO 路径 = 异步 worker thread + work queue (hart 写 QueueNotify 入队即返;
// io_worker_run drain avail ring + 真做 pread/pwrite + 写 used ring + 触发
// IRQ)。异步默认体例 (hart fast path 不阻塞) 见 trade_off_log §T.7。
#define VIRTIO_BLK_BASE             0x10001000UL
#define VIRTIO_BLK_SIZE             0x00001000UL    /* 4 KB MMIO (含 Config space @ +0x100) */
#define VIRTIO_BLK_PLIC_IRQ         1U              /* QEMU virt virtio-mmio.0 惯例 */
#define VIRTIO_BLK_WORK_QUEUE_CAP   8U              /* hart → worker 入队 ring 容量 */
#define VIRTIO_BLK_QUEUE_NUM_MAX    8U              /* legacy QueueNumMax (≤ work queue cap) */
#define VIRTIO_BLK_SECTOR_SIZE      512U            /* 标准 sector 大小 */

#endif //CONFIG_H
