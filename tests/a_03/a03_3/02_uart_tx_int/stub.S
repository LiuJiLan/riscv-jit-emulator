# a03_3 / 02_uart_tx_int
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# UART TX 中断 PLIC 联调 — 验 UART 内部状态机 → device_set_pending(10) → PLIC 仲裁
# → MEIP 合成 → handler claim/ack/complete 全链路:
#   1. 主帧 sb THR 输出 "OK\n" (IER.ETBEI=0 不拉 IRQ; 走 LSR.THRE=1 永远满足 fast path)
#   2. 主帧 sb IER = ETBEI (启 TX empty 中断) → uart_compute_device_line 算出
#      (ETBEI && !thr_empty_pending_acked) = true → device_set_pending(10)
#   3. PLIC 仲裁 ctx0 (= hart0 M ctx) → is_plic_meip_pending(0)=1 → csr_mip_read
#      合成 MEIP (bit 11) → trap_check_interrupt fire
#   4. m_handler:
#        - 验 mcause = 0x8000000B (INT_BIT | IRQ_M_EXT=11)
#        - lw claim → 期望 = 10
#        - lb UART IIR → 副作用 ack THR empty (thr_empty_pending_acked=1 →
#          uart_compute_device_line 拉低 → device_clear_pending(10))
#        - sw complete = 10
#        - mret
#   5. 主帧 spin wait_loop → s0=1 → 跳出 → test_dev FINISHER 干净停机
#
# ===== 思路 =====
# UART 8 寄存器 byte-access, sb/lb 走 size=1 path; 立即数 0..7 都在 RV I-type imm 12-bit
# signed 范围, 直接 sb t1, OFF(t0) 无需 lui.
#
# init 顺序保护 "trigger 前完成 IRQ-clean output":
#   1. mtvec / mie / mstatus 配 trap 接收能力
#   2. PLIC 配 source 10 路径
#   3. UART LCR=0x03 (8N1, DLAB=0); 后续 sb IER 操作 IER 而不是 DLM
#   4. sb 'O' 'K' '\n' (IER.ETBEI=0, 不 fire)
#   5. sb IER ETBEI (此处 fire)
#
# 注: 写 IER 同时 thr_empty_pending_acked 清 (uart_write 内 IER 副作用); 这是为了
# 让 ETBEI 从 0→1 沿立即触发 IRQ — 跟 ns16550a "level interrupt" 语义实际行为一致.

.equ PLIC_PRIO_BASE,    0x0C000000
.equ PLIC_ENABLE_BASE,  0x0C002000
.equ PLIC_THRESHOLD,    0x0C200000
.equ PLIC_CLAIM,        0x0C200004
.equ UART_BASE,         0x10000000

.equ UART_REG_THR,      0
.equ UART_REG_IER,      1
.equ UART_REG_IIR,      2
.equ UART_REG_LCR,      3

.section .text
.option norvc
.globl _start

_start:
    # ---- M init: mtvec / mie.MEIP / mstatus.MIE ----
    la      t0, m_handler
    csrw    mtvec, t0
    li      t0, 0x800               # mie.MEIP = bit 11
    csrw    mie, t0
    csrw    0x310, x0               # mstatush 清 MDT (A.7 prelude; 否则下条 MIE 写被强制 0)
    csrrsi  x0, mstatus, 0x8        # mstatus.MIE = bit 3

    # ---- 配 PLIC: priority[10]=1 ----
    li      t0, PLIC_PRIO_BASE
    li      t1, 1
    sw      t1, 40(t0)              # priority[10] @ 4*10 = 40

    # ---- 配 PLIC: threshold[ctx0]=0 ----
    li      t0, PLIC_THRESHOLD
    sw      x0, 0(t0)

    # ---- 配 PLIC: enable[ctx0][word0] |= (1<<10) = 0x400 ----
    li      t0, PLIC_ENABLE_BASE
    li      t1, 0x400
    sw      t1, 0(t0)

    # ---- 配 UART: LCR = 0x03 (8N1, DLAB=0) ----
    li      t0, UART_BASE
    li      t1, 0x03
    sb      t1, UART_REG_LCR(t0)

    # ---- s0/a5 初始化 (s0 = handler done flag; a5 = spin counter, 调试用) ----
    li      s0, 0
    li      a5, 0

    # ---- 主帧输出 "OK\n" (IER.ETBEI=0, 不 fire) ----
    li      t1, 'O'
    sb      t1, UART_REG_THR(t0)
    li      t1, 'K'
    sb      t1, UART_REG_THR(t0)
    li      t1, '\n'
    sb      t1, UART_REG_THR(t0)

    # ---- 触发: 启 IER.ETBEI=1 → device_set_pending(10) → MEIP fire ----
    li      t1, 0x02                # ETBEI bit
    sb      t1, UART_REG_IER(t0)

wait_loop:
    addi    a5, a5, 1
    beqz    s0, wait_loop

    # 保存 handler 给的结果到 a0/a1
    mv      a0, s2                  # mcause = 0x8000000B 期望
    mv      a1, s3                  # claim_src = 10 期望

    # halt: test_dev sifive FINISHER PASS → 干净停机, exit_code=0
    # 注: 替代旧 triple-fault 停机 (Smdbltrp MDT abort 后不再可用)
    li      t0, 0x00100000          # TEST_DEV_BASE
    li      t1, 0x5555              # FINISHER_PASS
    sw      t1, 0(t0)               # sifive FINISHER off 0
1:  j       1b                      # 防 fall-through: SRS=0 下个 block 边界退

m_handler:
    # 验 mcause
    csrr    s2, mcause

    # PLIC claim ctx 0
    li      t0, PLIC_CLAIM
    lw      s3, 0(t0)               # 期望 s3 = 10

    # 读 UART IIR (副作用 ack THR empty → device_line 拉低 → PLIC clear)
    li      t1, UART_BASE
    lb      t2, UART_REG_IIR(t1)    # t2 garbage; lb 只为 MMIO read 副作用

    # PLIC complete = 10
    sw      s3, 0(t0)

    # 通知主流程退出
    li      s0, 1
    mret

# ===== 期望结果 =====
# 期望最终寄存器:
#   a0 = 0x8000000B   (INT_BIT | IRQ_M_EXT = 0x80000000 | 11)
#   a1 = 10           (PLIC claim 返的 UART source_id)
#   s0 = 1
#   s2 = 0x8000000B
#   s3 = 10
#   a5 > 0
#
# 期望 stdout:
#   "OK\n"           (3 字符; UART TX 走 putchar)
#
# 期望停机 (test_dev sifive FINISHER PASS):
#   shell 退出码 $? = 0
#   [dispatcher] halted: ... (SRS=0 退主循环)
#
# 不期望:
#   - 任何 [bus] CAUSE_*_ACCESS_FAULT (UART/PLIC 不该产 fault)
#   - 程序崩溃 (ASan/UBSan 报错)
#   - wait_loop 永不退 (说明 UART → PLIC → MEIP 哪环断了)
