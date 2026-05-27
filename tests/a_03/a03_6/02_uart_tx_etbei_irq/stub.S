# a03_6 / 02_uart_tx_etbei_irq
#
# ===== 测试目的 =====
# UART TX async drain + ETBEI 中断真反映 queue 空验收 (跟 a03_3/02_uart_tx_int
# 同思路但加 Smdbltrp boot prelude + 验 drain 完 fire 中断路径):
#   1. boot prelude 清 MDT/SDT (Smdbltrp reset 默认锁 MIE/SIE)
#   2. 配 PLIC: priority[10]=1, threshold[ctx0]=0, enable[ctx0][word0] bit10=0x400
#   3. 配 UART: LCR=0x03 (8N1, DLAB=0)
#   4. 写 32 字节 'A' 到 THR (FIFO 内 enqueue, IER.ETBEI=0 不拉 IRQ)
#   5. 写 IER=ETBEI 启 TX empty 中断; 此刻 tx_count > 0 → thr_int = (ETBEI &&
#      tx_count==0 && !acked) = false → 不立即 fire (跟旧"THRE 永远 1"模型不同;
#      旧模型写 IER 立即 fire, 新模型要等 drain 完). 主帧 spin wait_loop 等 s0.
#   6. drain thread (10 ms 内) wake → drain 整 FIFO → reset tx_count=0 + 清 acked
#      + uart_compute_device_line → thr_int=true → device_set_pending(10) →
#      PLIC 仲裁 → MEIP fire
#   7. m_handler:
#        - 验 mcause = 0x8000000B (INT_BIT | IRQ_M_EXT=11)
#        - lw PLIC claim → 期望 = 10 (UART source_id)
#        - lb UART IIR → 期望 = 0xC2 (FIFO_ON 0xC0 | THR_EMP 0x02); 副作用 ack
#          acked=1 → uart_compute_device_line → device_clear_pending(10)
#        - sw PLIC complete = 10
#        - 设 s0=1 唤醒主帧
#        - mret
#   8. 主帧验 s2 (mcause) / s3 (claim) / s4 (iir) → PASS exit 0; 失败 FAIL.
#
# 跟 a03_3/02_uart_tx_int 体例差别:
#   - a03_3/02 旧"LSR.THRE=1 永远"模型: 写 IER 立即 fire; 没 boot prelude 撞
#     Smdbltrp MDT 锁 MIE bug spin 5 秒.
#   - 本 fixture: 新 queue-based 模型, drain 完后 fire (10 ms 内); 加 prelude 让
#     MIE/SIE 写得进; sifive_test PASS/FAIL 表达验证结果不靠 ecall triple fault.
#
# ===== 思路 =====
# - 32 字节用计数 loop (跟 fixture 01 同).
# - 主帧 wait_loop 用 beqz s0 自旋. drain thread 10 ms 内 fire, hart spin ~10 ms
#   ~ 1M instructions @ 100 MIPS, 不到 1 秒, fixture timeout 安全.
# - handler 内 stash mcause/claim/iir 到 s2/s3/s4 给主帧验证.
# - PASS/FAIL 通过 sifive_test FINISHER 表达, exit code = 0 (PASS) 或 FAIL 编码
#   (mcause/claim/iir 哪个错就把那个对应 nibble 进 exit code).
#
# ===== 期望结果 =====
# exit code = 0 (sifive_test PASS).
# stdout = "AAAA...A" (32 字节 'A', drain 完整, 跟 fixture 01 同体例).
# mcause / claim / iir 都对.

.equ PLIC_PRIO_BASE,      0x0C000000
.equ PLIC_ENABLE_BASE,    0x0C002000
.equ PLIC_THRESHOLD,      0x0C200000
.equ PLIC_CLAIM,          0x0C200004
.equ UART_BASE,           0x10000000
.equ UART_REG_THR,        0
.equ UART_REG_IER,        1
.equ UART_REG_IIR,        2
.equ UART_REG_LCR,        3
.equ UART_IER_ETBEI,      0x02
.equ UART_IIR_FIFO_THRE,  0xC2     # FIFO_ON 0xC0 | THR_EMP 0x02
.equ MIE_MEIP,            0x800    # bit 11
.equ MSTATUS_MIE,         0x8
.equ MCAUSE_EXPECTED,     0x8000000B

.equ TEST_DEV_BASE,       0x00100000
.equ TEST_DEV_FINISHER,   0
.equ TEST_PASS_MAGIC,     0x5555
.equ TEST_FAIL_CMD,       0x3333

.section .text
.option norvc
.globl _start

_start:
    # ---- Boot prelude: 清 MDT / SDT 让 MIE/SIE 写得进 ----
    csrw    0x310, x0               # mstatush 清 MDT
    csrw    sstatus, x0             # 清 SDT (顺便清 SIE/SPIE)

    # ---- M init: mtvec / mie.MEIP / mstatus.MIE ----
    la      t0, m_handler
    csrw    mtvec, t0
    li      t0, MIE_MEIP
    csrw    mie, t0
    csrrsi  x0, mstatus, MSTATUS_MIE

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

    # ---- s0=handler-done flag; s5=spin counter (调试) ----
    li      s0, 0
    li      s5, 0

    # ---- 写 32 字节 'A' 到 THR (IER.ETBEI=0, 不 fire) ----
    li      t1, 32
    li      t2, 'A'
write_loop:
    sb      t2, UART_REG_THR(t0)
    addi    t1, t1, -1
    bnez    t1, write_loop

    # ---- 触发: 开 IER.ETBEI=1; 此刻 tx_count > 0 不立即 fire, drain 完后 fire ----
    li      t1, UART_IER_ETBEI
    sb      t1, UART_REG_IER(t0)

wait_loop:
    addi    s5, s5, 1
    beqz    s0, wait_loop

    # ---- 验 mcause/claim/iir 三个值, 任一不对走 fail ----
    li      t3, MCAUSE_EXPECTED
    bne     s2, t3, fail            # mcause 错
    li      t3, 10
    bne     s3, t3, fail            # claim 错
    li      t3, UART_IIR_FIFO_THRE
    bne     s4, t3, fail            # iir 错

    # ---- PASS: sifive_test FINISHER 写 0x5555 ----
    li      t3, TEST_DEV_BASE
    li      t4, TEST_PASS_MAGIC
    sw      t4, TEST_DEV_FINISHER(t3)
    j       .

fail:
    # ---- FAIL: exit code = 1 (简化; 真要细分用 mcause/claim/iir 各 bit) ----
    li      t3, TEST_DEV_BASE
    li      t5, 0x00010000          # arg=1 (high 16)
    li      t4, TEST_FAIL_CMD       # cmd=0x3333 (low 16); li 拆 lui+addi
    or      t4, t4, t5
    sw      t4, TEST_DEV_FINISHER(t3)
    j       .

m_handler:
    # ---- stash mcause ----
    csrr    s2, mcause

    # ---- PLIC claim ctx 0 → s3 ----
    li      t0, PLIC_CLAIM
    lw      s3, 0(t0)

    # ---- 读 UART IIR (副作用 ack THR empty → uart compute → device_clear_pending) ----
    li      t1, UART_BASE
    lbu     s4, UART_REG_IIR(t1)

    # ---- PLIC complete = 10 ----
    sw      s3, 0(t0)

    # ---- 唤醒主帧 ----
    li      s0, 1
    mret
