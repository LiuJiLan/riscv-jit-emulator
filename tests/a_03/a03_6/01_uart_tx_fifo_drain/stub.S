# a03_6 / 01_uart_tx_fifo_drain
#
# ===== 测试目的 =====
# UART TX async drain 模型端到端 — 验:
#   1. UART_FIFO_SIZE 反压: hart 写 130 字节 (> 128 容量), 后 2 silent drop
#      (跟真 16550A FIFO 满时写 THR 字节丢一致; 跟 RX silent drop 同体例).
#      hart 看不到 drop 这件事 (LSR.THRE 写时检即可避免, 本 fixture 故意不查
#      模拟"polled driver 写不查反压"边界场景).
#   2. drain thread cond_signal 立即 wake + 锁外 batch write(STDOUT_FILENO):
#      130 字节 hart 写完后 spin csrr time ~50 ms, drain thread 应该早把 FIFO
#      drain 干净, 此时 LSR.THRE=1 + TEMT=1 (queue 空, 真反映 tx_count=0).
#   3. LSR queue-based 反映: bit 5 (THRE) = (tx_count < UART_FIFO_SIZE);
#      bit 6 (TEMT) = (tx_count == 0). drain 完后两 bit 应都 1, 即 LSR & 0x60 = 0x60.
#
# 不进 m_handler (mie.MEIP=0); 走 polled LSR 验最终状态. sifive_test FINISHER
# PASS / FAIL exit code 表达验证结果: exit=0 = PASS; exit!=0 = LSR & 0xFF 实际值.
#
# 注: 本 fixture 不验"瞬时反压" (drain 跟 hart 写之间的 race window), 因为
# drain wake ns~μs 级 vs hart 写 ns 级, 瞬时 race 不稳定. 验"drain 最终一致"
# (50 ms >> 10 ms drain 兜底周期, 几乎肯定 drain 完).
#
# ===== 思路 =====
# - Boot prelude: csrw 0x310, x0 (清 mstatush.MDT) + csrw sstatus, x0 (清 SDT/SIE/...)
#   防 Smdbltrp reset 状态锁 MIE/SIE (本 fixture 不用中断, prelude 严格说没必要,
#   但跟 a03_6 其他 fixture 体例一致 + 防 RESET cmd 路径下二次 iter 撞).
# - 写 130 字节用计数 loop (li counter=130; sb; addi -1; bnez), 紧凑.
# - csrr time spin 50 ms (500000 tick @ 100 ns/tick); 跟 a03_4/01 同写法
#   (csrr base + loop sub diff + blt threshold). low 32 wrap 在 ~429 秒, fixture
#   不长不撞.
# - LSR & 0x60 == 0x60 才 PASS; 否则 FAIL 把 LSR 低 8 bit 作 exit code 让人能看
#   实际值 (e.g. exit=0x20 = THRE=1 TEMT=0, drain 没干净; exit=0x40 = TEMT=1 但
#   THRE=0 不可能; exit=0x21 = THRE+DR, 有 stdin 字节进来无关).
#
# ===== 期望结果 =====
# exit code = 0 (sifive_test PASS magic 0x5555).
# 实际 stdout 收到 ~128 字节 'A' (FIFO 真容量, 后 2 丢; 不容易精确, 看 hexdump 即可).
# stderr [dispatcher] halted mstatus.MDT=1 sstatus.SDT=1 (一次 PASS 后 shutdown 走
# cleanup, MDT=1 是 reset 默认值, fixture 不动 MDT 是 SET 路径 PASS 没撞 critical-error).

.equ UART_BASE,           0x10000000
.equ UART_REG_THR,        0
.equ UART_REG_LSR,        5
.equ UART_LSR_DR,         0x01
.equ UART_LSR_THRE,       0x20
.equ UART_LSR_TEMT,       0x40
.equ UART_LSR_THRE_TEMT,  0x60

.equ TEST_DEV_BASE,       0x00100000
.equ TEST_DEV_FINISHER,   0
.equ TEST_PASS_MAGIC,     0x5555
.equ TEST_FAIL_CMD,       0x3333

.section .text
.option norvc
.globl _start

_start:
    # ---- Boot prelude: 清 MDT / SDT (Smdbltrp reset 默认锁 MIE/SIE) ----
    csrw    0x310, x0               # mstatush 清 MDT (binutils 2.38 以下不认 mstatush 助记符)
    csrw    sstatus, x0             # 清 SDT (顺便清 SIE/SPIE 兼容多版本 binutils)

    # ---- 写 130 字节 'A' 到 THR (超 UART_FIFO_SIZE=128, 后 2 silent drop) ----
    li      t0, UART_BASE
    li      t1, 130
    li      t2, 'A'
write_loop:
    sb      t2, UART_REG_THR(t0)
    addi    t1, t1, -1
    bnez    t1, write_loop

    # ---- spin csrr time ~50 ms (500000 tick @ 100 ns/tick) 让 drain thread 完成 ----
    csrr    s1, time                # base
    li      s2, 500000              # 50 ms threshold (drain 10 ms 兜底 × 5 安全余量)
spin_loop:
    csrr    s3, time
    sub     s4, s3, s1
    blt     s4, s2, spin_loop

    # ---- 读 LSR, 期望 THRE=1 + TEMT=1 (queue 空, 真反映 tx_count=0) ----
    lbu     s5, UART_REG_LSR(t0)
    andi    s6, s5, UART_LSR_THRE_TEMT
    li      s7, UART_LSR_THRE_TEMT
    bne     s6, s7, fail

    # ---- PASS: sifive_test FINISHER 写 0x5555 (low 16 = cmd, high 16 = arg=0) ----
    li      t3, TEST_DEV_BASE
    li      t4, TEST_PASS_MAGIC
    sw      t4, TEST_DEV_FINISHER(t3)
    j       .                       # 防御性 (sifive_test PASS 触发 shutdown 不会回这)

fail:
    # ---- FAIL: exit code = LSR & 0xFF (实际值); cmd=0x3333, arg=LSR 低 8 bit ----
    li      t3, TEST_DEV_BASE
    andi    t5, s5, 0xFF
    slli    t5, t5, 16              # arg (high 16) = LSR & 0xFF
    li      t4, TEST_FAIL_CMD       # cmd (low 16) = 0x3333 (li 拆 lui+addi)
    or      t4, t4, t5
    sw      t4, TEST_DEV_FINISHER(t3)
    j       .
