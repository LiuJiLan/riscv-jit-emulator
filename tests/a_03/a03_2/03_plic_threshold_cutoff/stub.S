# a03_2 / 03_plic_threshold_cutoff
#
# ===== 测试目的 =====
# PLIC threshold 截断: priority ≤ threshold 时, source pending 但仲裁不送 hart.
# phase 1: priority[3]=5, threshold[ctx0]=5, set src=3 → MEIP=0 (5 不 > 5)
# phase 2: 降 threshold=0 → 下一 block trap_check fire MEIP
# handler 验 fire 时机 = phase 2 (s4 snapshot 验)
#
# ===== 期望结果 =====
#   s0 = 1            (wait_loop 退出)
#   s1 = 1            (phase marker, 必须先到 2 才 fire — 实际记录 1 因为 phase 2 加 1)
#   s2 = 3            (claim 返 src=3)
#   s4 = 1            (handler 进入时 s1 snapshot, 必须 = 1 表示 phase 2 才 fire)

.equ TEST_DEV_BASE,      0x00100000
.equ TEST_DEV_SET_OFF,   0x40
.equ TEST_DEV_CLEAR_OFF, 0x44

.section .text
.option norvc
.globl _start

_start:
    la      t0, m_handler
    csrw    mtvec, t0
    li      t0, 0x800;  csrw mie, t0

    # PLIC: priority[3]=5, threshold[ctx0]=5, enable bit3
    li      t0, 0x0C000000;  li t1, 5;  sw t1, 12(t0)
    li      t0, 0x0C200000;  li t1, 5;  sw t1, 0(t0)
    li      t0, 0x0C002000;  li t1, 0x8;  sw t1, 0(t0)

    li      s0, 0
    li      s1, 0
    li      s2, 0
    li      s4, 0

    # set src=3 via test_dev (此时 priority=5, threshold=5, 5 ≤ 5 不送)
    li      t1, TEST_DEV_BASE
    li      t0, 3;  sw t0, TEST_DEV_SET_OFF(t1)

    csrrsi  x0, mstatus, 0x8                # MIE=1, 但 mip.MEIP=0 不 fire

    # phase 1 loop: 跑几条指令验不 fire
    li      t2, 5
phase1_loop:
    addi    t2, t2, -1
    bnez    t2, phase1_loop

    # phase 2: 降 threshold=0, mip.MEIP=1 → 下一 block fire
    li      t0, 0x0C200000;  sw x0, 0(t0)
    addi    s1, s1, 1                       # phase 2 marker (handler 应看到 s1=1)

wait_loop:
    beqz    s0, wait_loop

    csrw    mtvec, x0
    ecall

m_handler:
    mv      s4, s1                          # snapshot phase marker (期望 = 1)

    li      t0, 0x0C200004
    lw      s2, 0(t0)                       # claim → 期望 s2 = 3

    li      t0, 3;  li t1, TEST_DEV_BASE;  sw t0, TEST_DEV_CLEAR_OFF(t1)
    li      t0, 0x0C200004;  sw s2, 0(t0)   # complete

    li      s0, 1
    mret
