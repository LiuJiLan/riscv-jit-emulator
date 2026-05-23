# a03_2 / 04_plic_enable_mask
#
# ===== 测试目的 =====
# PLIC enable bitmap 截断: enable bit = 0 时, source pending 但仲裁不送 hart.
# phase 1: enable[ctx0][word0] = 0 (全 disable), set src=3 → 不 fire
# phase 2: enable[ctx0][word0] = (1<<3) → 下一 block fire MEIP
#
# ===== 期望结果 =====
#   s0 = 1
#   s1 = 1            (phase marker)
#   s2 = 3            (claim)
#   s4 = 1            (handler 进入时 s1 snapshot, 必须 = 1 表示 phase 2 才 fire)

.equ CSR_TOHOST, 0x800
.section .text
.option norvc
.globl _start

_start:
    la      t0, m_handler
    csrw    mtvec, t0
    li      t0, 0x800;  csrw mie, t0

    # PLIC: priority[3]=5, threshold=0, enable = 0 (全 disable)
    li      t0, 0x0C000000;  li t1, 5;  sw t1, 12(t0)
    li      t0, 0x0C200000;  sw x0, 0(t0)
    li      t0, 0x0C002000;  sw x0, 0(t0)

    li      s0, 0
    li      s1, 0
    li      s2, 0
    li      s4, 0

    li      t0, 0x80000003;  csrw CSR_TOHOST, t0   # set src=3 (但 enable=0 不送)

    csrrsi  x0, mstatus, 0x8

    li      t2, 5
phase1_loop:
    addi    t2, t2, -1
    bnez    t2, phase1_loop

    # phase 2: enable bit3 → fire
    li      t0, 0x0C002000;  li t1, 0x8;  sw t1, 0(t0)
    addi    s1, s1, 1

wait_loop:
    beqz    s0, wait_loop

    csrw    mtvec, x0
    ecall

m_handler:
    mv      s4, s1

    li      t0, 0x0C200004
    lw      s2, 0(t0)

    li      t0, 3;  csrw CSR_TOHOST, t0
    li      t0, 0x0C200004;  sw s2, 0(t0)

    li      s0, 1
    mret
