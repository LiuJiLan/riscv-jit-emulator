# a03_2 / 09_stvec_vectored_seip
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# stvec vectored mode (mode=1): fire SEIP delegated → S handler vectored, cause_low=9
# 期望跳 s_vec_base+36.
#
# ===== 验证手法 =====
# 跟 08 同 — jal x5, handler 自动留 ra=entry+4; handler 算 cause_low compare
# 期望 9, mismatch fail_loop 死循环; match 正常 PLIC ctx1 claim/complete + sret.
#
# ===== 期望结果 =====
#   pass: test_dev FINISHER PASS → exit code 0 (跳对 entry → S-ecall → m_halt 干净停机)
#   fail: timeout / 卡 fail_loop
#   debug 下 GPR: s0=1 / s1=9 / s2=3 / s3=36
#                 x30=0x100000 / x31=0x5555 (停机 scratch)
# (注: 原停机靠 S-ecall → m_halt csrw mtvec=0 + ecall triple fault; a_03 Smdbltrp 后变
#  MDT abort, m_halt 改 test_dev 干净停机; pass 判据从 in_trap=3 改 exit 0。)

.equ TEST_DEV_BASE,      0x00100000
.equ TEST_DEV_SET_OFF,   0x40
.equ TEST_DEV_CLEAR_OFF, 0x44

.section .text
.option norvc
.globl _start

_start:
    la      t0, m_halt;  csrw mtvec, t0

    la      t0, s_vec_base
    ori     t0, t0, 1                      # mode=1 vectored
    csrw    stvec, t0

    li      t0, 0x200;  csrw mideleg, t0   # SEIP delegated
    li      t0, 0x200;  csrw mie, t0       # SEIE

    # PLIC setup ctx 1: priority[3]=1, threshold=0, enable bit3
    li      t0, 0x0C000000;  li t1, 1;  sw t1, 12(t0)
    li      t0, 0x0C002080;  li t1, 0x8;  sw t1, 0(t0)
    li      t0, 0x0C201000;  sw x0, 0(t0)

    li      s0, 0
    li      s1, 0
    li      s2, 0
    li      s3, 0

    li      t1, TEST_DEV_BASE
    li      t0, 3;  sw t0, TEST_DEV_SET_OFF(t1)

    li      t0, 0x800;  csrw mstatus, t0   # MPP=S
    la      t0, s_target;  csrw mepc, t0
    mret

s_target:
    csrrsi  x0, sstatus, 0x2                # SIE=1 → fire SEIP vectored

wait_loop:
    beqz    s0, wait_loop
    ecall                                   # S-ecall → m_halt

# ============================================================================
# s_vec_base — vectored jump table
# ============================================================================
.balign 4
s_vec_base:
    jal     x5, s_vec_handler              # +0
    jal     x5, s_vec_handler              # +4   IRQ 1
    jal     x5, s_vec_handler              # +8
    jal     x5, s_vec_handler              # +12
    jal     x5, s_vec_handler              # +16
    jal     x5, s_vec_handler              # +20
    jal     x5, s_vec_handler              # +24
    jal     x5, s_vec_handler              # +28
    jal     x5, s_vec_handler              # +32
    jal     x5, s_vec_handler              # +36  IRQ 9 S_EXT (期望)

s_vec_handler:
    la      t1, s_vec_base
    sub     t2, x5, t1
    addi    t2, t2, -4
    mv      s3, t2                          # 期望 36
    srli    t1, t2, 2
    mv      s1, t1                          # 期望 9

    li      t2, 9
    bne     t1, t2, fail_loop

    li      t0, 0x0C201004
    lw      s2, 0(t0)
    sw      s2, 0(t0)
    li      t0, 3;  li t1, TEST_DEV_BASE;  sw t0, TEST_DEV_CLEAR_OFF(t1)

    li      s0, 1
    sret

fail_loop:
    j       fail_loop

m_halt:
    # S-ecall deliver 到此 (M); test_dev FINISHER PASS 干净停机 (scratch x30/x31)
    li      x30, 0x100000
    li      x31, 0x5555
    sw      x31, 0(x30)
