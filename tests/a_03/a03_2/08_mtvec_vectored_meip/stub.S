# a03_2 / 08_mtvec_vectored_meip
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# mtvec vectored mode (mode=1): fire async interrupt 时 pc = base + 4*cause_low.
# 验 MEIP (cause_low=11) 跳 m_vec_base+44 这条 entry.
#
# ===== 验证手法 =====
# vectored jump table 每个 entry = `jal x5, m_vec_handler` (x5=t0 当临时 ra; jal 后
# x5 = entry_addr + 4 = base + 4*cause_low + 4). handler 内算 cause_low = (x5 - base
# - 4) / 4, compare == 11, mismatch 走 fail_loop 死循环 (release timeout 暴露 fail);
# match 才走 PLIC claim/complete + mret 正常路径 → 主流程 test_dev FINISHER 干净停机.
#
# 比用 `j entry / li sN=marker` 强: release 不 dump GPR, 用 exit 0 vs timeout
# 区分 pass/fail, 不依赖 debug build GPR 验.
#
# ===== 期望结果 =====
#   pass: test_dev FINISHER PASS → exit code 0 (release / debug 都看得到; 跳对 entry)
#   fail (vectored 跳错 entry): timeout (release) / 卡 fail_loop (debug)
#   debug 下 GPR: s0=1 / s1=11 (cause_low 实际命中) / s3=44 (offset)
#                 x30=0x100000 / x31=0x5555 (停机 scratch)
# (注: 原停机靠 csrw mtvec=0 + ecall triple fault; a_03 Smdbltrp 后变 MDT abort, 改
#  test_dev 干净停机; pass 判据从 in_trap=3 改 exit 0。)

.equ TEST_DEV_BASE,      0x00100000
.equ TEST_DEV_SET_OFF,   0x40
.equ TEST_DEV_CLEAR_OFF, 0x44

.section .text
.option norvc
.globl _start

_start:
    la      t0, m_vec_base
    ori     t0, t0, 1                      # mode=1 vectored
    csrw    mtvec, t0

    li      t0, 0x800;  csrw mie, t0       # MEIE

    # PLIC setup ctx 0: priority[3]=1, threshold=0, enable bit3
    li      t0, 0x0C000000;  li t1, 1;  sw t1, 12(t0)
    li      t0, 0x0C200000;  sw x0, 0(t0)
    li      t0, 0x0C002000;  li t1, 0x8;  sw t1, 0(t0)

    li      s0, 0
    li      s1, 0
    li      s3, 0

    li      t1, TEST_DEV_BASE
    li      t0, 3;  sw t0, TEST_DEV_SET_OFF(t1)
    csrw    0x310, x0               # mstatush 清 MDT (A.7 prelude; 否则下条 MIE 写被强制 0)
    csrrsi  x0, mstatus, 0x8                # MIE=1 → fire MEIP → vectored

wait_loop:
    beqz    s0, wait_loop

    # halt: test_dev FINISHER PASS 干净停机 (scratch x30/x31)
    li      x30, 0x100000
    li      x31, 0x5555
    sw      x31, 0(x30)
1:  j       1b                      # 防 fall-through: SRS=0 下个 block 边界退

# ============================================================================
# m_vec_base — vectored jump table; 每 entry = jal x5, handler (x5 = entry+4)
# ============================================================================
.balign 4
m_vec_base:
    jal     x5, m_vec_handler              # +0   sync (vectored 不影响)
    jal     x5, m_vec_handler              # +4   IRQ 1
    jal     x5, m_vec_handler              # +8
    jal     x5, m_vec_handler              # +12  IRQ 3
    jal     x5, m_vec_handler              # +16
    jal     x5, m_vec_handler              # +20  IRQ 5
    jal     x5, m_vec_handler              # +24
    jal     x5, m_vec_handler              # +28  IRQ 7
    jal     x5, m_vec_handler              # +32
    jal     x5, m_vec_handler              # +36  IRQ 9
    jal     x5, m_vec_handler              # +40
    jal     x5, m_vec_handler              # +44  IRQ 11 M_EXT (期望)

m_vec_handler:
    # x5 = base + 4*cause_low + 4 (jal 之后); 算 cause_low
    la      t1, m_vec_base
    sub     t2, x5, t1                     # t2 = 4*cause_low + 4
    addi    t2, t2, -4                     # t2 = 4*cause_low
    mv      s3, t2                         # s3 = offset (debug 验: 期望 44)
    srli    t1, t2, 2                      # t1 = cause_low
    mv      s1, t1                         # s1 = cause_low (debug 验: 期望 11)

    # mismatch (期望 11) → fail_loop 死循环
    li      t2, 11
    bne     t1, t2, fail_loop

    # 跳对了 — 正常 PLIC claim/complete + mret
    li      t0, 0x0C200004
    lw      s2, 0(t0)
    sw      s2, 0(t0)
    li      t0, 3;  li t1, TEST_DEV_BASE;  sw t0, TEST_DEV_CLEAR_OFF(t1)

    li      s0, 1
    mret

fail_loop:
    j       fail_loop                       # release timeout / debug 卡死, 都暴露 fail
