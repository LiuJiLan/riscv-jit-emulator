# a03_2 / 07_int_seip_delegated
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# SEIP 通过 mideleg 代理到 S handler (而不是 M handler):
#   trap_set_interrupt_state: priv < M AND mideleg.bit(cause_low) = 1 → deliver_priv = S
#
# 触发用 PLIC (硬件 SEIP) 而不是 SEIP_sw 软件位 — 因为 S-mode csrw sip 只能动 SSIP
# (SIP_WRITABLE_MASK = 1<<IRQ_S_SOFT 只允许 SSIP), SEIP/STIP 在 sip 视角 RO; s_handler
# 无法清 SEIP_sw 会死循环. 用 PLIC 走真实 OS 用法 (claim+complete+device_clear).
#
# PLIC ctx 索引: hart 0 S ctx = 2*0 + 1 = ctx 1 (plic_init 内 hardcoded
# plic_ctx_map[h<<2 | PRIV_S] = 2h+1).
#
# fixture 流程:
#   1. M setup: mtvec=m_halt, stvec=s_handler, mideleg.SEIP=1, mie.SEIE=1
#   2. PLIC setup ctx 1: priority[3]=1, threshold[ctx1]=0, enable[ctx1][word0] bit3
#   3. sw TEST_DEV_SET(t1), 3 → PLIC device_set src=3 → ctx 1 pending → SEIP=1
#   4. mret 切 S (mstatus.MPP=S)
#   5. S 开 SIE → fire SEIP, deliver S (mideleg.SEIP=1), jump stvec
#   6. s_handler: scause=0x80000009; PLIC claim @ ctx 1 + complete + device_clear; sret
#   7. 回 S 退 wait_loop, S-ecall → m_halt → test_dev FINISHER PASS 干净停机
#
# ===== 期望结果 =====
#   s0 = 1
#   s1 = 0x80000009   (CAUSE_INTERRUPT_BIT | IRQ_S_EXT, 写 scause)
#   s2 = 3            (PLIC claim @ ctx 1 返 src=3)
#   x30 = 0x100000 / x31 = 0x5555 (停机 scratch)
#   停机 = test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
# (注: 原停机靠 S-ecall → m_halt csrw mtvec=0 + ecall triple fault; a_03 Smdbltrp 后变
#  MDT abort, m_halt 改 test_dev 干净停机 (S-ecall → M 的 deliver 路径保留)。
#  绝对 pc / total_count 见 REVIEW B.3 段。)

.equ TEST_DEV_BASE,      0x00100000
.equ TEST_DEV_SET_OFF,   0x40
.equ TEST_DEV_CLEAR_OFF, 0x44

.section .text
.option norvc
.globl _start

_start:
    la      t0, m_halt
    csrw    mtvec, t0
    la      t0, s_handler
    csrw    stvec, t0

    # mideleg = SEIP (bit 9 = 0x200)
    li      t0, 0x200;  csrw mideleg, t0
    # mie = SEIE (bit 9 = 0x200)
    li      t0, 0x200;  csrw mie, t0

    # PLIC setup for ctx 1 (= hart 0 S ctx)
    # priority[3] = 1
    li      t0, 0x0C000000;  li t1, 1;  sw t1, 12(t0)
    # ctx 1 enable @ PLIC_BASE + 0x2000 + 0x80*1 = 0x0C002080
    li      t0, 0x0C002080;  li t1, 0x8;  sw t1, 0(t0)
    # ctx 1 threshold @ PLIC_BASE + 0x200000 + 0x1000*1 = 0x0C201000
    li      t0, 0x0C201000;  sw x0, 0(t0)

    # init flags (必须在 mret 之前 — mret 后 PC=s_target 不回 init)
    li      s0, 0
    li      s1, 0
    li      s2, 0

    # set PLIC src=3 via test_dev → ctx 1 pending → mip.SEIP=1
    li      t1, TEST_DEV_BASE
    li      t0, 3;  sw t0, TEST_DEV_SET_OFF(t1)

    # mret 切 S: mstatus.MPP=S (=0b01<<11 = 0x800)
    li      t0, 0x800;  csrw mstatus, t0
    la      t0, s_target;  csrw mepc, t0

    mret

s_target:
    # 开 SIE → 下一 block fire SEIP, deliver S
    csrrsi  x0, sstatus, 0x2

wait_loop:
    beqz    s0, wait_loop

    # S-ecall (cause 9 from S) — medeleg.SECALL_FROM_S 默认 0 → deliver M (m_halt)
    ecall

s_handler:
    csrr    s1, scause                     # 期望 0x80000009

    # PLIC ctx 1 claim/complete @ 0x0C201004
    li      t0, 0x0C201004
    lw      s2, 0(t0)                      # claim → 期望 s2=3
    sw      s2, 0(t0)                      # complete
    # device_clear src=3 via test_dev MMIO
    li      t0, 3;  li t1, TEST_DEV_BASE;  sw t0, TEST_DEV_CLEAR_OFF(t1)

    li      s0, 1
    sret

m_halt:
    # S-ecall deliver 到此 (M); test_dev FINISHER PASS 干净停机 (scratch x30/x31)
    li      x30, 0x100000
    li      x31, 0x5555
    sw      x31, 0(x30)
