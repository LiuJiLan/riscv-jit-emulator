# a02_5 / 05_int_ssip_via_csrw
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# SSIP 软件 inject path (S-mode 自己 csrw sip 设 SSIP, mideleg.SSIE=1 deliver S)。
# 形态特点: 验证 SSIP 是 S-mode csrw sip 可写位 (spec §5.1.4: S-mode csrw sip 仅 SSIP 可写,
# STIP/SEIP RO from S 视角); 验证 mip <-> sip view 一致 (类 (5) 软件可写子集 _mip_sw).
#
# ===== 思路 =====
#   1. M setup: mtvec=m_handler, stvec=s_handler, mideleg.SSIE=0x2, mie.SSIE=0x2
#   2. M mret 到 S-mode (mstatus.MPP=PRIV_S, mstatus.SIE=1)
#   3. S-mode csrw sip = 0x2 → 写 _mip_sw bit 1 (SIP_WRITABLE_MASK 仅 SSIP 一位, S-mode 视图)
#   4. 下一轮 trap_check_interrupt 看 mip.SSIP=1 & mie.SSIE=1 & mideleg.SSIE=1 & priv=S &
#      mstatus.SIE=1 → fire IRQ_S_SOFT, deliver S → scause=0x80000001
#   5. s_handler: 验 scause, 清 SSIP via csrw sip, x0 (S-mode 自清 SSIP 是 spec allowed 操作), sret
#   6. halt path 同 04 (ecall → M handler → triple fault)

.section .text
.option norvc
.globl _start
_start:
    # ---- M-mode setup ----
    la      t0, m_handler
    csrw    mtvec, t0

    la      t0, s_handler
    csrw    stvec, t0

    # mideleg.SSIE = bit 1 (delegate S-software interrupt to S)
    li      t0, 0x2
    csrw    mideleg, t0

    # mie.SSIE = bit 1
    li      t0, 0x2
    csrw    mie, t0

    # init S-mode local state
    li      s0, 0
    li      a5, 0

    # mepc = s_program
    la      t0, s_program
    csrw    mepc, t0

    # mstatus: MPP=S (0x800), SIE=1 (0x2)
    li      t0, 0x802
    csrw    mstatus, t0

    mret                         # 切 S → s_program

s_program:
    # S-mode 起步: 自己 csrw sip 设 SSIP 触发软件中断
    li      t0, 0x2              # SSIP bit 1
    csrw    sip, t0              # csr_sip_write: SIP_WRITABLE_MASK 仅 SSIP, 写入 _mip_sw bit 1

wait_loop:
    addi    a5, a5, 1
    beqz    s0, wait_loop

    mv      a0, s1               # a0 = scause (期望 0x80000001)

    ecall                        # halt path

s_handler:
    csrr    s1, scause           # 期望 0x80000001 (CAUSE_INTERRUPT_BIT | IRQ_S_SOFT)

    # 清 SSIP via csrw sip, x0 (S-mode 可清自己 SSIP; RV spec §5.1.4)
    csrw    sip, x0

    li      s0, 1
    sret

m_handler:
    # halt: ecall_from_S 到 M (medeleg=0 默认) → test_dev 干净停机 (scratch x30/x31)
    li      x30, 0x100000        # test_dev base
    li      x31, 0x5555          # FINISHER PASS magic
    sw      x31, 0(x30)          # 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望最终寄存器:
#   a0 = 0x80000001   (CAUSE_INTERRUPT_BIT | IRQ_S_SOFT)
#   s0 = 1
#   s1 = 0x80000001
#   a5 > 0
#   x30 = 0x100000 / x31 = 0x5555 (停机用 scratch: test_dev base / FINISHER PASS magic)
#   停机 = test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
