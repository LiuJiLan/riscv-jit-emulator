# a02_5 / 04_int_s_timer_delegated
#
# ===== 测试目的 =====
# S-mode timer interrupt 委托 (mideleg.STIE=1, M-mode 软件 inject STIP)。
# 形态特点: 验证 mideleg / S-mode trap delivery / scause 高位 = CAUSE_INTERRUPT_BIT 拼接.
#
# ===== 思路 =====
#   1. M setup: mtvec=m_handler, stvec=s_handler, mideleg.STIE=0x20, mie.STIE=0x20
#   2. M csrw mip = 0x20 (软件 inject _mip_sw bit 5 = STIP; 项目无 Sstc, STIP 只能软件 inject)
#   3. M mret 到 S-mode (mepc=s_program, mstatus.MPP=PRIV_S=1, mstatus.SIE=1)
#   4. S 起步即被 trap_check_interrupt 看到: mip.STIP=1 & mie.STIE=1 & mideleg.STIE=1 &
#      priv=S & mstatus.SIE=1 → fire IRQ_S_TIMER, deliver S (mideleg.bit(5)=1 → deliver S)
#   5. s_handler: 验 scause=0x80000005, 清 sie.STIE (csrw sie, 0; mie.STIE=0 防重复), sret
#   6. S 恢复 wait_loop → s0=1 → ecall_from_S → cause=9 deliver M (medeleg=0 默认) →
#      m_handler halt: 清 mtvec, ecall → triple fault

.section .text
.option norvc
.globl _start
_start:
    # ---- M-mode setup ----
    la      t0, m_handler
    csrw    mtvec, t0

    la      t0, s_handler
    csrw    stvec, t0

    # mideleg.STIE = bit 5 (delegate S-timer interrupt to S)
    li      t0, 0x20
    csrw    mideleg, t0

    # mie.STIE = bit 5
    li      t0, 0x20
    csrw    mie, t0

    # 软件 inject STIP: csrw mip 写 _mip_sw bit 5 (MIP_SW_WRITABLE_MASK 含 bit 5)
    li      t0, 0x20
    csrw    mip, t0

    # init S-mode local state (在 M 这里设, 因为 sret 落 s_program 顶时不重设)
    li      s0, 0
    li      a5, 0

    # 设 mepc = s_program (即 wait_loop 入口)
    la      t0, s_program
    csrw    mepc, t0

    # mstatus: MPP=PRIV_S (1 << 11 = 0x800), SIE=1 (bit 1 = 0x2)
    # MIE 默认 0 (S-mode 走 SIE; MIE 用于 M-mode)
    li      t0, 0x802
    csrw    mstatus, t0

    mret                         # 切 S-mode → s_program (= wait_loop)

s_program:
wait_loop:
    addi    a5, a5, 1
    beqz    s0, wait_loop

    mv      a0, s1               # a0 = scause (期望 0x80000005)

    # halt path: ecall_from_S (cause=9) → M trap → m_handler 清 mtvec + ecall → triple fault
    ecall

s_handler:
    csrr    s1, scause           # 期望 0x80000005 (CAUSE_INTERRUPT_BIT | IRQ_S_TIMER)

    # 清 sie.STIE → mie.STIE=0 防重复 fire (sret 后 mip.STIP 仍 1 但 mie.STIE=0 → ready=0)
    csrw    sie, x0

    li      s0, 1
    sret

m_handler:
    # M-mode 处理: 期望此 fixture 不到 M timer 路径 (mideleg delegate 了); m_handler 实
    # 际只承接 S 的 ecall (cause=9 deliver M; medeleg=0 默认) → 充当 halt 入口.
    csrw    mtvec, x0
    ecall                        # ecall_from_M (cause=11) → mtvec=0 → triple fault

# ===== 期望结果 =====
# 期望最终寄存器:
#   a0 = 0x80000005   (handler 写 s1, main mv 到 a0)
#   s0 = 1            (handler 设置)
#   s1 = 0x80000005   (CAUSE_INTERRUPT_BIT | IRQ_S_TIMER)
#   a5 > 0
#
# 期望 trap dump (S 槽留有 interrupt deliver 信息):
#   scause = 0x80000005   (trap_set_interrupt_state 写 xcause[PRIV_S])
#   sepc   = s_program addr (trap_set_interrupt_state 写 xepc[PRIV_S] = 当前 pc)
#   stval  = 0            (interrupt tval RV spec)
#   stvec  = s_handler addr (不变, 设置时 mode=0)
