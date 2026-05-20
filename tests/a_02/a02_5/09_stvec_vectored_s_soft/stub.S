# a02_5 / 09_stvec_vectored_s_soft
#
# ===== 测试目的 =====
# stvec mode=1 vectored + delegated SSIP fire → pc = stvec_base + 4 * IRQ_S_SOFT。
# 跟 fixture 07 区别: stvec (S-mode trap vector) 代替 mtvec; 验 stvec mode=1 路径跟 mtvec
# 同形态生效 (trap_set_interrupt_state 内部统一读 xtvec[deliver_priv]).
#
# ===== 思路 =====
#   1. M setup: mtvec=m_handler, stvec=s_vec_table|1, mideleg.SSIE=0x2, mie.SSIE=0x2
#   2. mret 切 S (mstatus.MPP=S, mstatus.SIE=1)
#   3. S-mode csrw sip 设 SSIP (软件 inject; SIP_WRITABLE_MASK 仅 SSIP)
#   4. trap_check_interrupt 看 priv=S, mideleg.SSIE=1 → deliver S; cause_low=1
#   5. trap_set_interrupt_state 看 stvec.mode=1 → pc = base + 4*1 = slot 1
#   6. slot 1 = `j s_handler_ssip` → 真 handler
#   7. handler 验 scause = 0x80000001, 清 SSIP, sret
#   8. S resume → wait_loop 退出 → ecall → M halt path

.section .text
.option norvc
.globl _start
_start:
    # M setup
    la      t0, m_handler
    csrw    mtvec, t0

    la      t0, s_vec_table
    ori     t0, t0, 1            # stvec mode=1 vectored
    csrw    stvec, t0

    # mideleg.SSIE = 0x2 (delegate SSIP to S)
    li      t0, 0x2
    csrw    mideleg, t0

    # mie.SSIE = 0x2 (sie 是 mie mask view; 跟 mie 同字段)
    li      t0, 0x2
    csrw    mie, t0

    # init s0/a5 (mret 进 S 前; S 起步即 csrw sip fire, init 必须先做)
    li      s0, 0
    li      a5, 0

    la      t0, s_program
    csrw    mepc, t0

    # mstatus: MPP=PRIV_S (0x800) + SIE=1 (0x2)
    li      t0, 0x802
    csrw    mstatus, t0

    mret                         # 切 S → s_program

s_program:
    # S-mode 起步: csrw sip 设 SSIP (软件 inject 通过 _mip_sw bit 1)
    li      t0, 0x2
    csrw    sip, t0              # 触发 SSIP pending → 下一轮 dispatcher 顶 fire

wait_loop:
    addi    a5, a5, 1
    beqz    s0, wait_loop

    mv      a0, s1               # a0 = scause from s_handler_ssip

    ecall                        # halt: ecall_from_S → M (medeleg=0 默认) → m_handler

# stvec.BASE 4 字节对齐
.align 2
s_vec_table:
    j       halt                 # slot 0  (sync exception in S-mode 不该走这; 防御)
    j       s_handler_ssip       # slot 1  (IRQ_S_SOFT) — 验证点: pc = stvec_base + 4
    j       halt                 # slot 2
    j       halt                 # slot 3
    j       halt                 # slot 4
    j       halt                 # slot 5  (IRQ_S_TIMER)
    j       halt                 # slot 6
    j       halt                 # slot 7
    j       halt                 # slot 8
    j       halt                 # slot 9  (IRQ_S_EXT)
    j       halt                 # slot 10
    j       halt                 # slot 11

s_handler_ssip:
    csrr    s1, scause           # 期望 0x80000001 (CAUSE_INTERRUPT_BIT | IRQ_S_SOFT)

    csrw    sip, x0              # 清 SSIP (S-mode 可清 SSIP via SIP_WRITABLE_MASK)

    li      s0, 1
    sret

m_handler:
    # halt path: ecall_from_S 到 M; 清 mtvec + ecall_from_M → triple fault
    csrw    mtvec, x0
    ecall

halt:
    # s_vec_table 错跳 slot 兜底 (不该走到)
    j       m_handler

# ===== 期望结果 =====
# 期望最终寄存器:
#   a0 = 0x80000001   (scause = CAUSE_INTERRUPT_BIT | IRQ_S_SOFT)
#   s0 = 1
#   s1 = 0x80000001
#   a5 > 0
#
# 关键验证:
#   - s_handler_ssip 真跑 (s0=1, s1=0x80000001) → stvec vectored 跳到 base + 4
#   - 若 stvec 走 mode=0 direct, 进 slot 0 = `j halt` → s0/s1=0
