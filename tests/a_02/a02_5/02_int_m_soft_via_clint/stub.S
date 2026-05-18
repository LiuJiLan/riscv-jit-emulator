# a02_5 / 02_int_m_soft_via_clint
# M-mode software interrupt 通过 CLINT.msip MMIO 触发:
#   1. CLINT msip[0] @ 0x02000000 写 1 (MMIO; clint_msip_pending 异步源)
#   2. csr_mip_read 合成 bit 3 (MSIP) ← clint_msip_pending(0)
#   3. trap_check_interrupt: mip.MSIP & mie.MSIE & mstatus.MIE → fire IRQ_M_SOFT (3)
#   4. m_handler: 验 mcause = 0x80000003, 清 msip 写 0 via MMIO, mret
#
# 跟 fixture 01 区别: 触发源是 CLINT.msip (异步 atomic source) 不是 mtime 派生;
# 清 pending 也是 MMIO 写 0 (mip.MSIP 是 RO 视角不能 csrw 清).

.section .text
.option norvc
.globl _start
_start:
    la      t0, m_handler
    csrw    mtvec, t0

    # mie.MSIE = (1<<3) = 0x8
    li      t0, 0x8
    csrw    mie, t0

    # mstatus.MIE = 1
    csrrsi  x0, mstatus, 0x8

    # 写 CLINT.msip[0] = 1 → 触发 M software interrupt
    li      t0, 0x02000000          # msip[0]
    li      t1, 1
    sw      t1, 0(t0)

    # 此时下一轮 dispatcher 顶 check_interrupt 应 fire
    li      s0, 0
    li      a5, 0
wait_loop:
    addi    a5, a5, 1
    beqz    s0, wait_loop

    mv      a0, s1

    csrw    mtvec, x0
    ecall

m_handler:
    csrr    s1, mcause              # 期望 0x80000003 (CAUSE_INTERRUPT_BIT | IRQ_M_SOFT)

    # 清 msip[0] = 0 via MMIO (RV spec: MSIP RO from csrw mip 视角, 必走 MMIO)
    li      t0, 0x02000000
    sw      x0, 0(t0)

    li      s0, 1
    mret

# 期望最终寄存器:
#   a0 = 0x80000003   (CAUSE_INTERRUPT_BIT | IRQ_M_SOFT = 0x80000000 | 3)
#   s0 = 1
#   s1 = 0x80000003
#   a5 > 0
