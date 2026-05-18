# a02_5 / 01_int_m_timer_basic
# M-mode timer interrupt 基本路径 — 端到端验证 (假设 mtime 在主动步进):
#   1. mtime 推进 (实现方式取决于当前阶段, 见 platform/clint.h 顶段; 行为受步进
#      实现影响, 但 mip.MTIP 合成读 + trap 路径语义不变)
#   2. trap_check_interrupt 看到 mip.MTIP (clint_timer_pending) & mie.MTIE & mstatus.MIE → fire
#   3. trap_set_interrupt_state: deliver M (mideleg.MTIE=0), xcause = CAUSE_INTERRUPT_BIT | IRQ_M_TIMER
#      = 0x80000007; jump-to xtvec base (mode=0 direct)
#   4. m_handler: 验 mcause, 清 MTIP via 写 mtimecmp = UINT64_MAX, mret
#   5. main 看 s0=1 → 跳出 wait_loop → halt path
#
# trace 期望 (debug.h 互斥协议): 'cf' × N (主 setup 阶段 + wait_loop 多轮 check no fire)
# → 'tf' (timer fire + refetch from mtvec) → 'cf' (handler 内 MIE=0 deliver_mask=0)
# → 'Ef' × 3 (halt path ecall triple fault).

.section .text
.option norvc
.globl _start
_start:
    # ---- M-mode 起步 ----
    # 设 mtvec = m_handler (mode=0 direct; D9 解绑后 mode=1 vectored 也支持, 但这条
    # fixture 用 direct, vectored 形态由 fixture 之外的内核单测覆盖)
    la      t0, m_handler
    csrw    mtvec, t0

    # mie.MTIE = (1<<7) = 0x80
    li      t0, 0x80
    csrw    mie, t0

    # mtimecmp[0] = 50 (小值, mtime 推进几轮后超过, MTIP fire)
    li      t0, 0x02004000          # CLINT mtimecmp[0] 低 32
    li      t1, 50
    sw      t1, 0(t0)
    sw      x0, 4(t0)               # 高 32 = 0

    # mstatus.MIE = 1 (bit 3); csrrsi 立即数 0x8 = (1<<3)
    csrrsi  x0, mstatus, 0x8

    # 等待 timer interrupt fire 的 busy loop (handler 将设 s0=1 退出)
    li      s0, 0
    li      a5, 0
wait_loop:
    addi    a5, a5, 1               # 累加器 (验 wait_loop 真跑了几轮)
    beqz    s0, wait_loop

    # ---- handler 返回后 ----
    mv      a0, s1                  # a0 = handler 保存的 mcause (期望 0x80000007)

    # ---- halt: 清 mtvec + ecall 触发 triple fault ----
    csrw    mtvec, x0
    ecall

m_handler:
    # 保存 mcause 待 main 验证 (期望 0x80000007 = CAUSE_INTERRUPT_BIT | IRQ_M_TIMER)
    csrr    s1, mcause

    # 清 MTIP via 写 mtimecmp = UINT64_MAX (跟 clint_init sentinel 一致)
    li      t0, 0x02004000
    li      t1, -1                  # 0xFFFFFFFF
    sw      t1, 0(t0)
    sw      t1, 4(t0)

    li      s0, 1                   # 完成标志
    mret

# 期望最终寄存器:
#   a0 (x10) = 0x80000007   (handler s1 → main a0; CAUSE_INTERRUPT_BIT | IRQ_M_TIMER)
#   s0 (x8)  = 1            (handler 设置, 验证 handler 真跑)
#   s1 (x9)  = 0x80000007   (handler 保存 mcause)
#   a5 (x15) > 0            (wait_loop 至少跑一轮)
#
# 期望 trap dump:
#   in_trap = 3             (halt path triple fault)
#   mcause  = 1             (CAUSE_INST_ACCESS_FAULT 第 2 次保留; 路径:
#                            ecall_from_M (cause 11, in_trap=1) → mtvec=0 → mmu fail pc=0
#                            (cause 1, in_trap=2) → mmu fail again (cause 1, in_trap=3 早 return))
