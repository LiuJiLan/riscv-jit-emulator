# a02_5 / 06_int_priority
#
# ===== 测试目的 =====
# 中断优先级 (RV Priv Spec §3.1.9 + trap_check_interrupt 优先级 encoder):
#   优先级: M_EXT > M_SOFT > M_TIMER > S_EXT > S_SOFT > S_TIMER
# Fixture 同时让 MSIP + MTIP pending, 期望 mcause = 0x80000003 (M_SOFT, 优先 M_TIMER)。
#
# ===== 思路 =====
#   1. M setup: mtvec=m_handler, mie = MSIE | MTIE = 0x88, mstatus.MIE = 1
#   2. 设 mtimecmp[0] = 5 (small, 立即 pending after few iterations)
#   3. 设 CLINT.msip[0] = 1 (MSIP pending)
#   4. busy loop 让 mtime 步进过 mtimecmp → MTIP=1; 此时 MTIP + MSIP 都 pending + enabled
#   5. trap_check_interrupt 按优先级 encoder 选 M_SOFT (bit 3 > bit 7 in priority order)
#   6. m_handler 验 mcause = 0x80000003; 清 msip + 把 mtimecmp 设回 UINT64_MAX 清 MTIP
#   7. mret 后 halt path

.section .text
.option norvc
.globl _start
_start:
    la      t0, m_handler
    csrw    mtvec, t0

    # mie = MSIE | MTIE = 0x8 | 0x80 = 0x88
    li      t0, 0x88
    csrw    mie, t0

    # mtimecmp[0] = 5 (小值, 自旋几轮立即超过)
    li      t0, 0x02004000
    li      t1, 5
    sw      t1, 0(t0)
    sw      x0, 4(t0)

    # CLINT.msip[0] = 1
    li      t0, 0x02000000
    li      t1, 1
    sw      t1, 0(t0)

    # init s0/a5 必须在 csrrsi mstatus,0x8 (开 MIE) 之前 — 因为 mtimecmp=5 + msip=1 都已
    # pending, MIE 一开就 fire. fire 时 xepc = 下一条指令 addr; 若 init 在 csrrsi 后, mret
    # 会跳回 init 段, s0 被 reset 为 0 → wait_loop 死循 (跟 fixture 04/05 同模式).
    li      s0, 0
    li      a5, 0

    # mstatus.MIE = 1 (一开就 fire; xepc 落在 wait_loop 内, mret 不重置 s0)
    csrrsi  x0, mstatus, 0x8

wait_loop:
    addi    a5, a5, 1
    beqz    s0, wait_loop

    mv      a0, s1               # a0 = scause? 不, mcause; 期望 0x80000003 (M_SOFT 优先)

    csrw    mtvec, x0
    ecall                        # halt

m_handler:
    csrr    s1, mcause           # 期望 0x80000003 (CAUSE_INTERRUPT_BIT | IRQ_M_SOFT;
                                  # 优先 M_TIMER=0x80000007 by priority encoder)

    # 清 msip[0] = 0 (清 MSIP 异步源)
    li      t0, 0x02000000
    sw      x0, 0(t0)

    # 清 MTIP via mtimecmp = UINT64_MAX
    li      t0, 0x02004000
    li      t1, -1
    sw      t1, 0(t0)
    sw      t1, 4(t0)

    li      s0, 1
    mret

# ===== 期望结果 =====
# 期望最终寄存器:
#   a0 = 0x80000003   (M_SOFT 优先于 M_TIMER; priority encoder 选 IRQ_M_SOFT)
#   s0 = 1
#   s1 = 0x80000003
#   a5 > 0
#
# 关键验证: 即使 MTIP 跟 MSIP 同时 pending (mtime >= mtimecmp 跟 msip=1 都成立), trap_check_
# interrupt 内 encoder 按 §3.1.9 选 M_SOFT (bit 3) 而不是 M_TIMER (bit 7).
#
# 注: fixture 依赖 timer 辅助线程异步累加 mtime, 第一次 wake 就推进过 mtimecmp=5
# (实现方式见 platform/clint.h 顶段).
