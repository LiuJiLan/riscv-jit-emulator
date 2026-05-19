# a02_6 / 01_clint_mtime_async
# 验 timer 辅助线程真异步累加 atomic clint.mtime:
#   1. 读 CLINT mtime 低 32 (MMIO @ 0x0200BFF8) → t0
#   2. poll 等 mtime 值变 (max 10M iter 防卡死) → t1
#   3. 验 t1 > t0 → s0=1 success / 否则 s0=0xBAD fail
#   4. ecall → mtvec=0 triple fault halt
#
# 期望 (1ms 一次 fetch_add += TIMEBASE_PER_WAKE=10000):
#   host CPU ~ns 量级跑 poll, mtime 真异步推进; 几 ms host wall clock 后 t1 必 > t0,
#   delta 大概率 ~10000 量级 (一次 wake) 或更多 (跨多个 wake; host schedule 抖动).
#
# 本 fixture poll loop 内只有 lw + beq, 没指令数依赖 — 如果 timer thread 不真跑,
# poll 永远 stuck 直到 iteration 耗尽 → fail (s0=0xBAD)。
#
# trace 期望 (debug.h 互斥协议): 'f' 个数取决于 poll 多少轮才看到变化 (各 block
# 结束 boundary 再 refetch 一次), 末段 'Ef' × 3 (halt path ecall triple fault).

.section .text
.option norvc
.globl _start
_start:
    # CLINT mtime 地址 (config.h CLINT_BASE=0x02000000 + CLINT_MTIME_OFF=0xBFF8)
    li      t6, 0x0200BFF8

    # 第一次读 mtime 低 32 → t0
    lw      t0, 0(t6)

    # 防御性初值: t1=t0 让 fail 路径下 dump 看着干净 (a1=a0 表示从未观察到变化)
    mv      t1, t0

    # poll loop: 等 mtime 变化, max iterations 10M 防卡死
    li      s2, 10000000             # max iterations (host CPU 几 ms; 充分跨 1ms timer wake)
poll_loop:
    addi    s2, s2, -1
    beqz    s2, fail                # iteration 耗尽 → timer thread 没真跑 = fail
    lw      t1, 0(t6)
    beq     t1, t0, poll_loop       # mtime 没变继续 poll

    # mtime 变了, 走到这说明 timer thread 真异步推进了。
    bltu    t1, t0, fail            # 防御: t1 < t0 (溢出 / 时间倒流) 也算 fail
    sub     a5, t1, t0              # a5 = delta (期望 > 0)
    mv      a0, t0                  # a0 = mtime t0 (起点)
    mv      a1, t1                  # a1 = mtime t1 (终点)
    li      s0, 1                   # s0=1 success
    j       halt

fail:
    li      s0, 0xBAD               # s0=0xBAD fail
    mv      a0, t0                  # a0 = mtime t0
    mv      a1, t1                  # a1 = mtime t1 (poll 卡死时 t1 == t0)
    li      a5, 0                   # a5 = 0 (没观察到 delta)

halt:
    csrw    mtvec, x0               # 清 mtvec, ecall 走 pc=0 → mmu fail triple fault
    ecall

# 期望最终寄存器:
#   s0 (x8)  = 1            (success)
#   a0 (x10) = mtime t0     (一般 ~几千; cpu_create / clint_init 到 _start 已过几 us)
#   a1 (x11) = mtime t1     (> a0)
#   a5 (x15) > 0            (delta; 一般 ~10000 量级或更大, 取决于 host schedule)
#
# 期望 trap dump:
#   in_trap = 3             (halt path triple fault)
#   mcause  = 1             (CAUSE_INST_ACCESS_FAULT 第 2 次保留; 跟 a02_5 同形态)
