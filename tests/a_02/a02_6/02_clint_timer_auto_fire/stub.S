# a02_6 / 02_clint_timer_auto_fire
# 验 T5 timer 辅助线程真异步累加 mtime → mtimecmp 越过阈值 → MTIP → trap deliver
# 端到端 — 跟 a02_5/01 同形态, 区别在 mtimecmp 设大点 (跨多个 timer wake), 体现
# "mtime 跟 host wall clock 推进 / 不靠 dispatcher 指令数".
#
# 流程:
#   1. mtvec = m_handler (direct mode)
#   2. mie.MTIE = 1
#   3. mtimecmp[0] = 100000 (= 10ms 后 fire; 当前配置 TIMEBASE_PER_WAKE=10000 /
#      TIMER_WAKE_INTERVAL_NS=1000000 = 1ms, 跨 10 次 timer wake)
#   4. mstatus.MIE = 1
#   5. busy wait_loop (s0=0 时 spin); host wall clock ~10ms 后 timer fire
#   6. trap_check_interrupt → fire (mip.MTIP & mie.MTIE & mstatus.MIE)
#   7. trap_set_interrupt_state: cause = 0x80000007, jump-to mtvec
#   8. m_handler: 保存 mcause, 清 MTIP via mtimecmp=UINT64_MAX, s0=1, mret
#   9. main 看 s0=1 → 跳出 wait_loop → halt
#
# 跟 a02_5/01 区别:
#   - a02_5/01 mtimecmp=50: T3 setter 时代 dispatcher 主帧 50 个指令就 fire; T5
#     真异步下也能 fire 但只是"刚好第一个 wake 就跨过", 没充分体现异步
#   - 本 fixture mtimecmp=100000: 必须跨 10 次 timer wake, dispatcher 主帧 wait_loop
#     跑大量循环 (a5 累加器 > 0 显著体现); 唯一驱动 mtime 推进的是 timer thread
#     的 atomic_fetch_add, 跟 dispatcher 指令数完全解耦

.section .text
.option norvc
.globl _start
_start:
    # ---- M-mode 起步 ----
    la      t0, m_handler
    csrw    mtvec, t0

    # mie.MTIE = (1<<7) = 0x80
    li      t0, 0x80
    csrw    mie, t0

    # mtimecmp[0] = 100000 (= 10 ms after; 跨多个 timer wake)
    li      t0, 0x02004000          # CLINT mtimecmp[0] 低 32 (CLINT_BASE + 0x4000)
    li      t1, 100000
    sw      t1, 0(t0)
    sw      x0, 4(t0)               # 高 32 = 0

    # mstatus.MIE = 1 (bit 3)
    csrrsi  x0, mstatus, 0x8

    # busy wait_loop: spin 直到 handler 设 s0=1
    li      s0, 0
    li      a5, 0
wait_loop:
    addi    a5, a5, 1               # 累加器验 wait_loop 真跑了多少轮 (期望 >> a02_5/01)
    beqz    s0, wait_loop

    # ---- handler 返回后 halt ----
    mv      a0, s1                  # a0 = handler 保存的 mcause (期望 0x80000007)

    csrw    mtvec, x0               # 清 mtvec
    ecall                            # triple fault halt

m_handler:
    # 保存 mcause (期望 0x80000007 = CAUSE_INTERRUPT_BIT | IRQ_M_TIMER)
    csrr    s1, mcause

    # 清 MTIP via mtimecmp = UINT64_MAX (跟 clint_init / clint_reset sentinel 一致)
    li      t0, 0x02004000
    li      t1, -1                  # 0xFFFFFFFF
    sw      t1, 0(t0)
    sw      t1, 4(t0)

    li      s0, 1                   # 完成标志
    mret

# 期望最终寄存器:
#   a0 (x10) = 0x80000007   (handler s1 → main a0; CAUSE_INTERRUPT_BIT | IRQ_M_TIMER)
#   s0 (x8)  = 1            (handler 设, 验 handler 真跑)
#   s1 (x9)  = 0x80000007   (handler 保存 mcause)
#   a5 (x15) >> 0           (wait_loop 跑大量轮; 不像 a02_5/01 只跑几十轮)
#
# 期望 trap dump:
#   in_trap = 3             (halt path triple fault, 跟 a02_5/01 同形态)
#   mcause  = 1             (ecall → mtvec=0 → mmu fail; 跟 a02_5/01 同)
