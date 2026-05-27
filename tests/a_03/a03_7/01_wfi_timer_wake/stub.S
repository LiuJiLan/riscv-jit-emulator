# a03_7 / 01_wfi_timer_wake
#
# ===== 测试目的 =====
# WFI 真实装 (015) — 验证三件事:
#   1. WFI 在 mip 当前无 pending 时不立即返回 (cond_wait 阻塞 hart 线程, 不是
#      NOP 立返); hart 醒来必须是 CLINT timer thread tick → mtime ≥ mtimecmp
#      → clint_recompute_mtip → wfi_kick 唤醒
#   2. 醒来后 mstatus.MIE=1 + MTIE=1 → trap_check_interrupt 派发 → mtvec 入口
#      handler 执行 (cause = 0x80000007 = M_TIMER)
#   3. handler 清 MTIP (mtimecmp = UINT64_MAX) + 写 sifive_test PASS magic 退出
#
# 顺路验:
#   - MTIP cache (clint_per_hart_slot_t.mtip) 跟 timer thread 同步刷, 不
#     spurious wake (mtimecmp 设大值 → mtip=0 → WFI 真睡)
#   - wfi_kick 在 mtip 0→1 翻转准确触发 (timer thread 第 N 次 tick 时)
#
# 类 a03_2/05_int_priority 那种 setup, 但只 1 个 source (MTIP) + 多走一个 wfi 指令。
#
# ===== 期望结果 =====
#   t0 = 0x80000007 (mcause = M_TIMER interrupt; 1<<31 | 7)
#   s0 = 1 (handler 成功标志)
#   sifive_test FINISHER 写 PASS=0x5555 → emulator exit code 0
#
# 失败模式:
#   - WFI 立即返回不挂起 → handler 不进, ecall 直触发 cause 11 (M_ECALL_FROM_M)
#     → fixture 跑过 ecall 直接 halt, t0/s0 都是 0
#   - WFI 挂起但 timer 不来 → emulator hang 在 wfi_wait cond_timedwait,
#     500ms timeout 后 predicate 重 check 仍 false 又睡回去 → 测试超时 (run_tests.py
#     wall-clock cap)
#   - wfi_kick 漏调 → 同 timer 不来 (实际 timer 来了但 hart 不醒)

.equ CLINT_BASE,         0x02000000
.equ CLINT_MTIMECMP_OFF, 0x4000
.equ TEST_DEV_BASE,      0x00100000
.equ TEST_DEV_SIFIVE_OFF,0x00
.equ FINISHER_PASS,      0x5555
.equ FINISHER_FAIL,      0x3333

.section .text
.option norvc
.globl _start

_start:
    # ---- Boot prelude: 清 MDT / SDT 让 MIE/SIE 写得进 ----
    # (Smdbltrp/Ssdbltrp 扩展 reset 后 MDT=SDT=1, csrw mstatus 写 MIE=1 会被
    # 硬件强制 0. small_plan A.7 老 fixture prelude 不全, 新 fixture 必须自己加。)
    csrw    0x310, x0               # mstatush 清 MDT
    csrw    sstatus, x0             # 清 SDT (顺便清 SIE/SPIE)

    # ---- mtvec ----
    la      t0, m_timer_handler
    csrw    mtvec, t0

    # ---- mie = MTIE (bit 7) ----
    li      t0, 0x80
    csrw    mie, t0

    # ---- mideleg = 0 (不 delegate, 走 M handler) ----
    csrw    mideleg, x0

    # ---- mtimecmp[0] = 500 ----
    # TIMEBASE_PER_WAKE 默认配置下, 500 个 tick 在前几次 timer thread wake 内
    # 必到达 (TIMER_WAKE_INTERVAL_NS ~ 1ms, TIMEBASE_PER_WAKE 看 config.h).
    # 设大一点保证 WFI 之前 mtip 还是 0 (避免 _start 顺序 + decode 慢 + 编译路径
    # 不一致导致 mtime 已经过 500 → WFI 立返).
    li      t0, CLINT_BASE + CLINT_MTIMECMP_OFF
    li      t1, 500
    sw      t1, 0(t0)
    sw      x0, 4(t0)

    # ---- counters / flags init ----
    li      s0, 0           # handler 成功标志 (handler 内置 1)
    li      s1, 0           # handler 进入计数 (本测试期望 = 1, 多了表示 spurious)

    # ---- 开 MIE → 之后 WFI 可被中断唤醒并 trap ----
    csrrsi  x0, mstatus, 0x8

    # ---- WFI: 等 timer 来 ----
    # 进 WFI 时 mip.MTIP = 0 (mtimecmp=500, mtime 才几个 tick 不到 500), hart 必挂起;
    # timer thread tick 累加 mtime → 第 N 次 tick 时 mtime ≥ 500 → mtip cache 0→1 →
    # wfi_kick → hart 醒来 → trap_check_interrupt 派发 cause 7 → mtvec → handler
    wfi

    # WFI 醒来后 (handler 已跑过 mret 返回) 跳 success 写 PASS
    j       success

# ============================================================================
# m_timer_handler — MTIP handler: 记 cause + 清 MTIP + 标志位 + mret
# ============================================================================
m_timer_handler:
    csrr    t0, mcause              # t0 = 0x80000007 (cause 7 = M_TIMER)
    addi    s1, s1, 1               # 计数

    # 清 MTIP: mtimecmp = UINT64_MAX (写 -1 到两个 32-bit 字段)
    li      t1, CLINT_BASE + CLINT_MTIMECMP_OFF
    li      t2, -1
    sw      t2, 0(t1)
    sw      t2, 4(t1)

    li      s0, 1                   # handler 成功标志
    mret

# ============================================================================
# success / fail — sifive_test FINISHER 写 magic 退出
# ============================================================================
success:
    # 期望: s0 == 1 且 s1 == 1
    li      t0, 1
    bne     s0, t0, fail
    bne     s1, t0, fail
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_PASS
    sw      t1, TEST_DEV_SIFIVE_OFF(t0)
    j       .                       # halt 在此 (FINISHER 写后 emulator system reset 退出)

fail:
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_FAIL
    sw      t1, TEST_DEV_SIFIVE_OFF(t0)
    j       .
