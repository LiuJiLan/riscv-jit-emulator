# a03_7 / 06_wfi_timeout_resleep
#
# ===== 测试目的 =====
# 验 015 wfi_wait 内 while loop + cond_timedwait timeout 兜底 (a_03_wfi_fixture_result.md
# 不变式 4): mtimecmp 比 mtime 高很多 → WFI 进 cond_wait → 500ms timeout 自醒 →
# predicate 重 check 仍 false (mtime 还没到 mtimecmp) → 再进 cond_wait → 'S' 多打一次.
# 最后 timer thread 推 mtime 越过 mtimecmp → recompute_mtip 0→1 → wfi_kick →
# cond_timedwait 真醒 → predicate true → 退 wfi_wait → 'W'.
#
# trace 期望 'wSSSSW' (4 S; 取决于 timer thread 跟 cond_timedwait 相对时序, 实际可能
# wSSSW (3 S) 或 wSSSSW (4 S)):
#   - WFI_TIMEOUT_NS = 500ms → 单次 cond_timedwait ≤ 0.5s
#   - mtimecmp - mtime ≈ 2s 真实时间 = TIMEBASE_FREQ_HZ * 2 = 20M ticks
#   - 2s / 500ms = 4 cond_timedwait 周期; 最后一次被 kick 中断早醒
#   - 起始 mtime 已经被 timer thread 推过几次 (fixture 启动 ~ms 级), 所以 4 周期估值是
#     上限, 实际可能 3 个完整 timeout + 1 个 kick 中断 = 'wSSSW' (3 S)
#
# 跟 01_wfi_timer_wake 区别:
#   01: mtimecmp=500 极小, timer 第一次 tick 就推过 (没等到 500ms timeout), kick 立即,
#       trace 'wSW' (单 S — 真睡一次但被 kick 立刻醒)
#   06: mtimecmp=20M (2s), 必经多次 timeout 重 check, 验 while loop 跟 timeout 防御自愈
#
# 不开 mstatus.MIE — 醒后不走 trap, 直接 fall-through 验 mip.MTIP=1 (跟 05 同体例)

.equ CLINT_BASE,            0x02000000
.equ CLINT_MTIMECMP_OFF,    0x4000
.equ TEST_DEV_BASE,         0x00100000
.equ TEST_DEV_SIFIVE_OFF,   0x00
.equ FINISHER_PASS,         0x5555
.equ FINISHER_FAIL,         0x3333

.section .text
.option norvc
.globl _start

_start:
    # ---- Boot prelude ----
    csrw    0x310, x0
    csrw    sstatus, x0

    # ---- mie = MTIE (mstatus.MIE 故意不开, wake-no-trap 模式) ----
    li      t0, 0x80
    csrw    mie, t0

    # ---- mtimecmp = 20_000_000 (低 32 bits; 高 32 bits = 0) ----
    # 在 TIMEBASE_FREQ_HZ=10MHz 下相当 2 秒 mtime tick. mtime 起始接近 0 (clint_init=0,
    # fixture 启动到此可能涨几万 tick), 距 20M 远, mip.MTIP 一开始一定 = 0.
    li      t0, CLINT_BASE + CLINT_MTIMECMP_OFF
    li      t1, 20000000
    sw      t1, 0(t0)
    sw      x0, 4(t0)

    li      s0, 0

    # ---- WFI: 必经多次 cond_timedwait timeout 自愈, 直到 mtime 越过 mtimecmp ----
    wfi

    # 醒来后 (MIE=0 无 trap, 直接顺序) 验 mip.MTIP=1
    csrr    s0, mip
    li      t1, 0x80
    and     t2, s0, t1
    beqz    t2, fail

success:
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_PASS
    sw      t1, TEST_DEV_SIFIVE_OFF(t0)
    j       .

fail:
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_FAIL
    sw      t1, TEST_DEV_SIFIVE_OFF(t0)
    j       .
