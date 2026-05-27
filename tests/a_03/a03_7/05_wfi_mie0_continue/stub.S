# a03_7 / 05_wfi_mie0_continue
#
# ===== 测试目的 =====
# 验 RV Priv Spec §3.3.2 WFI 唤醒语义: 唤醒条件 = (mie & mip) != 0, 跟 mstatus.MIE
# 无关. mstatus.MIE=0 时 WFI 仍能被中断唤醒, 但醒后不 trap, 继续 PC+4 顺序执行
# (典型用法: 关全局中断后 WFI 等事件, 软件醒来自己 poll mip).
#
# 跟 01_wfi_timer_wake 区别:
#   - 01: mstatus.MIE=1 + MTIE=1 → wake 后立即 trap to handler (cause=M_TIMER)
#   - 05: mstatus.MIE=0 + MTIE=1 → wake 但不 trap, PC+=4 继续, 用户态 poll mip
#
# 这是 015 关键 spec compliance 点 — wfi_should_wake 内 (csr_mip_read & _mie) !=0
# 决定, 跟 mstatus.MIE 无关. 跟 trap_check_interrupt 内的 "MIE 控制是否 deliver"
# 是两件事.
#
# 单 hart + 真异步 wake (timer thread 推 mtime) — 跟 01 同体例; 区别只是 MIE.
#
# trace 期望: '_._._._.w(sleep gap)._._.' — 'w' 后无 't' (没 trap fire), '.' 是
# 醒来后 trap_check_interrupt ready==0 (mie&mip 非 0 但 MIE=0 不 deliver).
#
# ===== 期望结果 =====
#   s0 = 0x80 (post-wfi 读 mip, 期望 MTIP bit 7 = 0x80)
#   sifive_test FINISHER PASS

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
    # ---- Boot prelude (清 MDT/SDT, 否则 MIE 写不进 — 虽然本测我们故意不开 MIE, 但
    #      prelude 保证 csr 写一致行为) ----
    csrw    0x310, x0
    csrw    sstatus, x0

    # ---- mie = MTIE (per-source 开) ----
    li      t0, 0x80
    csrw    mie, t0

    # ---- mstatus.MIE 故意保 0 (boot prelude 后 mstatus 已基本清 0) ----
    # 不调 csrrsi mstatus, MIE
    # 不设 mtvec (反正不 trap)

    # ---- mtimecmp = 500 (短 tick, timer thread 几次 wake 内必到) ----
    li      t0, CLINT_BASE + CLINT_MTIMECMP_OFF
    li      t1, 500
    sw      t1, 0(t0)
    sw      x0, 4(t0)

    li      s0, 0

    # ---- WFI: 真挂起 (mip.MTIP 初 0, predicate false → cond_wait); timer thread
    # tick 推 mtime 过 500 → mtip 0→1 → wfi_kick(0) → wake.
    # 醒后 mstatus.MIE=0 不 trap, PC+=4 顺序继续到下方 csrr mip.
    # trace 期望 'w' (真睡 — gap visible in trace stream as adjacent 字符 但实际有
    # ~ms 级延迟 between).
    wfi

    # 醒来后顺序执行 (no trap). 读 mip 验 MTIP=1.
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
