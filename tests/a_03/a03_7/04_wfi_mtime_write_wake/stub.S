# a03_7 / 04_wfi_mtime_write_wake
#
# ===== 测试目的 =====
# 验 015 clint_write mtime 路径接的 recompute_all_mtip + wfi_kick 钩子 (跟
# clint_write mtimecmps 同 design — 之前漏的 path, 015 补的).
#
# 单 hart 局限: hart 在 WFI 时没法自己写 mtime; 异步 wake "guest 写 mtime 唤醒
# sleeping WFI" 需 SMP 第二 hart 触发. 本 fixture 退化验:
#   1. guest 写 mtime 巨值 (> mtimecmp), 触发 clint_write mtime → recompute_all_mtip
#      → mtip 0→1 → wfi_kick(0) (hart 不在 WFI, kick 是 noop)
#   2. 紧接 WFI (同 block 内, sw 非 boundary): predicate 看 mip.MTIP=1 → 立即返
#   3. 不开 mstatus.MIE — wake 后不 trap, 继续读 mip 验 MTIP=1 → PASS
#
# 跟 01_wfi_timer_wake 区别:
#   - 01: timer thread tick async 推 mtime → mtip 0→1, hart 真在 cond_wait 被 kick
#   - 04: guest sw mtime 自己推, hart 自醒 (predicate 进 wfi_wait 就 true), 验
#     clint_write mtime path 而不是 timer thread path
#
# 实装路径 (clint.c clint_write off==CLINT_MTIME_OFF):
#   atomic_store(clint.mtime, nxt, relaxed)
#   clint_recompute_all_mtip()      # 015 补的钩子; 内 for-loop 调 recompute_mtip
#
# trace 期望 '_._..._w._.' (w 后无 't', 因 MIE=0 不 trap; 继续顺序执行).
#
# ===== 期望结果 =====
#   s0 = 0x80 (post-wfi 读 mip.MTIP, 期望 MTIP bit 7 = 0x80)
#   sifive_test FINISHER PASS

.equ CLINT_BASE,            0x02000000
.equ CLINT_MTIMECMP_OFF,    0x4000
.equ CLINT_MTIME_OFF,       0xBFF8
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

    # ---- mie = MTIE (per-source 开; mstatus.MIE 故意不开, wake-without-trap 路径) ----
    li      t0, 0x80
    csrw    mie, t0

    # ---- mtimecmp = 0x100000000 (4G, mtime 几乎不可能短时间过) ----
    # 写两个 32-bit 半 (low + high). bo bit 2 = 0 写 low 半, = 4 写 high 半.
    li      t0, CLINT_BASE + CLINT_MTIMECMP_OFF
    li      t1, 0                       # mtimecmp low = 0
    sw      t1, 0(t0)
    li      t1, 1                       # mtimecmp high = 1 → 总值 = 0x100000000
    sw      t1, 4(t0)

    li      s0, 0                       # 后面填 post-wfi mip 值

    # ---- 关键 block: write mtime LARGE + WFI 同 block (sw 非 boundary) ----
    # 写 mtime = 0x200000000 (8G); clint_write mtime path 触发 recompute_all_mtip
    # → mtip 0→1 (8G >= 4G). wfi_kick(0) 调 (无 waiter, noop).
    # 紧接 WFI: predicate (mie & mip)=MTIE & MTIP=0x80 → 立即返不挂起.
    li      t0, CLINT_BASE + CLINT_MTIME_OFF
    li      t1, 0                       # mtime low = 0
    sw      t1, 0(t0)
    li      t1, 2                       # mtime high = 2 → 总值 = 0x200000000
    sw      t1, 4(t0)
    wfi

    # post-wfi: 读 mip 验 MTIP=1 (= 0x80). MIE=0 没 trap, 这里继续顺序执行.
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
