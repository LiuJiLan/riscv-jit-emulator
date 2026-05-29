# a02_4 / 06_mip_mtip_via_mtime
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# mip.MTIP 合成路径验证 — clint_timer_pending 内 (mtime ≥ mtimecmp[hartid]) 派生.
# 验证 clint_init mtimecmp 初值 UINT64_MAX (不是 0; 否则 0>=0 → spurious MTIP).
# RV Priv Spec §3.2.1 `>=` 边界 (mtime==mtimecmp 仍 pending).
# (注: 改 test_dev 停机, 旧叙事见 REVIEW B.3)
#
# ===== 思路 =====
# **重要**: timer 辅助线程异步累加 mtime 后, guest sw 写 mtime=100 是瞬态值; 任一
# wake (~1ms 一次) 都会 fetch_add += TIMEBASE_PER_WAKE(=10000) 把 guest 写的值
# clobber 掉。本 fixture 的核心验证目标 (clint_init mtimecmp 初值 + csrr mip
# 合成路径 + WARL) 不依赖具体 mtime 数值, 只看 MTIP bit 是否按 spec 语义 set。
#
# BARE M-mode.
# CLINT mtime     @ 0x0200BFF8 (低 32) / 0x0200BFFC (高 32)
# CLINT mtimecmp[0] @ 0x02004000 (低 32) / 0x02004004 (高 32)

.section .text
.option norvc
.globl _start
_start:
    # --- 1. 起步 csrr mip → MTIP=0 (clint_init mtimecmp=UINT64_MAX) ---
    csrr    a0, mip                               # a0 = 0

    # --- 2. 写 mtime = 100 (mtimecmp 仍 UINT64_MAX) ---
    li      t0, 0x0200BFF8                        # mtime 低 32 地址
    li      t1, 100
    sw      t1, 0(t0)                             # mtime[31:0] = 100
    sw      x0, 4(t0)                             # mtime[63:32] = 0
    csrr    a1, mip                               # a1 = 0 (100 < UINT64_MAX)

    # --- 3. 写 mtimecmp[0] = 50 → MTIP=1 ---
    li      t0, 0x02004000                        # mtimecmp[0] 低 32 地址
    li      t1, 50
    sw      t1, 0(t0)                             # mtimecmp[31:0] = 50
    sw      x0, 4(t0)                             # mtimecmp[63:32] = 0
    csrr    a2, mip                               # a2 = 0x80 (MTIP, 1<<IRQ_M_TIMER)

    # --- 4. 写 mtimecmp[0] = 200 → MTIP=0 ---
    li      t1, 200
    sw      t1, 0(t0)                             # mtimecmp[31:0] = 200
    csrr    a3, mip                               # a3 = 0

    # --- 5. 边界 `>=` 测试: mtimecmp = mtime = 100 → MTIP=1 (spec 明确 >=) ---
    li      t1, 100
    sw      t1, 0(t0)                             # mtimecmp[31:0] = 100
    csrr    a4, mip                               # a4 = 0x80

    # ---- 停机: test_dev FINISHER PASS ----
    li      x30, 0x100000                         # test_dev base
    li      x31, 0x5555                           # FINISHER PASS magic
    sw      x31, 0(x30)                            # 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望寄存器 a3 不严格确定 (取决于 timer wake 时机):
#   - 没 wake 跨过本 fixture 期间: mtime ≈ 100, mtimecmp=200 → MTIP=0 (a3=0)
#   - 有 wake 跨过: mtime ≥ 10000+100, mtimecmp=200 → MTIP=1 (a3=0x80)
# a2 / a4 永远 0x80 (mtimecmp <= guest 写的 100 或 timer 累加值, MTIP 必 set).
#
# 期望最终寄存器:
#   a0 (x10) = 0x00000000   起步: mtime=N, mtimecmp=UINT64_MAX → MTIP=0 (验初值修复)
#   a1 (x11) = 0x00000000   写 mtime=100, mtimecmp 仍 UINT64_MAX → MTIP=0 (∀ mtime<MAX)
#   a2 (x12) = 0x00000080   写 mtimecmp=50,  mtime≥50  → MTIP=1 (1<<7)
#   a3 (x13) = 0 or 0x80    写 mtimecmp=200, MTIP 取决 wake 时机 (见上)
#   a4 (x14) = 0x00000080   写 mtimecmp=100, mtime≥100 → MTIP=1
#   x30 = 0x100000 / x31 = 0x5555  (停机 scratch)
#   停机 = test_dev FINISHER PASS → exit 0
#
# 长期 TODO: 真要严格验 a3=0 要把 fixture 拆成 "短时间窗口内串完 step 3-4-5"
# 或用 csrw 直接屏蔽 MTIE 测合成读 (而不是 sw mtimecmp).
