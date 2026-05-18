# a02_4 / 06_mip_mtip_via_mtime
# mip.MTIP 合成路径验证 — clint_timer_pending 内 (mtime ≥ mtimecmp[hartid]) 派生.
# 验证 clint_init mtimecmp 初值 UINT64_MAX (不是 0; 否则 0>=0 → spurious MTIP).
# RV Priv Spec §3.2.1 `>=` 边界 (mtime==mtimecmp 仍 pending).
#
# **本 fixture 假设 mtime 不主动步进** — guest sw 写多少, csrr 读多少, 跨 block 保留.
#   T2 阶段 (T3+T4 前) 这个假设成立, 验证 csrr mip 合成路径 + mtimecmp WARL + `>=` 边界
#   语义都不需要 mtime 真步进.
#
#   一旦后续 milestone 实现"真步进 mtime" (跟 RV Priv Spec §3.2.1 rtc_toggle 异步累加
#   语义对齐), 本 fixture 的 a2 / a4 期望可能变化 (具体 0x80 / 0 取决于步进时机跟 sw
#   写入的相对顺序). 不同实现方式 (dispatcher 主帧同步推进 / 独立 timer 辅助线程异步累
#   加) 行为细节会不同, 但 csrr mip 合成读语义本身不变. 真步进上线后:
#     - 如仍想保持本 fixture 的 a2/a4 = 0x80 期望, 需要把"mtime 步进"形态改成 fetch_add
#       (累加而不是 reset), 让 guest sw mtime 写入保留.
#     - 或接受 a2/a4 变 0, 把 "mtime ≥ mtimecmp 边界" 测试迁到能控制 mtime 步进的更端
#       到端 fixture (例如 timer interrupt 真触发的 a02_5/01).
#
# BARE M-mode.
# CLINT mtime     @ 0x0200BFF8 (低 32) / 0x0200BFFC (高 32)
# CLINT mtimecmp[0] @ 0x02004000 (低 32) / 0x02004004 (高 32)
#
# 期望最终寄存器 (mtime 不主动步进的前提下):
#   a0 (x10) = 0x00000000   起步: mtime=0, mtimecmp=UINT64_MAX → MTIP=0 (验初值修复)
#   a1 (x11) = 0x00000000   写 mtime=100, mtimecmp 仍 UINT64_MAX → MTIP=0 (100<MAX)
#   a2 (x12) = 0x00000080   写 mtimecmp=50, mtime=100 → 100>=50 → MTIP=1 (1<<7)
#   a3 (x13) = 0x00000000   写 mtimecmp=200, mtime=100 → 100<200 → MTIP=0
#   a4 (x14) = 0x00000080   写 mtimecmp=100 (==mtime), `>=` 边界 → MTIP=1
#
# halt: ecall.

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

    ecall
