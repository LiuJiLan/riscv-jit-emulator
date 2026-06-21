# b_03_2 / 02_fencei_lrsc_clear
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验 b_03 T2 fence_i_helper 副作用 lrsc_clear_self 仍 work — T2 fence_i_helper 函数
# 体不动 (audit R15 同精神, 仍是 `lrsc_clear_self(hart)`), 但加 fixture 锁住 b_03
# 范围内不退化. RV spec 七类清除 #4 (fence.i): 任何 fence.i 调用清当前 hart reservation.
#
# 路径 walk through:
#   1. li t1, 0xCAFE0001 + sw 0(s0) — 设初值
#   2. lr.w t3, (s0) — 读 + 建 reservation @ s0 (PA = SCRATCH)
#   3. fence.i — *** 关键: lrsc_clear_self 清 self reservation INVALID ***
#   4. sc.w t6, t2, (s0) — 写; 应失败 rd=1 (reservation 已被 fence.i 清)
#   5. lw t5, 0(s0) — verify mem 没改 (仍是 0xCAFE0001, SC fail 不写)
#
# 任一子测失败走 fail handler → exit_code = FINISHER_FAIL (0x3333).
#
# 单 hart, M-mode (REGIME_BARE; LR/SC 走 lrsc_lr_helper / lrsc_sc_helper BARE 路径).
# Boot prelude 清 MDT/SDT 防 Smdbltrp.

.equ TEST_DEV_BASE,     0x00100000
.equ FINISHER_PASS,     0x5555
.equ FINISHER_FAIL,     0x3333

.equ SCRATCH,           0x80100000

.section .text
.option norvc
.globl _start

_start:
    # ---- Boot prelude ----
    csrw    0x310, x0                   # mstatush 清 MDT
    csrw    sstatus, x0                 # 清 SDT

    li      s0, SCRATCH                 # 工作地址 (固定 4-byte aligned)

    # ============================================================
    # Test: LR + fence.i + SC → 期望 SC fail (rd=1, mem 不变)
    # ============================================================
    li      t1, 0xCAFE0001
    sw      t1, 0(s0)                   # 初值
    lr.w    t3, (s0)                    # rd = 0xCAFE0001, 建 reservation @ s0
    li      t4, 0xCAFE0001
    bne     t3, t4, fail                # LR 读值验证

    # *** 关键: fence.i 副作用清当前 hart reservation (lrsc_clear_self) ***
    # T2 不动 fence_i_helper 函数体 (仍是 lrsc_clear_self), 本 fixture 锁住此副作用.
    fence.i

    li      t2, 0xDEADBEEF
    sc.w    t6, t2, (s0)                # 应失败 rd=1 (reservation 被 fence.i 清)
    li      t4, 1
    bne     t6, t4, fail                # rd != 1 → fail (SC 误成功)

    lw      t5, 0(s0)                   # 验 mem 没改 (仍是 0xCAFE0001)
    li      t4, 0xCAFE0001
    bne     t5, t4, fail                # mem 变 → fail (SC fail 仍写了 BUG)

    # ---- PASS ----
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_PASS
    sw      t1, 0(t0)
1:  j       1b

fail:
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_FAIL
    sw      t1, 0(t0)
2:  j       2b

# ===== 期望结果 =====
#   exit_code = 0 (FINISHER_PASS)
#   reg dump (cpu_destroy):
#     t3 (x28) = 0xCAFE0001  (LR 读值)
#     t6 (x31) = 1           ← 关键: SC fail (reservation 被 fence.i 清)
#                              (若 = 0 → fence.i 没清 reservation BUG)
#     t5 (x29) = 0xCAFE0001  (SC fail 不写 mem)
#     priv = 3 (M-mode 全程)
#
# 不期望:
#   - SC 误成功 (rd=0): fence_i_helper 函数体 lrsc_clear_self 丢失 → T2 退化 BUG
#   - mem 变 0xDEADBEEF: SC fail 路径错误写 mem → lrsc.c BUG
#   - trap (cause 6/7): LR/SC misalign or access fault → LR/SC decode 漏 BUG
