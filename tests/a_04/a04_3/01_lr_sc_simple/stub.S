# a04_3 / 01_lr_sc_simple
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# T3 Zalrsc 单 hart 基本路径验证. 5 个子测试覆盖:
#   1. LR.W 基本读 + 建 reservation
#   2. SC.W 立即配对 LR (无 cross-hart) → 应成功 (rd=0, mem 更新)
#   3. SC.W 没 LR (上一次 SC 已清 self.res) → 应失败 (rd=1, mem 不变)
#   4. LR + 普通 SW 同 PA → SC 应失败 (lrsc_on_store 清自己 reservation)
#   5. cmpxchg pattern (atomic_inc retry loop) — 单 hart 一次成功
#
# 任一子测失败走 fail handler → exit_code = FINISHER_FAIL (0x3333).
#
# ===== 思路 =====
# 单 hart, M 模式 (REGIME_BARE; LR/SC 走 lrsc_lr_helper / lrsc_sc_helper BARE 路径
# IS_GPA_RAM 命中). Boot prelude 清 MDT/SDT 防 Smdbltrp.
# SCRATCH = 0x80100000 (远离 .text, 跟 a04_2 体例一致).
# 测试值刻意选可识别位模式 (DEAD0001 / CAFE0002 / BAD0BAD0 / 0x11111111 / 0x22222222)
# 防漏 zero-ext / 比较错误.

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
    # Test 1: LR.W 基本读 + 建 reservation
    # ============================================================
    li      t1, 0xDEAD0001
    sw      t1, 0(s0)
    lr.w    t3, (s0)                    # rd = mem (0xDEAD0001), 建 reservation @ s0
    li      t4, 0xDEAD0001
    bne     t3, t4, fail

    # ============================================================
    # Test 2: SC.W 立即配对 LR (无 cross-hart 干扰) → 应成功 (rd=0, mem=value)
    # ============================================================
    li      t2, 0xCAFE0002
    sc.w    t3, t2, (s0)                # 应成功 rd=0
    bnez    t3, fail                    # rd != 0 → fail
    lw      t5, 0(s0)
    li      t4, 0xCAFE0002
    bne     t5, t4, fail                # mem_after != value → fail

    # ============================================================
    # Test 3: SC.W 没 LR (Test 2 SC 已清 self.res INVALID) → 应失败 rd=1
    # ============================================================
    li      t2, 0xBAD0BAD0
    sc.w    t3, t2, (s0)                # 应失败 rd=1
    li      t4, 1
    bne     t3, t4, fail
    lw      t5, 0(s0)                   # 验 mem 没改 (仍是 Test 2 的 0xCAFE0002)
    li      t4, 0xCAFE0002
    bne     t5, t4, fail

    # ============================================================
    # Test 4: LR + 普通 SW 同 PA (lrsc_on_store 扫所有 hart 清, 含自己) + SC → 应失败
    # ============================================================
    li      t1, 0x11111111
    sw      t1, 0(s0)
    lr.w    t3, (s0)                    # 建 reservation
    li      t1, 0x22222222
    sw      t1, 0(s0)                   # 普通 SW 同 PA → lrsc_on_store 清自己 reservation
    li      t2, 0x33333333
    sc.w    t3, t2, (s0)                # 应失败 rd=1
    li      t4, 1
    bne     t3, t4, fail
    lw      t5, 0(s0)                   # 验 mem 仍是 SW 写的 0x22222222 (SC 失败不改 mem)
    li      t4, 0x22222222
    bne     t5, t4, fail

    # ============================================================
    # Test 5: cmpxchg pattern (atomic_inc retry loop) — 单 hart 一次成功
    # ============================================================
    li      t1, 100
    sw      t1, 0(s0)
retry5:
    lr.w    t3, (s0)                    # old = *s0
    addi    t3, t3, 1                   # new = old + 1
    sc.w    t6, t3, (s0)                # 写回; 失败 retry (单 hart 不应触发)
    bnez    t6, retry5
    lw      t5, 0(s0)
    li      t4, 101                     # 100 + 1
    bne     t5, t4, fail

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
#   5 段 Test 全部通过 → exit_code = 0 (FINISHER_PASS)
#   stderr 无 trap cause 2 (illegal) / cause 4 (load misalign) / cause 5 (load access fault) /
#     cause 6 (store/AMO misalign) / cause 7 (store/AMO access fault) / cause 15 (page fault)
#   wall time < 100ms (~30 条指令)
#   无 ASan / TSan 报错
# 不期望:
#   - LR / SC decode 漏 → cause 2 illegal
#   - SC 立即配对 LR 失败 → Test 2 fail
#   - SC 没 LR 虚假成功 → Test 3 fail (跟 b 路径 CAS-equivalent 区分点)
#   - 普通 SW 不清 reservation → Test 4 fail
#   - cmpxchg retry 死循环 → 单 hart 不应触发
