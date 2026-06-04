# a04_3 / 03_lr_sc_aba_aa
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# ABA / AA 场景 — 验 c 主方案 SC 严格失败 (跟 b 路径 CAS-equivalent SC 区分点).
#
# RV spec 选 LR/SC 不选 host CAS 的核心理由 = 避 ABA: SC 必须基于 reservation 状态判,
# 不能仅基于 "*mem 当前值跟 LR.old 比对" — 否则 ABA / AA 场景 SC 会虚假成功:
#   ABA: 其它 hart write B → write A; SC 路径下 *mem == A 比对成功但语义 racy
#   AA:  其它 hart write A (同值); SC *mem == A 比对成功但 reservation 已被清
#
# c 主方案 SC 内部:
#   1. atomic_load self.reservation
#   2. 若 self.reservation == pa_word → SC 成功; 否则失败
#   并不查 *mem 值. ABA 后 lrsc_on_store 已清 self.reservation, SC atomic_load 看
#   INVALID, return 1 (失败) ✓
#
# 单 hart 测试等价 cross-hart (lrsc_on_store 含自己, hart 自己 SW 也清自己 reservation;
# lrsc_amo_decision.md Q5 A3a / RV spec line 116-118 implementation discretion).
# 选单 hart 避开 cross-hart RAM access TSan latent race (跟 02_lr_sc_spinlock fixture
# 上 TSan note 一致, 项目接受).
#
# 2 子测覆盖 ABA + AA 场景, 任一虚假成功走 fail.

.equ TEST_DEV_BASE,     0x00100000
.equ FINISHER_PASS,     0x5555
.equ FINISHER_FAIL,     0x3333

.equ SCRATCH,           0x80100000

.section .text
.option norvc
.globl _start

_start:
    # ---- Boot prelude ----
    csrw    0x310, x0
    csrw    sstatus, x0

    li      s0, SCRATCH

    # ============================================================
    # Test 1: ABA 场景
    #   1. mem = A=0xAAAAAAAA
    #   2. LR.W → rd = A, 建 reservation
    #   3. SW B=0xBBBBBBBB → mem = B, lrsc_on_store 清自己 reservation
    #   4. SW A → mem = A again, lrsc_on_store 仍清 (但 reservation 已 INVALID, 无操作)
    #   5. SC.W X=0x99999999 → c 路径 reservation INVALID → 必失败 rd=1
    #      (b 路径 CAS-equivalent 会比对 *mem == A 跟 LR.old == A 成功, 虚假成功)
    # ============================================================
    li      t1, 0xAAAAAAAA              # value A
    sw      t1, 0(s0)
    lr.w    t3, (s0)                    # rd=A, 建 reservation @ s0
    li      t4, 0xAAAAAAAA
    bne     t3, t4, fail                # LR 返回 A ✓

    li      t1, 0xBBBBBBBB              # value B
    sw      t1, 0(s0)                   # mem = B, lrsc_on_store 清 self.reservation
    li      t1, 0xAAAAAAAA              # value A again
    sw      t1, 0(s0)                   # mem = A again (ABA 关键)

    li      t2, 0x99999999              # SC value (不是 A/B/原值)
    sc.w    t3, t2, (s0)                # c 路径必失败 rd=1
    li      t4, 1
    bne     t3, t4, fail                # SC 应失败; 虚假成功 = b 路径行为, fail
    lw      t5, 0(s0)
    li      t4, 0xAAAAAAAA              # SC 失败不写 mem, 仍是 A
    bne     t5, t4, fail

    # ============================================================
    # Test 2: AA 场景 (单次同值写)
    #   1. mem = A=0xCCCCCCCC
    #   2. LR.W → rd = A, 建 reservation
    #   3. SW A (同值) → mem = A, lrsc_on_store 清 self.reservation
    #   4. SC.W X → c 路径 reservation INVALID → 必失败 rd=1
    #      (b 路径 *mem == A 跟 LR.old == A 比对成功, 虚假成功)
    # ============================================================
    li      t1, 0xCCCCCCCC              # value A
    sw      t1, 0(s0)
    lr.w    t3, (s0)                    # rd=A, 建 reservation
    li      t4, 0xCCCCCCCC
    bne     t3, t4, fail

    li      t1, 0xCCCCCCCC              # 写同值 (AA 关键)
    sw      t1, 0(s0)                   # mem 没变化, 但 lrsc_on_store 仍清 self.reservation

    li      t2, 0x77777777
    sc.w    t3, t2, (s0)                # c 路径必失败 rd=1
    li      t4, 1
    bne     t3, t4, fail
    lw      t5, 0(s0)
    li      t4, 0xCCCCCCCC              # SC 失败不写 mem
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
#   ABA + AA 场景 SC 都正确失败 → exit_code = 0 (FINISHER_PASS)
#   wall time < 100ms (~30 条指令)
# 不期望:
#   - SC 虚假成功 (b 路径 CAS-equivalent 行为) → fail handler → exit_code != 0
#     这是 c 主方案选择不选 b 路径的核心 spec compliance 区分点
