# a04_3 / 04_lr_sc_misalign_reject
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# T3 LR.W / SC.W misalign 反向验证 (lrsc_amo_decision.md Q8):
#   - LR.W gva 非 4-byte aligned → cause 4 CAUSE_LOAD_ADDR_MISALIGNED
#   - SC.W gva 非 4-byte aligned → cause 6 CAUSE_STORE_ADDR_MISALIGNED
#
# interpreter case 入口 LOAD_MISALIGN_CHECK / STORE_MISALIGN_CHECK 宏 trap_raise_exception,
# helper 内部 lrsc_lr_helper / lrsc_sc_helper 不会到达; lrsc_lr_w / lrsc_sc_w 也不到.
#
# 2 子测覆盖 LR/SC 各一; 各自走单独 trap handler 验 mcause + mtval, 验完 increment
# stage 计数, 跑完 2 子测 → PASS.
#
# 配 01_lr_sc_simple 一起组成 LR/SC misalign 正反双覆盖:
#   01: 4-byte aligned LR/SC 不 trap
#   04: 非 4-byte aligned LR/SC trap cause 4/6 (本 fixture)

.equ TEST_DEV_BASE,     0x00100000
.equ FINISHER_PASS,     0x5555
.equ FINISHER_FAIL,     0x3333

.equ MISALIGN_ADDR,     0x80100001      # = 0x80100000 + 1 (非 4-byte aligned)
.equ CAUSE_LOAD_MISALIGN,  4
.equ CAUSE_STORE_MISALIGN, 6

.equ STAGE_ADDR,        0x80100010      # stage 计数器: 0=未跑 / 1=LR misalign 验过 / 2=PASS

.section .text
.option norvc
.globl _start

_start:
    csrw    0x310, x0                   # 清 MDT
    csrw    sstatus, x0                 # 清 SDT

    # init stage = 0
    li      t0, STAGE_ADDR
    sw      x0, 0(t0)

    # 装 unified handler (dispatch by mcause)
    la      t0, misalign_handler
    csrw    mtvec, t0

    # ============================================================
    # Stage 1: LR.W misalign → cause 4
    # ============================================================
    li      s0, MISALIGN_ADDR
    lr.w    t3, (s0)                    # → LOAD_MISALIGN_CHECK trap cause 4

    # fall-through: 不该到这 (handler 内 jump 不回主帧, mret 自走逻辑)
    j       fail

stage2_start:
    # ============================================================
    # Stage 2: SC.W misalign → cause 6
    # ============================================================
    li      s0, MISALIGN_ADDR
    li      t2, 0xDEADBEEF
    sc.w    t3, t2, (s0)                # → STORE_MISALIGN_CHECK trap cause 6

    j       fail

misalign_handler:
    csrr    t0, mcause
    csrr    t1, mtval                   # tval 应 = MISALIGN_ADDR
    li      t2, MISALIGN_ADDR
    bne     t1, t2, fail

    # Stage 1: 期望 cause 4 (LR misalign)
    li      t3, STAGE_ADDR
    lw      t4, 0(t3)
    bnez    t4, stage2_check            # stage != 0 → 已过 stage 1, 看 stage 2

    # Stage 1 check
    li      t5, CAUSE_LOAD_MISALIGN
    bne     t0, t5, fail
    # 标 stage = 1, 跳 stage 2 主帧
    li      t4, 1
    sw      t4, 0(t3)
    la      t0, stage2_start
    csrw    mepc, t0                    # 改写 mepc, mret 跳到 stage2_start
    mret

stage2_check:
    # Stage 2 期望 cause 6 (SC misalign)
    li      t5, CAUSE_STORE_MISALIGN
    bne     t0, t5, fail
    # 两子测都过, PASS
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
#   Stage 1: LR.W @ 0x80100001 → cause 4 + mtval=0x80100001 → handler mret 跳 stage 2
#   Stage 2: SC.W @ 0x80100001 → cause 6 + mtval=0x80100001 → handler 写 PASS
#   exit_code = 0 (FINISHER_PASS)
#   stderr 有两条 trap 报告 (cause 4 + cause 6)
#   wall time < 50ms
# 不期望:
#   - LR/SC 不 trap (= LOAD/STORE_MISALIGN_CHECK 漏 / 失效) → fall-through fail
#   - mcause 错 (走错 trap 路径)
#   - mtval 错 (= 不齐 gva)
#   - sanitizer 报错
