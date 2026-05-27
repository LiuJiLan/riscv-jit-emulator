# a03_7 / 03_wfi_plic_wake
#
# ===== 测试目的 =====
# WFI + PLIC MEIP 配合 (015 plic_recompute_ctx_eip_locked 内 0→1 翻转钩子接 wfi_kick).
#
# 单 hart 局限: 没法 "hart 在 WFI 睡觉时, 外部触发 PLIC pending → kick 真唤醒".
# 真异步 wake 验证需 SMP 第二 hart (或 host async source 主动 set test_dev pending),
# 当前框架不具备. 本 fixture 退化验:
#   1. WFI 醒来后 trap 到 mtvec, mcause=MEIP (cause 11)
#   2. handler 走 PLIC claim+complete 标准 flow
#   3. plic_recompute_ctx_eip_locked → wfi_kick 钩子 (claim 改 eip / complete 改 eip)
#      路径都跑通不破其它东西
#
# 跟 timer wake (01) 配合验完整: 既然 01 已证 WFI 在 cond_wait 真挂起被 kick 唤醒,
# 本 fixture 在 WFI 进入时 MEIP 已 pending (set_pending 先于 WFI), predicate 立即返
# true, wfi_wait 不挂起立刻返 — trace 'w' 后立即 'e' (PLIC MEIP), 无 sleep gap.
#
# 实装路径 (plic.c plic_recompute_ctx_eip_locked):
#   atomic_store(plic_ctx_eip[ctx_id], v_new, release)
#   v_old==0 + v_new!=0 → wfi_kick(plic_ctx_to_hartid[ctx_id])  (015 加, MEIP/SEIP 0→1 唤醒)
#
# ===== 期望结果 =====
#   s0 = 1 (handler 进入标志)
#   s1 = 0x8000000B (mcause = M_EXT interrupt = 0x80000000 | 11)
#   s2 = 3 (claim 返回值 = source id)
#   sifive_test FINISHER PASS

.equ CLINT_BASE,            0x02000000
.equ PLIC_BASE,             0x0C000000
.equ PLIC_PRIORITY_OFF,     0x0           # per-source priority (4B each, src 0..N)
.equ PLIC_PENDING_OFF,      0x1000        # pending bitmap (RO)
.equ PLIC_ENABLE_CTX0_OFF,  0x2000        # enable bitmap for ctx 0 (M-mode hart 0)
.equ PLIC_THRESHOLD_CTX0_OFF, 0x200000    # threshold for ctx 0
.equ PLIC_CLAIM_CTX0_OFF,   0x200004      # claim/complete for ctx 0
.equ TEST_DEV_BASE,         0x00100000
.equ TEST_DEV_SIFIVE_OFF,   0x00
.equ TEST_DEV_SET_OFF,      0x40
.equ TEST_DEV_CLEAR_OFF,    0x44
.equ FINISHER_PASS,         0x5555
.equ FINISHER_FAIL,         0x3333
.equ PLIC_SRC_TEST,         3              # test_dev fanout src id (跟 a03_2/05 一致)

.section .text
.option norvc
.globl _start

_start:
    # ---- Boot prelude: 清 MDT / SDT ----
    csrw    0x310, x0
    csrw    sstatus, x0

    # ---- mtvec = m_handler ----
    la      t0, m_handler
    csrw    mtvec, t0

    # ---- mie = MEIE (bit 11) ----
    li      t0, 0x800
    csrw    mie, t0

    # ---- mideleg = 0 (不 delegate) ----
    csrw    mideleg, x0

    # ---- PLIC setup ----
    # priority[3] = 1
    li      t0, PLIC_BASE
    li      t1, 1
    sw      t1, 12(t0)              # offset 4*3 = 12

    # threshold[ctx0] = 0
    li      t0, PLIC_BASE + PLIC_THRESHOLD_CTX0_OFF
    sw      x0, 0(t0)

    # enable bit 3 for ctx 0 (M-mode hart 0)
    li      t0, PLIC_BASE + PLIC_ENABLE_CTX0_OFF
    li      t1, 0x8                 # bit 3
    sw      t1, 0(t0)

    # ---- counters / flags init ----
    li      s0, 0           # handler 进入标志
    li      s1, 0           # mcause
    li      s2, 0           # claim 值

    # ---- 开 MIE FIRST (无 MEIP pending, csrrsi boundary 后 trap_check 不 fire) ----
    csrrsi  x0, mstatus, 0x8

    # ---- 关键: set_pending + wfi 必须在同一 block 内, sw 是非 boundary,
    #      跟 wfi 在同 block. 否则:
    #        set_pending → boundary → trap_check fire MEIP → handler 清 MEIP →
    #        mret 回 wfi → wfi_should_wake 看 mip=0 → cond_wait 永睡.
    #      正确流程:
    #        block N: sw (set_pending) + wfi (boundary 结束)
    #          sw 内 plic_recompute → mip.MEIP=1
    #          wfi 内 wfi_should_wake 看 (mie & mip)=MEIE & MEIP ≠ 0 → 立即返
    #          PC+=4, block 结束
    #        dispatcher trap_check: MEIP+MIE=1 → fire trap → handler → mret 回 j success
    # ---- ----
    li      t0, TEST_DEV_BASE
    li      t1, PLIC_SRC_TEST
    sw      t1, TEST_DEV_SET_OFF(t0)
    # WFI: MEIP 已 pending, predicate 立即返 true, 不挂起;
    # trace 期望 'w' 后无 sleep, 紧接 'e' (MEIP trap), 但 'e' 不在本 block — 是 wfi
    # boundary 退出 block 后 dispatcher trap_check 打的.
    wfi

    # 醒来后 (handler 已跑过 mret 返回) 跳 success 写 PASS
    j       success

m_handler:
    csrr    t0, mcause              # 期望 0x8000000B (M_EXT)
    li      s0, 1
    mv      s1, t0

    # check mcause == 0x8000000B
    li      t2, 0x8000000B
    bne     t0, t2, fail

    # claim: 读 PLIC claim 寄存器, 应该返 source id = 3
    li      t0, PLIC_BASE + PLIC_CLAIM_CTX0_OFF
    lw      t1, 0(t0)
    mv      s2, t1
    li      t2, PLIC_SRC_TEST
    bne     t1, t2, fail

    # clear device line first (test_dev_clear_pending)
    li      t0, TEST_DEV_BASE
    li      t1, PLIC_SRC_TEST
    sw      t1, TEST_DEV_CLEAR_OFF(t0)

    # complete: 写回 source id 到 claim 寄存器 (RV PLIC complete protocol)
    li      t0, PLIC_BASE + PLIC_CLAIM_CTX0_OFF
    li      t1, PLIC_SRC_TEST
    sw      t1, 0(t0)

    mret

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
