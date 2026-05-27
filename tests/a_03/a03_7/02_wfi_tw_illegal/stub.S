# a03_7 / 02_wfi_tw_illegal
#
# ===== 测试目的 =====
# RV Privileged Spec §3.1.6.5: mstatus.TW=1 + priv < M 时 WFI 必须 "有限时间内"
# trap illegal instruction; 项目 015 实装取 timeout=0 (立即 trap, spec 允许).
#
# 验证:
#   1. M-mode 设 mstatus.TW=1, mret 切到 S-mode
#   2. S-mode 执行 WFI → 必须立即触发 cause=2 illegal instruction trap
#   3. M-handler 捕获 mcause=2, mtval=0x10500073 (WFI raw instruction)
#   4. 验通过则 PASS
#
# 015 实装路径 (interpreter.c case OP_WFI):
#   priv<M + mstatus.TW=1 → SYNC_COUNT + trap_raise_exception(ILLEGAL, raw_inst)
#
# trace 期望: '_._.E' (没 'w' 字符 — TW illegal 路径不调 DEBUG_WFI, 不进 wfi_wait,
# 直接 trap → 'E' 表示 sync exception)
#
# ===== 期望结果 =====
#   s0 = 1 (handler 进入标志)
#   s1 = 2 (mcause = ILLEGAL_INSTRUCTION)
#   s2 = 0x10500073 (mtval = WFI raw instruction encoding)
#   sifive_test FINISHER PASS

.equ TEST_DEV_BASE,      0x00100000
.equ TEST_DEV_SIFIVE_OFF,0x00
.equ FINISHER_PASS,      0x5555
.equ FINISHER_FAIL,      0x3333
.equ WFI_RAW,            0x10500073

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

    # ---- counters / flags init ----
    li      s0, 0           # handler 进入标志
    li      s1, 0           # mcause
    li      s2, 0           # mtval

    # ---- mstatus 设 TW=1 + MPP=01 (S-mode) ----
    # bit 21 = TW; bits[12:11] = MPP (01=S)
    li      t0, (1 << 21) | (1 << 11)
    csrw    mstatus, t0

    # ---- mepc 指向 S-mode 入口 ----
    la      t0, s_entry
    csrw    mepc, t0

    # ---- mret → 切 S-mode ----
    mret

s_entry:
    # 现在 priv=S; mstatus.TW=1 → WFI 必须立即 trap illegal
    wfi
    # 走到这说明 TW 检查没生效 (bug)
    j       fail

m_handler:
    # 期望: mcause = 2 (ILLEGAL_INSTRUCTION), mtval = 0x10500073 (WFI raw)
    csrr    t0, mcause
    csrr    t1, mtval
    li      s0, 1
    mv      s1, t0
    mv      s2, t1

    # check mcause == 2
    li      t2, 2
    bne     t0, t2, fail
    # check mtval == 0x10500073
    li      t2, WFI_RAW
    bne     t1, t2, fail
    j       success

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
