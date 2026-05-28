# a03_8 / 03_m_ext_div_edge
#
# ===== 测试目的 =====
# 验 RV32M div/rem 的 6 个 spec edge cases (RV Unprivileged § 7.2 Table 7.1).
# 关键 invariant: 全部不 trap, 全部返定值。emulator 实装漏短路 → 撞 C UB →
# sanitizer 喷或返乱值, 这里直接 bne fail; 没撞 sanitizer 也至少返不对值。
#
#   1. DIV  by-0           : rd = -1 (all-ones)
#   2. DIVU by-0           : rd = 2^32-1 (UINT32_MAX, all-ones)
#   3. REM  by-0           : rd = rs1 (dividend 原样)
#   4. REMU by-0           : rd = rs1
#   5. DIV  INT_MIN ÷ -1   : rd = INT_MIN (overflow 返 dividend)
#   6. REM  INT_MIN ÷ -1   : rd = 0       (overflow 返 0)
#
# 顺手验 2 个 sanity (跟 edge case 区分): dividend 0, divisor 非 0 走 normal path
#   → 0; 这两个 test 万一 short-circuit by-0 写错把 divisor 跟 dividend 颠倒判,
#   会撞 (走 by-0 路径写 -1, 期望 0)。
#
# ===== 设计 =====
# 同 01/02: 不接中断, 漏 case → ABORT 隐式 fail; bne 显式 fail FINISHER。

.equ TEST_DEV_BASE,      0x00100000
.equ TEST_DEV_SIFIVE_OFF,0x00
.equ FINISHER_PASS,      0x5555
.equ FINISHER_FAIL,      0x3333

.section .text
.option norvc
.globl _start

_start:
    # ---- Test 1: DIV by-0 → rd = -1 (all-ones) ----
    li      t0, 42
    li      t1, 0
    div     t2, t0, t1
    li      t3, -1
    bne     t2, t3, fail

    # ---- Test 2: DIVU by-0 → rd = UINT32_MAX (= 0xFFFFFFFF, all-ones) ----
    li      t0, 42
    li      t1, 0
    divu    t2, t0, t1
    li      t3, -1                   # 0xFFFFFFFF
    bne     t2, t3, fail

    # ---- Test 3: REM by-0 → rd = rs1 (dividend 原样) ----
    li      t0, 42
    li      t1, 0
    rem     t2, t0, t1
    li      t3, 42
    bne     t2, t3, fail

    # ---- Test 4: REMU by-0 → rd = rs1 ----
    li      t0, 42
    li      t1, 0
    remu    t2, t0, t1
    li      t3, 42
    bne     t2, t3, fail

    # ---- Test 5: DIV INT_MIN ÷ -1 → rd = INT_MIN (overflow 不 trap, 返 dividend) ----
    # C signed overflow 是 UB, emulator 实装漏短路 → UBSan 喷; bne 也能抓不对值。
    li      t0, 0x80000000           # INT_MIN
    li      t1, -1
    div     t2, t0, t1
    li      t3, 0x80000000           # INT_MIN
    bne     t2, t3, fail

    # ---- Test 6: REM INT_MIN ÷ -1 → rd = 0 (overflow 不 trap, 返 0) ----
    li      t0, 0x80000000           # INT_MIN
    li      t1, -1
    rem     t2, t0, t1
    bne     t2, x0, fail

    # ---- Test 7: DIV 0 / 7 = 0 (sanity, 撞 short-circuit 颠倒 dividend/divisor 判) ----
    li      t0, 0
    li      t1, 7
    div     t2, t0, t1
    bne     t2, x0, fail

    # ---- Test 8: REM 0 % 7 = 0 (同上 sanity) ----
    li      t0, 0
    li      t1, 7
    rem     t2, t0, t1
    bne     t2, x0, fail

    j       success

# ============================================================================
# success / fail — sifive_test FINISHER 写 magic 退出
# ============================================================================
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
