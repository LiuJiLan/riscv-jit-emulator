# a03_8 / 02_m_ext_div
#
# ===== 测试目的 =====
# 验 RV32M div/rem 4 条 (DIV/DIVU/REM/REMU) 正常路径:
#   - signed: 正/正 + 负/正 + 正/负 + 负/负 各一例 (验 truncation 方向 + 符号)
#   - unsigned: 大数 / 小 + 0 / N 各一例
#   - REM 跟 DIV 同 dividend/divisor: 验 (a/b)*b + (a%b) == a 不变式 + 符号匹配 dividend
#
# RV spec § 7.2 truncation: round towards zero (跟 C 同; signed -100/3 = -33, 不是 -34).
# edge cases (by-0 + INT_MIN÷-1 overflow) 在 03_m_ext_div_edge。
#
# ===== 设计 =====
# 同 01_m_ext_mul: 不接中断, 漏 case → ABORT 隐式 fail; bne 显式 fail FINISHER。

.equ TEST_DEV_BASE,      0x00100000
.equ TEST_DEV_SIFIVE_OFF,0x00
.equ FINISHER_PASS,      0x5555
.equ FINISHER_FAIL,      0x3333

.section .text
.option norvc
.globl _start

_start:
    # ---- Test 1: DIV 100 / 3 = 33 (truncation: 33.33 → 33) ----
    li      t0, 100
    li      t1, 3
    div     t2, t0, t1
    li      t3, 33
    bne     t2, t3, fail

    # ---- Test 2: DIV -100 / 3 = -33 (truncation towards 0: -33.33 → -33, 不是 -34) ----
    li      t0, -100
    li      t1, 3
    div     t2, t0, t1
    li      t3, -33
    bne     t2, t3, fail

    # ---- Test 3: DIV 100 / -3 = -33 (truncation towards 0) ----
    li      t0, 100
    li      t1, -3
    div     t2, t0, t1
    li      t3, -33
    bne     t2, t3, fail

    # ---- Test 4: DIV -100 / -3 = 33 ----
    li      t0, -100
    li      t1, -3
    div     t2, t0, t1
    li      t3, 33
    bne     t2, t3, fail

    # ---- Test 5: DIVU 0xFFFFFFFE / 2 = 0x7FFFFFFF
    #              (signed -2 = unsigned 4294967294; / 2 = 2147483647 = 0x7FFFFFFF) ----
    li      t0, -2                   # 0xFFFFFFFE as unsigned
    li      t1, 2
    divu    t2, t0, t1
    li      t3, 0x7FFFFFFF
    bne     t2, t3, fail

    # ---- Test 6: REM 100 % 3 = 1 ----
    li      t0, 100
    li      t1, 3
    rem     t2, t0, t1
    li      t3, 1
    bne     t2, t3, fail

    # ---- Test 7: REM -100 % 3 = -1 (验: -33 * 3 + (-1) = -100; 余数符号匹配 dividend) ----
    li      t0, -100
    li      t1, 3
    rem     t2, t0, t1
    li      t3, -1
    bne     t2, t3, fail

    # ---- Test 8: REM 100 % -3 = 1 (验: -33 * (-3) + 1 = 100) ----
    li      t0, 100
    li      t1, -3
    rem     t2, t0, t1
    li      t3, 1
    bne     t2, t3, fail

    # ---- Test 9: REM -100 % -3 = -1 (验: 33 * (-3) + (-1) = -100) ----
    li      t0, -100
    li      t1, -3
    rem     t2, t0, t1
    li      t3, -1
    bne     t2, t3, fail

    # ---- Test 10: REMU 0xFFFFFFFE % 3
    #               4294967294 / 3 = 1431655764 余 2 (4294967294 - 1431655764*3 = 2) ----
    li      t0, -2                   # 0xFFFFFFFE as unsigned
    li      t1, 3
    remu    t2, t0, t1
    li      t3, 2
    bne     t2, t3, fail

    # ---- Test 11: DIVU 0 / 5 = 0 ----
    li      t0, 0
    li      t1, 5
    divu    t2, t0, t1
    bne     t2, x0, fail

    # ---- Test 12: REMU 0 % 5 = 0 (复用 t0/t1) ----
    remu    t2, t0, t1
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
