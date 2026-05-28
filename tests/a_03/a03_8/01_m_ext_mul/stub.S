# a03_8 / 01_m_ext_mul
#
# ===== 测试目的 =====
# 验 RV32M 4 条 mul 在 emulator interpreter 跑得对:
#   MUL    rd = (rs1 × rs2)[31:0]            低 32 bit
#   MULH   rd = (sx(rs1) × sx(rs2))[63:32]   signed × signed 高 32 bit
#   MULHSU rd = (sx(rs1) × zx(rs2))[63:32]   signed × unsigned 高 32 bit (不对称)
#   MULHU  rd = (zx(rs1) × zx(rs2))[63:32]   unsigned × unsigned 高 32 bit
#
# small_plan A.1 拍定 RV32M 实装 (silent miscompile 修复); 本 fixture 是 mul 4
# 条的正面 happy path 覆盖。div/rem 在 02_m_ext_div + 03_m_ext_div_edge。
#
# ===== 设计 =====
# 不接中断 / 不开 MIE / 不清 MDT/SDT。失败检测靠两路:
#   1. 显式 bne 跳 fail FINISHER 路径 (期望值不匹配)
#   2. 漏 case → OP_UNSUPPORTED → cause 2 illegal → MDT=1 reset 后 double trap
#      → main ABORT cleanup → emulator exit code != 0 (隐式抓)
# 任何一路都让 run_tests.py 判 fail; fixture 自身保持最简。
#
# ===== 期望结果 =====
# 10 个子测全过 → 写 sifive_test FINISHER PASS=0x5555 → emulator exit 0。
# 任何一个 bne 跳 fail → 写 FINISHER FAIL=0x3333 → emulator exit 1。

.equ TEST_DEV_BASE,      0x00100000
.equ TEST_DEV_SIFIVE_OFF,0x00
.equ FINISHER_PASS,      0x5555
.equ FINISHER_FAIL,      0x3333

.section .text
.option norvc
.globl _start

_start:
    # ---- Test 1: MUL 基础 6 × 7 = 42 ----
    li      t0, 6
    li      t1, 7
    mul     t2, t0, t1
    li      t3, 42
    bne     t2, t3, fail

    # ---- Test 2: MUL 截断 (mod 2^32; 0x10000 × 0x10000 = 0x1_00000000, 低 32 = 0) ----
    li      t0, 0x10000
    mul     t2, t0, t0
    bne     t2, x0, fail

    # ---- Test 3: MULH signed 同号 (0x10000 × 0x10000 = 0x1_00000000, 高 32 = 1) ----
    mulh    t2, t0, t0
    li      t3, 1
    bne     t2, t3, fail

    # ---- Test 4: MUL signed/unsigned 等价 ((-1) × (-1) = 1 低 32) ----
    li      t0, -1
    li      t1, -1
    mul     t2, t0, t1
    li      t3, 1
    bne     t2, t3, fail

    # ---- Test 5: MULH 负 × 正 ((-2) × 3 = -6, 64-bit signed 高 32 = 0xFFFFFFFF) ----
    li      t0, -2
    li      t1, 3
    mulh    t2, t0, t1
    li      t3, -1
    bne     t2, t3, fail

    # ---- Test 6: MULH 负 × 负 ((-2) × (-3) = 6, 高 32 = 0) ----
    li      t0, -2
    li      t1, -3
    mulh    t2, t0, t1
    bne     t2, x0, fail

    # ---- Test 7: MULHU 大 × 大 (0xFFFFFFFF × 0xFFFFFFFF = 0xFFFFFFFE_00000001,
    #              高 32 = 0xFFFFFFFE) ----
    li      t0, -1                  # = 0xFFFFFFFF as unsigned
    mulhu   t2, t0, t0
    li      t3, 0xFFFFFFFE
    bne     t2, t3, fail

    # ---- Test 8: MULHSU 负 × unsigned 大数 ((-1) × 0xFFFFFFFF
    #              数学: -1 × 4294967295 = -4294967295
    #              64-bit two's-comp: 0xFFFFFFFF_00000001, 高 32 = 0xFFFFFFFF = -1) ----
    li      t0, -1                  # signed: -1
    li      t1, -1                  # MULHSU 把 rs2 当 unsigned: 0xFFFFFFFF
    mulhsu  t2, t0, t1
    li      t3, -1                  # 0xFFFFFFFF
    bne     t2, t3, fail

    # ---- Test 9: MULHSU 正 × unsigned 大数 (1 × 0xFFFFFFFF = 0xFFFFFFFF, 高 32 = 0) ----
    li      t0, 1                   # signed: 1
    li      t1, -1                  # unsigned: 0xFFFFFFFF
    mulhsu  t2, t0, t1
    bne     t2, x0, fail

    # ---- Test 10: MULHSU 负 × 大 unsigned ((-2) × 0xFFFFFFFE
    #              数学: -2 × 4294967294 = -8589934588 = -0x1_FFFFFFFC
    #              64-bit two's-comp: 0xFFFFFFFE_00000004, 高 32 = 0xFFFFFFFE) ----
    li      t0, -2                  # signed: -2
    li      t1, -2                  # unsigned: 0xFFFFFFFE
    mulhsu  t2, t0, t1
    li      t3, 0xFFFFFFFE
    bne     t2, t3, fail

    j       success

# ============================================================================
# success / fail — sifive_test FINISHER 写 magic 退出
# ============================================================================
success:
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_PASS
    sw      t1, TEST_DEV_SIFIVE_OFF(t0)
    j       .                       # halt 在此 (FINISHER 写后 emulator system reset 退出)

fail:
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_FAIL
    sw      t1, TEST_DEV_SIFIVE_OFF(t0)
    j       .
