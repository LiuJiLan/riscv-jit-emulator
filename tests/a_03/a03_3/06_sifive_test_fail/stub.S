# a03_3 / 06_sifive_test_fail
#
# ===== 测试目的 =====
# sifive_test FINISHER FAIL 路径 — 验 jit-emu 退出码上报 fail code:
#   1. sw (42<<16)|0x3333 → test_dev offset 0
#   2. test_dev_write 解析 cmd=0x3333=FAIL, arg=42 → set test_dev_exit_code=42 + SRS=0+SDS=0
#   3. main while 退 → cleanup chain → return 42
#   4. shell 退出码 42 (FAIL with code 42)
#
# ===== 思路 =====
# FAIL value 编码 = (arg << 16) | 0x3333 = (42 << 16) | 0x3333 = 0x002A3333. 单 li 走
# lui+addi pair.

.equ TEST_DEV_BASE,         0x00100000
.equ TEST_DEV_SIFIVE_OFF,   0x00
.equ FINISHER_FAIL_42,      0x002A3333          # (42 << 16) | 0x3333

.section .text
.option norvc
.globl _start

_start:
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_FAIL_42
    sw      t1, TEST_DEV_SIFIVE_OFF(t0)

    # 保险段: 若 dispatcher 未立即退, ecall halt 走 cleanup; exit_code 已设 42
    csrw    mtvec, x0
    ecall

# ===== 期望结果 =====
# shell 退出码:
#   $? = 42   (test_dev sifive_test FINISHER FAIL arg=42)
#
# 不期望:
#   - 退出码 0 (说明 FAIL magic 没解析对, 或 exit_code 字段没更新)
#   - 退出码 != 42 (arg 解析错)
#   - 程序 hang
