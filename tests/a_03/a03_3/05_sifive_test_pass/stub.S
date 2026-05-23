# a03_3 / 05_sifive_test_pass
#
# ===== 测试目的 =====
# sifive_test FINISHER PASS 路径 — 验 jit-emu 退出码:
#   1. sw 0x5555 → test_dev offset 0 (TEST_DEV_SIFIVE_OFF)
#   2. test_dev_write 解析 cmd=0x5555=PASS → set test_dev_exit_code=0 + atomic SRS=0+SDS=0
#   3. dispatcher 下个 block 末 atomic_load SRS=0 退主循环
#   4. main while 退 → cleanup chain → return test_dev_get_exit_code() = 0
#   5. shell 退出码 0 (PASS)
#
# ===== 思路 =====
# RV I-type imm signed 12-bit (-2048..+2047), 立即数 0x5555 = 21845 > 2047 → 单条 li
# 走 lui+addi pair (GNU as macro 展开). sw t1, 0(t0) offset=0 直接.
#
# csrw mtvec, x0 + ecall 保险段 — sw 0x5555 后理论上 dispatcher 立即退, 但写 ecall
# 防御 (若 dispatcher SRS check 推迟到本 block 之后, ecall 让 in_trap 累加另路径).

.equ TEST_DEV_BASE,         0x00100000
.equ TEST_DEV_SIFIVE_OFF,   0x00
.equ FINISHER_PASS,         0x5555

.section .text
.option norvc
.globl _start

_start:
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_PASS
    sw      t1, TEST_DEV_SIFIVE_OFF(t0)

    # 保险段: 若 dispatcher 未立即退, ecall halt 也走 SRS=0 + cleanup, 退出码仍 0
    csrw    mtvec, x0
    ecall

# ===== 期望结果 =====
# shell 退出码:
#   $? = 0   (test_dev sifive_test FINISHER PASS magic 路径)
#
# 期望 stderr (release config):
#   [decode_test] PASS
#   [perf] elapsed=...
#   [dispatcher] halted: in_trap=... pc=...
#     (in_trap 可能 = 0 = dispatcher SRS=0 退; 或 = 3 = ecall + triple fault, 两路均合规)
#   [main] elapsed: ...
#
# 不期望:
#   - 退出码非 0 (PASS magic 该走 exit_code=0 路径)
#   - 程序 hang (test_dev_write SRS=0+SDS=0 让 main + dispatcher 都退)
