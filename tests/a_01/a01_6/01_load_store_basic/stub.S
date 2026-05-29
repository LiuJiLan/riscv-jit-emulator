# a_01_6 / 01_load_store_basic
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验证 a_01_6 load/store 基本读写 + sext/zext 区别:
#   - sw + lw 32-bit 往返 (验 store_helper 写 + load_helper 读 + 4-byte 对齐 OK)
#   - sb 0xFF + lb (sext) → -1 = 0xFFFFFFFF
#   - sb 同字节 + lbu (zext) → 0xFF
# (注: 原收尾用 csrw mtvec,x0 + ecall triple fault; a_03 后改 test_dev 干净停机。本 fixture
#  不触 handler trap, 不需清 MDT prelude。)
#
# ===== 思路 =====
# 数据区: 用 RAM 内 [0x80001000, 0x80001008) 的 8 字节做读写 (RAM 是 mmap 上的, 任意地址都
# 可写; .text 段离 0x80001000 还远不冲突)。算术/load/store 跑完末尾 test_dev PASS 停机。
# test_dev 用 x30/x31 (避开数据基址 x6)。

.section .text
.option norvc
.globl _start
_start:
    lui  x6,  0x80001       # x6 = 0x80001000 (data base, RAM 内)

    # ---- 测试 1: sw + lw 32-bit 往返 ----
    addi x10, x0, 0x42      # x10 = 0x42 (源)
    sw   x10, 0(x6)         # mem[0x80001000] = 0x00000042 (4 字节)
    lw   x1,  0(x6)         # x1 = 0x42

    # ---- 测试 2: sb + lb sign-ext ----
    addi x11, x0, -1        # x11 = 0xFFFFFFFF (低 8 位 = 0xFF)
    sb   x11, 4(x6)         # mem[0x80001004] = 0xFF (1 字节)
    lb   x2,  4(x6)         # x2 = sext(0xFF) = -1 = 0xFFFFFFFF

    # ---- 测试 3: lbu zero-ext (同字节, 验 helper 不做 sext) ----
    lbu  x3,  4(x6)         # x3 = zext(0xFF) = 0xFF

    # ---- 停机: test_dev FINISHER PASS ----
    li   x30, 0x100000      # test_dev base (用 x30/x31 避开数据基址 x6)
    li   x31, 0x5555        # FINISHER PASS magic
    sw   x31, 0(x30)        # 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望 (main.c 末尾 fprintf):
#   x1 = 0x42       (= 66 in %u; 32-bit lw 读回)
#   x2 = 0xFFFFFFFF (= 4294967295 in %u; lb sign-ext -1)
#   x3 = 0xFF       (= 255 in %u; lbu zero-ext)
#   x30 = 0x100000 / x31 = 0x5555  (停机 scratch)
#   停机 = test_dev FINISHER PASS → exit code 0
# (注: 绝对 pc / total_count 见 REVIEW B.3 段; 不在断言内。)
