# a_01_6 / 02_load_misalign_reject
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验证 load_helper 的 misalign 检查 (Spike 风格): lw 一个奇地址 → trap_raise(4 load misalign)。
# 跳 handler 后 csrr 验 mcause/mtval/mepc, 然后 test_dev 干净停机。
# (注: 原收尾用 ecall + triple fault; a_03 后改 test_dev 干净停机 + 开头清 MDT prelude
#  让 misalign trap 能投递到 handler。)
#
# ===== 思路 =====
# misalign 触发条件 (lsu.h load_helper Step 1):
#   (gva & (size - 1)) != 0
# lw size=4, mask=3; gva=0x80001001 → 0x80001001 & 3 = 1 != 0 → trap cause=4, tval=0x80001001。
# 控制流:
#   _start: 清 MDT → la+csrw 设 mtvec=handler → lui+addi 算奇地址 → lw (misalign → trap)
#   trap → handler: csrr x1=mcause(4), x2=mtval(0x80001001), x3=mepc(lw PC) → test_dev PASS 停机
# test_dev 用 x30/x31 (避开 x1/x2/x3/x4/x6/x9)。

.section .text
.option norvc
.globl _start
_start:
    csrw 0x310, x0          # 清 mstatush.MDT (否则 misalign trap 撞 MDT=1 直接 abort, handler 跑不到)
    la   x4,  handler       # auipc + addi (la pseudo)
    csrw mtvec, x4          # mtvec = handler

    # ---- 触发 misalign load ----
    lui  x6,  0x80001       # x6 = 0x80001000
    addi x6,  x6, 1         # x6 = 0x80001001 (奇地址, 不 4-byte 对齐)
    lw   x9,  0(x6)         # lw misalign → trap cause=4, tval=0x80001001
    # ↑ trap_raise_exception _Noreturn longjmp; 不到此处

handler:
    csrr x1,  mcause        # x1 = mcause = 4 (load address misaligned)
    csrr x2,  mtval         # x2 = mtval = 0x80001001 (触发 ea)
    csrr x3,  mepc          # x3 = mepc = lw 指令的 PC

    # ---- 停机: test_dev FINISHER PASS ----
    li   x30, 0x100000      # test_dev base
    li   x31, 0x5555        # FINISHER PASS magic
    sw   x31, 0(x30)        # 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望 (handler 内 csrr 拿到 trap 当时的 csr 值):
#   x1 = 4          (mcause = load address misaligned, RV spec §3.1.16)
#   x2 = 0x80001001 (mtval = 触发 misalign 的 GVA; 数据相关, 不随代码平移)
#   x3 = lw 指令的 PC (mepc; 绝对值随 prelude +4 漂, 见 REVIEW B.3 段)
#   x30 = 0x100000 / x31 = 0x5555  (停机 scratch)
#   停机 = test_dev FINISHER PASS → exit code 0 (misalign→handler 投递 OK)
# (注: 绝对 pc / total_count 见 REVIEW B.3 段; 不在断言内。)
