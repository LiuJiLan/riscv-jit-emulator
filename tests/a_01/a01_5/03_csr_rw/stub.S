# a_01_5 / 03_csr_rw
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验证 csr.c 6 小 helper 真读写 (csrw 写 → csrr 读回相同值; WARL 截断对 mtvec/mepc 真生效)。
# (注: 原收尾用 ecall → triple fault 停机; a_03 后改 test_dev 干净停机。本 fixture 不触
#  handler trap, 不需清 MDT prelude。)
#
# ===== 思路 =====
# csrw / csrr 是 pseudo:
#   csrw csr, rs1 = csrrw x0, csr, rs1  (op=RW, do_write=true)
#   csrr rd, csr  = csrrs rd, csr, x0   (op=RS, new_val=0, do_write=false → 只读不写)
#
# WARL 截断验证:
#   - mtvec write 0x42 (0b01000010) → mask 低 2 位 → 0x40 (Vectored 不实现, 强制 Direct)
#   - mepc write 0x42 → mask IALIGN_MASK (=1) → 0x42 (低 1 位本来就 0, 不变)
#   - mcause / mtval 无 WARL, 接受任意值
# 每条 csr 是硬块边界, dispatcher 多次重派发; 末尾 test_dev PASS 干净停机。

.section .text
.option norvc
.globl _start
_start:
    addi x10, x0, 0x42      # x10 = 0x42 (csr 写源)
    csrw mtvec, x10         # mtvec = 0x42 → WARL → 0x40
    csrr x1, mtvec          # x1 = mtvec = 0x40 (验真读写 + WARL 截断)
    csrw mcause, x10        # mcause = 0x42 (无 WARL)
    csrr x2, mcause         # x2 = mcause = 0x42
    csrw mepc, x10          # mepc = 0x42 → WARL IALIGN=1 → 0x42 (低 1 位 0)
    csrr x3, mepc           # x3 = mepc = 0x42

    # ---- 停机: test_dev FINISHER PASS ----
    li   x30, 0x100000      # test_dev base
    li   x31, 0x5555        # FINISHER PASS magic
    sw   x31, 0(x30)        # 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望:
#   x1 = 0x40   (csrr mtvec 读回 csrw 写的; 验 mtvec WARL 真截断 0x42→0x40)
#   x2 = 0x42   (csrr mcause 读回; 验 mcause 无 WARL)
#   x3 = 0x42   (csrr mepc 读回; 验 mepc WARL 对偶数不影响)
#   x30 = 0x100000 / x31 = 0x5555  (停机 scratch)
#   停机 = test_dev FINISHER PASS → exit code 0
# (注: 绝对 pc / total_count 见 REVIEW B.3 段; 不在断言内。)
