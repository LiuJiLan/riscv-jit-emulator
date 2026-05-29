# a02_7 / 03_csr_heavy
#
# ===== 运行参数 =====
# RUN-RELEASE
#
# ===== 测试目的 =====
# 解释器 perf 套件之三: CSR 密集路径吞吐。M-mode bare 下每轮读 8 条 CSR, 每条
# csrr 走 csr_op 大 helper (slow path)。03↔01_bare 的 MIPS 差 = CSR 读路径相对
# 纯 ALU 的开销。
#
# ===== 思路 =====
# 工作循环每轮 10 inst: 8 条 csrr + addi + bnez。
# 读的 CSR: mhartid / misa / mvendorid / marchid / mimpid / mscratch / mcause /
#   mtval —— 覆盖 csr_op 大 switch 的多个分支 (RO identity 类 / M-trap 类)。
# 每条 csrr 进 csr_op 入口 (判 priv / RO) + 大 switch 分发, 是典型 slow path。
# 注: csrr (=csrrs rd,csr,x0) 虽不写 CSR, 但 decode.h is_block_boundary_inst 把全部
#   6 个 OP_CSR* 都当硬边界 (plan §1.6 "所有 csr 视硬边界" 简化) → 每条 csrr 都结束
#   一个 block。本循环每轮 = 9 个 block (8 个单-csrr block + 末尾 addi/bnez block),
#   dispatch 次数远高于其它 fixture (它们每轮 1 block)。
# ITER_COUNT (s1 初值) 调长跑量, 目标 release ~1s; 性能读 [perf] 行, 跑批取中位。

.section .text
.option norvc
.globl _start
_start:
    li      s1, 8500000             # s1 = ITER_COUNT (调这个值改长跑量; 调到 release ~1s)

work_loop:
    csrr    t0, mhartid             # 8 条 csrr — 每条走 csr_op slow path
    csrr    t1, misa
    csrr    t2, mvendorid
    csrr    t3, marchid
    csrr    t4, mimpid
    csrr    t5, mscratch
    csrr    t6, mcause
    csrr    s0, mtval
    addi    s1, s1, -1              # counter--
    bnez    s1, work_loop           # 块边界 + 循环回跳

    mv      a0, s0                  # a0 = 最后一次读到的 mtval

    # ---- halt: 清 mtvec + ecall 触发 triple fault ----
    csrw    mtvec, x0
    ecall

# ===== 期望结果 =====
# [perf] elapsed≈1s  total_count≈ITER_COUNT*10 + 预备/收尾 ~5  MIPS≈实测
# [dispatcher] halted: in_trap=3  (csrw mtvec,x0 + ecall → triple fault)
# a0 (x10) = 0  (mtval 从未被写, csrr 读出 0)
# MIPS 应明显低于 01_bare (csr_op slow path vs 纯 ALU)
#
# ============================================================================
# 建立这个测试当下的实验结果记录
# (未来可能会过时, 但属于试验记录, 不用清理; 数值未来可能不同)
# ============================================================================
#
# 试验日期: AEST 2026-05-23   环境: release build (-O2, 无 ASan), 解释器, 单 hart
# 跑批: 连跑 5 次取 median
#
#   [perf] elapsed≈1.03s  total_count=85000004  median MIPS ≈ 83
#   5 跑 MIPS: 80.3 / 82.4 / 82.7 / 83.3 / 83.8
#   观察: ≈ 01_bare (≈174) 的一半。每轮同样 10 inst, 但 03 每轮 9 个 block (csrr
#   是硬边界) vs 01 的 1 个 —— 03 的慢是 csr_op slow path + 9x dispatch 开销 (含每
#   block 一次中断检查) 叠加。见 tests/review/REVIEW.md 中断检查开销实验。
