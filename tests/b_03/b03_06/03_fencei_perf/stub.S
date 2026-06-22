# b_03_6 / 03_fencei_perf
#
# ===== 运行参数 =====
# RUN-RELEASE
#
# ===== 测试目的 =====
# perf 套件 · fence.i 稳态调用开销 (b_03 T2 设计 verify):
# b_03 T2 audit Q4.2.1+ 拍 fence_i_helper **不主动 invalidate** (SMC SIGSEGV 是
# 唯一 invalidate 工作来源), 仅做 lrsc_clear_self. 设计预期 fence.i 在无 SMC 触
# 发时 cost = (块边界 + helper call + 单原子 store 清 reservation) ~= 几十 cycle,
# 跟无 fence.i 的对照基线只差块边界 dispatcher 一次回程.
#
# 本 fixture 量化此预期: hot loop 每 ~10 inst 1 个 fence.i, 比 a02_7/01_bare 多
# 一个块边界 + helper call. 期望 MIPS 显著低于 a02_7/01_bare baseline (674 MIPS) —
# 不是因 fence.i 贵, 而是因 block 一半变小 (每 10 inst 变 2 块 vs 01_bare 1 块);
# 跟 a02_7/13_blocksize_08 (564 MIPS, 8 inst block) 量级近.
#
# ===== 思路 =====
# 循环体 = 7 条 ALU + fence.i + addi + bnez = 10 inst
#   block A: 7 ALU + fence.i = 8 inst (fence.i 是硬块边界)
#   block B: addi + bnez = 2 inst (bnez 是硬块边界, 回跳 work_loop)
# 每 iter = 2 block dispatch, 跟 a02_7/01_bare (10 inst 1 block) 对照.
#
# ITER_COUNT 调到 release ~1s 量级.

.section .text
.option norvc
.globl _start
_start:
    li      s0, 0
    li      s1, 30000000               # ITER_COUNT (调到 release ~1s; smoke 60M 跑了 2.1s)
    li      t0, 0x12345

work_loop:
    add     s0, s0, t0                 # 7 ALU
    xor     s0, s0, t0
    sub     s0, s0, t0
    and     s0, s0, t0
    or      s0, s0, t0
    slli    s0, s0, 1
    srli    s0, s0, 1
    fence.i                            # 硬块边界 (b_03 T2: helper = lrsc_clear_self)
    addi    s1, s1, -1                 # block B
    bnez    s1, work_loop              # 块边界 + 回跳

    mv      a0, s0

    csrw    mtvec, x0
    ecall

# ===== 期望结果 =====
# [perf] elapsed≈1s  total_count≈300M  MIPS≈? (release median × 5)
# 期望 MIPS:
#   - 显著低于 a02_7/01_bare (674 MIPS, 10 inst block, 1 块每 iter)
#   - 跟 a02_7/13_blocksize_08 (564 MIPS, 8 inst block, 1 块每 iter) 接近 — 平均
#     5 inst block (8+2)/2 = 5 跟 blocksize_08 同档
#   - 跟 13 的 delta 反映 fence.i helper call + lrsc_clear_self 额外开销 (~几十 ns)
#
# 验 b_03 T2 设计: fence.i 无 SMC 时 cost ≈ 块边界 cost, 跟 sfence.vma "不主动
# invalidate" 同精神 (dummy.txt §16/§17).
# x6/s0 = ALU 累加结果, 跨运行恒定.
# 停机 = MDT abort exit 1
