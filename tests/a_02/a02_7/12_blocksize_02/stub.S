# a02_7 / 12_blocksize_02
#
# ===== 运行参数 =====
# RUN-RELEASE
#
# ===== 测试目的 =====
# perf 套件·block 大小 sweep (fixture 12-16): bare M-mode 纯算术紧循环, 每个 block
# 固定 2 条指令。配 中断检查 ON/OFF A/B (见 REVIEW.md), 测"每 block 一次的中断
# 检查开销"如何随 block 大小变化 —— block 越小、每指令摊到的检查越多。本格是最
# 小 block (2), 检查占比应最高。
#
# ===== 思路 =====
# 循环体 = 0 条 add (WORK_N = block-2 = 0) + addi(计数) + bnez。bnez 是唯一硬边界
# → 每轮 = 1 个 block = 2 条指令。全程 BARE (M-mode + satp=0), 纯算术不碰
# load/store/csr。中断检查每 block 一次 → block=2 时每 2 条指令就查一次。
# sweep 12-16 工作指令同型 (add), 每指令成本恒定, 唯一变量是 block 大小。
# ITER_COUNT (s1 初值) 调到 release ~1s。

.section .text
.option norvc
.globl _start
_start:
    li      s0, 0                   # s0 = accumulator
    li      s1, 68000000            # s1 = ITER_COUNT (调这个值改长跑量; 调到 release ~1s)
    li      t0, 0x12345             # t0 = 常量

work_loop:
    .rept 0                         # WORK_N = block(2) - 2 = 0 条 add
    add     s0, s0, t0
    .endr
    addi    s1, s1, -1              # 计数 -- (block 第 1 条)
    bnez    s1, work_loop           # 硬边界 → block 收尾 (第 2 条)

    mv      a0, s0                  # a0 = 最终 acc

    # ---- halt: 清 mtvec + ecall 触发 triple fault ----
    csrw    mtvec, x0
    ecall

# ===== 期望结果 =====
# [perf] MIPS≈实测 (跑批读 median); [dispatcher] halted: in_trap=3
# a0 (x10) = 最终 acc, 跨运行恒定
# block 大小 = 2; total_count ≈ ITER_COUNT*2 + 设置/halt ~6
# 中断检查 ON/OFF 对照 + block 大小曲线见 tests/review/REVIEW.md
#
# ============================================================================
# 建立这个测试当下的实验结果记录
# (未来可能会过时, 但属于试验记录, 不用清理; 数值未来可能不同)
# ============================================================================
#
# 试验日期: AEST 2026-05-23   环境: release build (-O2, 无 ASan), 解释器, 单 hart
# 中断检查 A/B (交错 off/on 6 对取 median):
#   block 大小 = 2; total_count = 136000007
#   中断检查 ON  ≈ 129.2 MIPS   OFF ≈ 162.3 MIPS   → 检查占 ON 运行 ≈ 20.4%
#   观察: 最小 block, 检查占比 sweep 里最高 —— 每 2 条指令摊一次中断检查。跟
#   03_csr_heavy 的 21.9% 基本一致 (03 每轮 9 block / 10 inst, 平均 block ≈ 1.1,
#   比 block 2 还小)。
