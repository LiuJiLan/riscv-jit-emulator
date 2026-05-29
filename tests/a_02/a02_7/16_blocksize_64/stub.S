# a02_7 / 16_blocksize_64
#
# ===== 运行参数 =====
# RUN-RELEASE
#
# ===== 测试目的 =====
# perf 套件·block 大小 sweep (fixture 12-16): bare M-mode 纯算术紧循环, 每个 block
# 固定 64 条指令 (= config.h BLOCK_INST_LIMIT 软边界上限)。配 中断检查 ON/OFF A/B
# (见 REVIEW.md), 测"每 block 一次的中断检查开销"如何随 block 大小变化 —— 本格是
# 最大 block, 检查占比应最低。
#
# ===== 思路 =====
# 循环体 = 62 条 add (WORK_N = block-2 = 62) + addi(计数) + bnez。bnez 在第 64 条
# 是硬边界; 64 = BLOCK_INST_LIMIT, interpreter fetch loop while(count<64) 恰好放完
# 64 条、第 64 条 (bnez) 照常执行并收块 → 每轮 = 1 个 block = 64 条指令。
# 全程 BARE (M-mode + satp=0), 纯算术不碰 load/store/csr。中断检查每 block 一次
# → block=64 时每 64 条指令才查一次 (sweep 里检查最稀)。
# sweep 12-16 工作指令同型 (add), 每指令成本恒定, 唯一变量是 block 大小。
# ITER_COUNT (s1 初值) 调到 release ~1s。

.section .text
.option norvc
.globl _start
_start:
    li      s0, 0                   # s0 = accumulator
    li      s1, 3400000             # s1 = ITER_COUNT (调这个值改长跑量; 调到 release ~1s)
    li      t0, 0x12345             # t0 = 常量

work_loop:
    .rept 62                        # WORK_N = block(64) - 2 = 62 条 add
    add     s0, s0, t0
    .endr
    addi    s1, s1, -1              # 计数 -- (block 第 63 条)
    bnez    s1, work_loop           # 硬边界 → block 收尾 (第 64 条; 恰 BLOCK_INST_LIMIT)

    mv      a0, s0                  # a0 = 最终 acc

    # ---- halt: 清 mtvec + ecall 触发 triple fault ----
    csrw    mtvec, x0
    ecall

# ===== 期望结果 =====
# [perf] MIPS≈实测 (跑批读 median); [dispatcher] halted: in_trap=3
# a0 (x10) = 最终 acc, 跨运行恒定
# block 大小 = 64; total_count ≈ ITER_COUNT*64 + 设置/halt ~6
# 中断检查 ON/OFF 对照 + block 大小曲线见 tests/review/REVIEW.md
#
# ============================================================================
# 建立这个测试当下的实验结果记录
# (未来可能会过时, 但属于试验记录, 不用清理; 数值未来可能不同)
# ============================================================================
#
# 试验日期: AEST 2026-05-23   环境: release build (-O2, 无 ASan), 解释器, 单 hart
# 中断检查 A/B (交错 off/on 6 对取 median):
#   block 大小 = 64 (= BLOCK_INST_LIMIT 上限); total_count = 217600007
#   中断检查 ON  ≈ 226.5 MIPS   OFF ≈ 234.1 MIPS   → 检查占 ON 运行 ≈ 3.2%
#   观察: 最大 block, 检查占比 sweep 里最低 —— 固定检查被 64 条指令摊薄到 ~3%。
