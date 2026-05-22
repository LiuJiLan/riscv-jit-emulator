# a02_7 / 06_bare_load
#
# ===== 测试目的 =====
# perf 套件 3×2 之一 (bare/密集mmu/稀疏mmu × 纯load/纯store; fixture 06-11)。M-mode
# bare 下每轮 8 条 lw 打同
# 一热地址 —— 测 load fast path (BARE IS_GPA_RAM 直 *hva inline) 的吞吐。
#
# ===== 思路 =====
# 每轮 10 inst (8 lw + addi + bnez), 跟 01_bare (8 ALU) / 03_csr_heavy (8 csrr)
#   同形状 → MIPS 可跨 fixture 直比。热地址单 word 0x80010000, 宿主 cache 常驻,
#   测的是解释器 load 指令本身。
# ITER_COUNT (s1 初值) 调到 release ~1s; 性能读 [perf] 行, 跑批取中位。
#
# 3×2 矩阵 (六个 fixture 循环体逐字一致 = 8 访存 + addi + bnez, 仅 regime/访存类型不同):
#                纯 load              纯 store
#   bare         06_bare_load         07_bare_store
#   密集 mmu     08_mmu_dense_load    09_mmu_dense_store
#   稀疏 mmu     10_mmu_sparse_load   11_mmu_sparse_store

.section .text
.option norvc
.globl _start
_start:
    li      s1, 19000000            # s1 = ITER_COUNT (调这个值改长跑量; 调到 release ~1s)
    li      s2, 0x80010000          # s2 = 热数据地址 (单 word, 远离 .text)

work_loop:
    lw      t0, 0(s2)               # 8 条 lw — 每条走 BARE load fast path (*hva inline)
    lw      t0, 0(s2)
    lw      t0, 0(s2)
    lw      t0, 0(s2)
    lw      t0, 0(s2)
    lw      t0, 0(s2)
    lw      t0, 0(s2)
    lw      t0, 0(s2)
    addi    s1, s1, -1              # counter--
    bnez    s1, work_loop           # 块边界 + 循环回跳

    mv      a0, t0                  # a0 = 最后一次 load 值

    # ---- halt: 清 mtvec + ecall 触发 triple fault ----
    csrw    mtvec, x0
    ecall

# ===== 期望结果 =====
# [perf] MIPS≈实测 (跑批读 median); [dispatcher] halted: in_trap=3
# a0 (x10) = 0  (热区匿名 RAM 清零, lw 读出 0)
#
# ============================================================================
# 建立这个测试当下的实验结果记录
# (未来可能会过时, 但属于试验记录, 不用清理; 数值未来可能不同)
# ============================================================================
#
# 试验日期: AEST 2026-05-23   环境: release build (-O2, 无 ASan), 解释器, 单 hart
# 跑批: 连跑 5 次取 median
#
#   [perf] elapsed≈0.94s  total_count=190000005  median MIPS ≈ 203
#   5 跑 MIPS: 193.2 / 195.9 / 203.2 / 204.0 / 205.8
#
#   bare/密集mmu/稀疏mmu × 纯load/纯store 3×2 (同期 median MIPS):
#                  纯 load   纯 store
#       bare          203       125
#       密集 mmu      175       121
#       稀疏 mmu       99        87
#   观察: 本格 (bare load) 是 load fast path 上限。load 那一列 bare→密集→稀疏 =
#   203→175→99: load 强烈受 regime/TLB 影响 —— fast path (*hva inline) 本身便宜,
#   任何叠加开销占比都显眼 (略高于 01_bare 的 8 ALU 也因 8 条同型 lw 让宿主分支
#   预测器对解释器 dispatch 命中更稳)。
