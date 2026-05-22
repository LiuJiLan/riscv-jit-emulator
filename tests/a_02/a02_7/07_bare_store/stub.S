# a02_7 / 07_bare_store
#
# ===== 测试目的 =====
# perf 套件 3×2 之一 (bare/密集mmu/稀疏mmu × 纯load/纯store; fixture 06-11)。M-mode
# bare 下每轮 8 条 sw 打同
# 一热地址 —— 测 store slow path (store_helper: LR/SC reservation + 未来 SMC 副作用
# 入口) 的吞吐。06↔07 隔出 load fast path vs store slow path 的不对称。
#
# ===== 思路 =====
# 每轮 10 inst (8 sw + addi + bnez), 跟 06_bare_load 同形状 → MIPS 可直比。
#   load 命中走 *hva inline, store 命中仍走 store_helper —— 这条不对称是 by design
#   (dummy.txt §1 末段 / lsu.h 顶段)。
# ITER_COUNT (s1 初值) 调到 release ~1s; 性能读 [perf] 行, 跑批取中位。
#
# 注 (观察点 — store 成本目前是下界): store_helper 的 slow-path 成本尚未包含真正的
#   LR/SC reservation 管理 (LR/SC 在 isa/amo.c, 推迟未实装) 与 SMC 副作用 (smc 子
#   系统未实装) —— store_helper 顶注里这两项现在是"入口预留"。所以本套件测到的
#   load/store 差距是个下界; 将来 LR/SC + SMC 落地后每条 store 多做事, 差距会进一步
#   拉大。这个观察点在纯 store fixture (07/09/11) 都标注。
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
    li      s1, 12000000            # s1 = ITER_COUNT (调这个值改长跑量; 调到 release ~1s)
    li      s2, 0x80010000          # s2 = 热数据地址 (单 word, 远离 .text)
    li      t0, 0x0ABCDEF0          # t0 = 待存值

work_loop:
    sw      t0, 0(s2)               # 8 条 sw — 每条走 store slow path (store_helper)
    sw      t0, 0(s2)
    sw      t0, 0(s2)
    sw      t0, 0(s2)
    sw      t0, 0(s2)
    sw      t0, 0(s2)
    sw      t0, 0(s2)
    sw      t0, 0(s2)
    addi    s1, s1, -1              # counter--
    bnez    s1, work_loop           # 块边界 + 循环回跳

    mv      a0, t0                  # a0 = 待存值 (sanity)

    # ---- halt: 清 mtvec + ecall 触发 triple fault ----
    csrw    mtvec, x0
    ecall

# ===== 期望结果 =====
# [perf] MIPS≈实测 (跑批读 median); [dispatcher] halted: in_trap=3
# a0 (x10) = 0x0ABCDEF0
# MIPS 应低于 06_bare_load (store slow path vs load fast path)
#
# ============================================================================
# 建立这个测试当下的实验结果记录
# (未来可能会过时, 但属于试验记录, 不用清理; 数值未来可能不同)
# ============================================================================
#
# 试验日期: AEST 2026-05-23   环境: release build (-O2, 无 ASan), 解释器, 单 hart
# 跑批: 连跑 5 次取 median
#
#   [perf] elapsed≈0.96s  total_count=120000007  median MIPS ≈ 125
#   5 跑 MIPS: 122.5 / 123.5 / 125.4 / 125.5 / 127.1
#
#   bare/密集mmu/稀疏mmu × 纯load/纯store 3×2 (同期 median MIPS):
#                  纯 load   纯 store
#       bare          203       125
#       密集 mmu      175       121
#       稀疏 mmu       99        87
#   观察: bare store 125 ≈ bare load 203 的 0.62x —— store 命中仍走 store_helper
#   (slow path), load 走 *hva inline (fast path); load/store 不对称 by design
#   (dummy.txt §1 末段 / 决策表 #23), 这里实测量化。store 那一列 125→121→87: 密集
#   几乎不掉 (store_helper 主成本掩盖 TLB 命中查找), 稀疏才明显掉。见思路段 LR/SC
#   观察点 —— 这一列将来会整体下移。
