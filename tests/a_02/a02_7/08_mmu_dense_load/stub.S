# a02_7 / 08_mmu_dense_load
#
# ===== 运行参数 =====
# RUN-RELEASE
#
# ===== 测试目的 =====
# perf 套件 3×2 之一 (bare/密集mmu/稀疏mmu × 纯load/纯store; fixture 06-11)。
# S-mode + Sv32 下每轮 8 条 lw 打同一热数据页 (叶 TLB 命中, "密集" = 单热页) ——
# 测 SV32 load 命中路径吞吐。06↔08 隔出 bare identity vs Sv32 TLB 命中; 08↔10
# 隔出 TLB 命中 vs miss。
#
# ===== 思路 =====
# M 起步: setup_pt + satp + sfence + mret 切 S (跟 02_mmu_sv32 同)。PT (main.c)
#   映射代码页 + 数据页 0x80010000 (leaf_code[16])。
# S phase 每轮 10 inst (8 lw + addi + bnez), 跟 06_bare_load 逐字相同的循环体, 仅
#   regime 不同。8 次访同一数据页 → 叶 TLB 首轮 fill、之后恒命中。
# 收尾 ecall (cause 9 from S) → m_handler → triple fault。
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
    lui  sp, 0x80300                # M-mode 栈 (setup_pt 在 -O0 下用栈)

    la   x4, m_handler
    csrw mtvec, x4

    call setup_pt                   # 建 Sv32 页表 (代码页 + 数据页)

    lui  x6, 0x80080
    addi x6, x6, 0x100              # satp = 0x80080100 (Sv32 + PPN=0x80100)
    csrw satp, x6
    sfence.vma

    la   x7, s_loop_entry
    csrw mepc, x7
    addi x4, x0, 1
    slli x4, x4, 11                 # mstatus.MPP = S
    csrw mstatus, x4
    mret                            # M→S, 进 Sv32

s_loop_entry:
    li      s1, 16500000            # s1 = ITER_COUNT (调这个值改长跑量; 调到 release ~1s)
    li      s2, 0x80010000          # s2 = 热数据地址 (映射在 leaf_code[16])

work_loop:
    lw      t0, 0(s2)               # 8 条 lw — SV32 load, 叶 TLB 命中
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
    ecall                           # S→M (cause 9)

m_handler:
    csrw    mtvec, x0
    ecall                           # M, cause 11 → triple fault

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
#   [perf] elapsed≈0.95s  total_count=165000048  median MIPS ≈ 175
#   5 跑 MIPS: 171.4 / 171.5 / 174.5 / 175.2 / 175.6
#
#   bare/密集mmu/稀疏mmu × 纯load/纯store 3×2 (同期 median MIPS):
#                  纯 load   纯 store
#       bare          203       125
#       密集 mmu      175       121
#       稀疏 mmu       99        87
#   观察: 密集 mmu load 175 vs bare load 203 → Sv32 叶 TLB 命中查找在 load 上约
#   14% 开销。load fast path 本身便宜, TLB 查找的占比才显出来。
