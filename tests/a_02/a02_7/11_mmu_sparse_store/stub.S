# a02_7 / 11_mmu_sparse_store
#
# ===== 测试目的 =====
# perf 套件 3×2 之一 (bare/密集mmu/稀疏mmu × 纯load/纯store; fixture 06-11)。
# S-mode + Sv32 下每轮 8 条 sw 打 8 个故意撞同一叶 TLB index 的页 ("稀疏") ——
# 稳态下每条 sw 都 TLB miss → mmu_walk 重走 + store_helper。09↔11 隔出 TLB 命中
# vs miss 在 store 上的开销; 10↔11 隔出稀疏下 load vs store 的不对称。
#
# ===== 思路 =====
# M 起步: setup_pt + satp + sfence + mret 切 S。PT (main.c): 代码页 + 8 个数据页
#   (vaddr 0x80401000 + k*0x40000, 每页 256KB 间隔)。8 页 VPN 低 6 位都 = 1 →
#   全撞 direct-mapped 叶 TLB 的 index 1; 代码页撞 index 0 (取指 TLB 不被挤)。
# S phase 每轮 10 inst (8 sw + addi + bnez), 跟 07/09 逐字同形状。8 条 sw 顺序访
#   8 个 index-1 页 → 每条都把上一条挤出 TLB → 条条 miss → 条条 mmu_walk 两级重走。
# 收尾 ecall (cause 9 from S) → m_handler → triple fault。
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
    lui  sp, 0x80300                # M-mode 栈 (setup_pt 在 -O0 下用栈)

    la   x4, m_handler
    csrw mtvec, x4

    call setup_pt                   # 建 Sv32 页表 (代码页 + 8 个数据页)

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
    # s2..s9 = 8 个数据页指针 (每页相距 256KB → VPN 低 6 位全 = 1 → 撞同一 TLB index)
    li      s2, 0x80401000
    li      s3, 0x80441000
    li      s4, 0x80481000
    li      s5, 0x804C1000
    li      s6, 0x80501000
    li      s7, 0x80541000
    li      s8, 0x80581000
    li      s9, 0x805C1000
    li      s1, 8600000             # s1 = ITER_COUNT (调这个值改长跑量; 调到 release ~1s)
    li      t0, 0x0ABCDEF0          # t0 = 待存值

work_loop:
    sw      t0, 0(s2)               # 8 条 sw 各访不同页 → direct-mapped 叶 TLB 条条 miss
    sw      t0, 0(s3)
    sw      t0, 0(s4)
    sw      t0, 0(s5)
    sw      t0, 0(s6)
    sw      t0, 0(s7)
    sw      t0, 0(s8)
    sw      t0, 0(s9)
    addi    s1, s1, -1              # counter--
    bnez    s1, work_loop           # 块边界 + 循环回跳

    mv      a0, t0                  # a0 = 待存值 (sanity)
    ecall                           # S→M (cause 9)

m_handler:
    csrw    mtvec, x0
    ecall                           # M, cause 11 → triple fault

# ===== 期望结果 =====
# [perf] MIPS≈实测 (跑批读 median); [dispatcher] halted: in_trap=3
# a0 (x10) = 0x0ABCDEF0
# MIPS 应低于 10_mmu_sparse_load (store slow path) 且低于 09_mmu_dense_store (条条 miss)
#
# ============================================================================
# 建立这个测试当下的实验结果记录
# (未来可能会过时, 但属于试验记录, 不用清理; 数值未来可能不同)
# ============================================================================
#
# 试验日期: AEST 2026-05-23   环境: release build (-O2, 无 ASan), 解释器, 单 hart
# 跑批: 连跑 5 次取 median
#
#   [perf] elapsed≈0.99s  total_count=86000259  median MIPS ≈ 87
#   5 跑 MIPS: 85.5 / 86.9 / 87.3 / 88.1 / 89.0
#
#   bare/密集mmu/稀疏mmu × 纯load/纯store 3×2 (同期 median MIPS):
#                  纯 load   纯 store
#       bare          203       125
#       密集 mmu      175       121
#       稀疏 mmu       99        87
#   观察: 稀疏 mmu store 87 vs 密集 121 → 条条 miss 砍 store ~28%。注意 load/store
#   差距随 regime 收窄: bare 203/125=1.6x → 密集 175/121=1.4x → 稀疏 99/87=1.14x ——
#   mmu_walk 是固定开销, 加在便宜的 load 上占比更大, 把 load 拉向 store。
#   (思路段 LR/SC 观察点: store 列将来引入 reservation 后整体下移, 差距又会反向拉开。)
