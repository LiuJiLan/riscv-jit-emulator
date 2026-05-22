# a02_7 / 09_mmu_dense_store
#
# ===== 测试目的 =====
# perf 套件 3×2 之一 (bare/密集mmu/稀疏mmu × 纯load/纯store; fixture 06-11)。
# S-mode + Sv32 下每轮 8 条 sw 打同一热数据页 (叶 TLB 命中, "密集" = 单热页) ——
# 测 SV32 store 命中路径吞吐。07↔09 隔出 bare vs Sv32 在 store 上的开销; 08↔09
# 隔出 SV32 下 load vs store 的不对称; 09↔11 隔出 TLB 命中 vs miss。
#
# ===== 思路 =====
# M 起步: setup_pt + satp + sfence + mret 切 S。PT (main.c) 映射代码页 + 数据页
#   0x80010000 (leaf_code[16], W+D 置位)。
# S phase 每轮 10 inst (8 sw + addi + bnez), 跟 07_bare_store 逐字相同的循环体,
#   仅 regime 不同。store 命中仍走 store_helper (slow path); 8 次同页 → 叶 TLB 恒命中。
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
    li      s1, 12000000            # s1 = ITER_COUNT (调这个值改长跑量; 调到 release ~1s)
    li      s2, 0x80010000          # s2 = 热数据地址 (映射在 leaf_code[16])
    li      t0, 0x0ABCDEF0          # t0 = 待存值

work_loop:
    sw      t0, 0(s2)               # 8 条 sw — SV32 store, 叶 TLB 命中, 仍走 store_helper
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
    ecall                           # S→M (cause 9)

m_handler:
    csrw    mtvec, x0
    ecall                           # M, cause 11 → triple fault

# ===== 期望结果 =====
# [perf] MIPS≈实测 (跑批读 median); [dispatcher] halted: in_trap=3
# a0 (x10) = 0x0ABCDEF0
# MIPS 应低于 08_mmu_dense_load (store slow path vs load fast path)
#
# ============================================================================
# 建立这个测试当下的实验结果记录
# (未来可能会过时, 但属于试验记录, 不用清理; 数值未来可能不同)
# ============================================================================
#
# 试验日期: AEST 2026-05-23   环境: release build (-O2, 无 ASan), 解释器, 单 hart
# 跑批: 连跑 5 次取 median
#
#   [perf] elapsed≈0.99s  total_count=120000050  median MIPS ≈ 121
#   5 跑 MIPS: 117.7 / 121.1 / 121.4 / 121.9 / 122.7
#
#   bare/密集mmu/稀疏mmu × 纯load/纯store 3×2 (同期 median MIPS):
#                  纯 load   纯 store
#       bare          203       125
#       密集 mmu      175       121
#       稀疏 mmu       99        87
#   观察: 密集 mmu store 121 ≈ bare store 125 —— store 几乎不受 regime 影响 (对比
#   load 列 203→175 明显掉)。store_helper slow path 主成本掩盖了 TLB 命中查找开销。
