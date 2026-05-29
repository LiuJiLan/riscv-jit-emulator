# a02_7 / 05_mem_tlbmiss
#
# ===== 运行参数 =====
# RUN-RELEASE
#
# ===== 测试目的 =====
# 解释器 perf 套件之五: 访存松散、故意制造 TLB miss。S-mode + Sv32 下用步进跨页
# 的 load 模式扫过 256 个数据页 (> 64-entry 叶 TLB), 稳态下每次 load 都 TLB miss
# → mmu_walk 重走。05↔04_mem_dense 隔出"页表 walker 重走"的开销。
#
# ===== 思路 =====
# M-mode 起步: 设 mtvec / 建页表 (main.c setup_pt) / 写 satp + sfence.vma / mret 切 S。
# PT (main.c): 代码段 1 页 + 数据段 256 个 4KB 页 identity 映射 (vaddr 0x80400000
#   起 1MB, root_pt[0x201]→leaf_data[0..255])。
# S phase 工作循环每轮 6 inst: 指针每轮 +4KB 步进、扫满 1MB 后 branchless wrap
#   (and 掩码)。256 页 > 64-entry direct-mapped 叶 TLB → 稳态每次 lw 都 miss →
#   走 mmu_walker_helper_load → mmu_walk 两级翻译 (root PTE + leaf PTE 各读一次)。
# 收尾 ecall (cause 9 from S) → m_handler → csrw mtvec,x0 + ecall → triple fault。
# ITER_COUNT (s1 初值) 调长跑量, 目标 release ~1s; 性能读 [perf] 行, 跑批取中位。

.section .text
.option norvc
.globl _start
_start:
    lui  sp, 0x80300                # M-mode 栈 (setup_pt 在 -O0 下用栈; S 循环不碰)

    la   x4, m_handler
    csrw mtvec, x4

    call setup_pt                   # 建 Sv32 页表 (代码 1 页 + 数据 256 页)

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
    li      s3, 0x80400000          # s3 = 数据区虚拟基址
    li      s4, 0x000FF000          # s4 = 1MB wrap 掩码 (256 页, 页对齐)
    li      s5, 0x1000              # s5 = 页步进 4096 (超 addi 立即数范围, 入寄存器)
    li      s2, 0                   # s2 = 当前页内偏移
    li      s1, 23000000            # s1 = ITER_COUNT (调这个值改长跑量; 调到 release ~1s)

work_loop:
    add     t0, s3, s2              # t0 = 数据指针 (base + offset)
    lw      t1, 0(t0)               # load — 故意 TLB miss (256 页 thrash 64-entry TLB)
    add     s2, s2, s5              # 下一页 (+4096)
    and     s2, s2, s4              # wrap within 1MB (branchless)
    addi    s1, s1, -1              # counter--
    bnez    s1, work_loop           # 块边界 + 循环回跳

    mv      a0, t1                  # a0 = 最后一次 load 值
    ecall                           # S→M (cause 9)

m_handler:
    csrw    mtvec, x0
    ecall                           # M, cause 11 → triple fault

# ===== 期望结果 =====
# [perf] elapsed≈1s  total_count≈ITER_COUNT*6 + 预备/收尾 ~25  MIPS≈实测
# [dispatcher] halted: in_trap=3  (csrw mtvec,x0 + ecall → triple fault)
# a0 (x10) = 0  (数据区匿名 RAM 清零, lw 读出 0)
#
# ============================================================================
# 建立这个测试当下的实验结果记录
# (未来可能会过时, 但属于试验记录, 不用清理; 数值未来可能不同)
# ============================================================================
#
# 试验日期: AEST 2026-05-23   环境: release build (-O2, 无 ASan), 解释器, 单 hart
# 跑批: 连跑 5 次取 median
#
#   [perf] elapsed≈0.93s  total_count=138004925  median MIPS ≈ 149
#   5 跑 MIPS: 141.3 / 148.3 / 148.6 / 149.0 / 149.5
#   观察: 低于 02_mmu_sv32 (同 Sv32 但 TLB 命中态) —— 每次 lw 触发 mmu_walk 两级
#   重走的开销可见; 但 guest 页表小、宿主 cache 常驻, walk 本身不算贵。
#   注: total_count 比 ITER*6 多 ~4900 = setup_pt 填 256 个叶 PTE 的 C 循环 (-O0)。
