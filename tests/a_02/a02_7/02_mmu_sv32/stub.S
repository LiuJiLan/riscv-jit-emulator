# a02_7 / 02_mmu_sv32
#
# ===== 测试目的 =====
# 解释器 perf 套件之二: REGIME_SV32 取指路径吞吐。跟 01_bare 同一个纯 ALU+branch
# 紧循环, 但在 S-mode + Sv32 下跑 —— 每个 block 取指经叶 TLB 查找 (命中态)。
# 01↔02 的 MIPS 差 = "Sv32 取指翻译 (TLB 命中) 相对 bare identity 的开销"。
#
# ===== 思路 =====
# M-mode 起步: 设 mtvec / 建页表 (main.c setup_pt) / 写 satp + sfence.vma / mret 切 S。
# PT (main.c): root_pt[0x200]→leaf_code, leaf_code[0]=代码页 identity V+R+X+A;
#   satp=0x80080100 (Sv32 + PPN=0x80100, root_pt @ 0x80100000)。
# S phase: 跟 01_bare 逐字相同的 10-inst/轮循环 (8 ALU + addi + bnez)。代码全在
#   首个 4KB 页内、循环体小 → 叶 TLB 取指首轮 fill、之后恒命中。
# 收尾 ecall (cause 9 from S) → m_handler → csrw mtvec,x0 + ecall → triple fault。
# ITER_COUNT (s1 初值) 调长跑量, 目标 release ~1s; 性能读 [perf] 行, 跑批取中位。

.section .text
.option norvc
.globl _start
_start:
    lui  sp, 0x80300                # M-mode 栈 (setup_pt 在 -O0 下用栈; S 循环不碰)

    la   x4, m_handler
    csrw mtvec, x4

    call setup_pt                   # 建 Sv32 页表 (M-mode bare 下写 RAM)

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
    li      s0, 0                   # s0 = accumulator
    li      s1, 18000000            # s1 = ITER_COUNT (调这个值改长跑量; 调到 release ~1s)
    li      t0, 0x12345             # t0 = 常量 (循环内不改)

work_loop:
    add     s0, s0, t0              # 8 条纯 ALU (跟 01_bare 逐字相同)
    xor     s0, s0, s1
    sub     t1, s0, t0
    and     t2, t1, s0
    or      t3, t2, t1
    slli    t4, t3, 1
    srli    t5, t4, 2
    slt     t6, t5, s0
    addi    s1, s1, -1              # counter--
    bnez    s1, work_loop           # 块边界 + 循环回跳

    mv      a0, s0                  # a0 = 最终 acc (跨运行一致)
    ecall                           # S→M (cause 9)

m_handler:
    csrw    mtvec, x0
    ecall                           # M, cause 11 → triple fault

# ===== 期望结果 =====
# [perf] elapsed≈1s  total_count≈ITER_COUNT*10 + 预备/收尾 ~20  MIPS≈实测
# [dispatcher] halted: in_trap=3  (csrw mtvec,x0 + ecall → triple fault)
# a0 (x10) = 最终 acc, 跨运行恒定
#
# ============================================================================
# 建立这个测试当下的实验结果记录
# (未来可能会过时, 但属于试验记录, 不用清理; 数值未来可能不同)
# ============================================================================
#
# 试验日期: AEST 2026-05-23   环境: release build (-O2, 无 ASan), 解释器, 单 hart
# 跑批: 连跑 5 次取 median
#
#   [perf] elapsed≈1.03s  total_count=180000045  median MIPS ≈ 176
#   5 跑 MIPS: 169.3 / 170.7 / 175.9 / 176.3 / 179.0
#   观察: 跟 01_bare (≈174) 基本持平 —— 同循环, Sv32 取指 TLB 命中的查找开销落在
#   测量噪声内 (说明叶 TLB 命中快路径确实廉价)。
