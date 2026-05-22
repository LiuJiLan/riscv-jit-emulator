# a02_7 / 04_mem_dense
#
# ===== 测试目的 =====
# 解释器 perf 套件之四: 访存密集路径吞吐。M-mode bare 下每轮 6 条访存 (3 sw +
# 3 lw) 打一个小热工作集。04↔01_bare 隔出访存路径开销; lw 走 fast path (BARE
# IS_GPA_RAM 直 *hva), sw 走 slow path store_helper。
#
# ===== 思路 =====
# 工作循环每轮 9 inst: 3 sw + 3 lw + add + addi + bnez。
# 热工作集 = heap 0x80010000 起 3 个 word (远离 .text), 恒命中宿主 cache —— 测
#   的是解释器访存指令本身的开销, 不是宿主内存延迟。
# BARE 模式: lw 命中 IS_GPA_RAM 直接 *hva inline (fast path); sw 走 store_helper
#   (slow path, LR/SC reservation + 未来 SMC 副作用入口)。
# ITER_COUNT (s1 初值) 调长跑量, 目标 release ~1s; 性能读 [perf] 行, 跑批取中位。

.section .text
.option norvc
.globl _start
_start:
    li      s0, 0                   # s0 = accumulator
    li      s1, 17000000            # s1 = ITER_COUNT (调这个值改长跑量; 调到 release ~1s)
    li      s2, 0x80010000          # s2 = heap base (远离 .text)

work_loop:
    sw      s0, 0(s2)               # 3 sw (slow path store_helper)
    lw      t0, 0(s2)               # 3 lw (fast path BARE *hva)
    sw      t0, 4(s2)
    lw      t1, 4(s2)
    sw      t1, 8(s2)
    lw      t2, 8(s2)
    add     s0, t2, s1              # acc 混入 counter
    addi    s1, s1, -1              # counter--
    bnez    s1, work_loop           # 块边界 + 循环回跳

    mv      a0, s0                  # a0 = 最终 acc (跨运行一致)

    # ---- halt: 清 mtvec + ecall 触发 triple fault ----
    csrw    mtvec, x0
    ecall

# ===== 期望结果 =====
# [perf] elapsed≈1s  total_count≈ITER_COUNT*9 + 预备/收尾 ~9  MIPS≈实测
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
#   [perf] elapsed≈0.97s  total_count=153000006  median MIPS ≈ 157
#   5 跑 MIPS: 156.6 / 156.7 / 156.8 / 158.6 / 161.3
#   观察: 低于 01_bare (≈174) —— 每轮 9 inst 含 6 条访存 (3 sw slow path +
#   3 lw fast path), 访存指令拖低平均吞吐。
