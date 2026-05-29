# a02_7 / 01_bare
#
# ===== 运行参数 =====
# RUN-RELEASE
#
# ===== 测试目的 =====
# 解释器 perf 套件之一: REGIME_BARE 路径吞吐基线。M-mode bare 下纯 ALU+branch
# 紧循环, 不碰内存 / CSR / MMU —— 测"取指走 identity bypass、解释器执行纯计算
# 指令"的上限速度, 作其余 4 个 fixture 的对照基线 (01↔02 隔出 bare vs Sv32
# 取指开销; 01↔03/04/05 隔出 CSR / 访存 / walker 开销)。
#
# ===== 思路 =====
# 工作循环每轮 10 inst (8 ALU + addi + bnez); bnez 是块边界 → 每轮 1 block dispatch。
# 全程 BARE: cpu_create 默认 M-mode + satp=0, 取指走 IS_GPA_RAM identity, 不查 TLB。
# 纯 ALU 永不 trap, 测的是 fetch+decode+execute 的纯解释循环。
# 性能读 dispatcher [perf] 行 (elapsed / total_count / MIPS); AI 跑批连跑 5 次取中位。
# ITER_COUNT (s1 初值) 调长跑量 —— 目标 release ~1s。总 RV inst ≈ ITER_COUNT*10 + ~8。

.section .text
.option norvc
.globl _start
_start:
    li      s0, 0                   # s0 = accumulator
    li      s1, 18000000            # s1 = ITER_COUNT (调这个值改长跑量; 调到 release ~1s)
    li      t0, 0x12345             # t0 = 常量 (循环内不改)

work_loop:
    add     s0, s0, t0              # 8 条纯 ALU: add
    xor     s0, s0, s1              #            xor
    sub     t1, s0, t0              #            sub
    and     t2, t1, s0              #            and
    or      t3, t2, t1              #            or
    slli    t4, t3, 1               #            slli
    srli    t5, t4, 2               #            srli
    slt     t6, t5, s0              #            slt
    addi    s1, s1, -1              # counter--
    bnez    s1, work_loop           # 块边界 + 循环回跳

    mv      a0, s0                  # a0 = 最终 acc (跨运行一致, 验解释器结果确定)

    # ---- halt: 清 mtvec + ecall 触发 triple fault ----
    csrw    mtvec, x0
    ecall

# ===== 期望结果 =====
# [perf] elapsed≈1s  total_count≈ITER_COUNT*10  MIPS≈实测 (跑批读 median)
# [dispatcher] halted: in_trap=3  (csrw mtvec,x0 + ecall → triple fault)
# a0 (x10) = 最终 acc, 跨运行恒定 (纯 ALU, 无随机源)
#
# ============================================================================
# 建立这个测试当下的实验结果记录
# (未来可能会过时, 但属于试验记录, 不用清理; 数值未来可能不同)
# ============================================================================
#
# 试验日期: AEST 2026-05-23   环境: release build (-O2, 无 ASan), 解释器, 单 hart
# 跑批: 连跑 5 次取 median (a02_7 perf 套件基线; release + JIT 接入后跑同 fixture 对比)
#
#   [perf] elapsed≈1.03s  total_count=180000007  median MIPS ≈ 174
#   5 跑 MIPS: 173.9 / 174.0 / 174.2 / 175.0 / 178.3
#   bare 路径基线 —— 其余 4 个 fixture 的对照锚点。
