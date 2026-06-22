# b_03_6 / 02_smc_perf_low
#
# ===== 运行参数 =====
# RUN-RELEASE
#
# ===== 测试目的 =====
# perf 套件 · SMC chain 低频稳态开销 — 跟 01_smc_perf_high 是 SMC 频率对照对:
#   01_smc_perf_high: SMC 每 ~1400 inst 一次 — SMC chain 主导
#   02_smc_perf_low : SMC 每 ~70K inst 一次 — JIT 稳态主导, SMC chain 摊薄
#
# 量化 SMC chain 在频率谱两端的开销曲线; 落 OS 偶尔 SMC (动态加载 / JIT runtime
# patching / Linux ftrace) 场景的稳态 perf 量级.
# 对照锚: a02_7/01_bare baseline (674 MIPS), 期望本 fixture MIPS 接近 baseline.
#
# ===== 思路 =====
# 跟 01 同构, 调比:
#   OUTER=500, INNER=100000  → 每 outer iter ≈ 700K inst, 500 次 SMC trigger
#   总 inst ≈ 350M (跟 01 70M 量级稍高, SMC trigger 数量低 100x 摊薄 chain 开销)
#
# 寄存器选用跟 01 同 (t2/t3/t4/a4/a5/t5), smc_target 同体例.

.section .text
.option norvc
.globl _start
_start:
    lui     sp, 0x80300                # sp = 0x80300000
    li      s0, 0                      # acc placeholder
    li      s1, 500                    # OUTER_ITER (低频; SMC 500 次)
    la      a7, smc_target
    li      t6, 0xdeadbeef             # SMC sw 写值 (任意)

outer:
    sw      t6, 0(a7)                  # SMC trigger (低频)
    li      t5, 100000                 # INNER_ITER (高, 摊薄 SMC chain)

inner:
    addi    t2, t2, 1                  # inner block 7 inst
    add     t3, t3, t2
    xor     t4, t4, t2
    sub     a4, a4, t2
    or      a5, a5, t2
    addi    t5, t5, -1
    bnez    t5, inner                  # 块边界

    addi    s1, s1, -1
    bnez    s1, outer

    mv      a0, s0

    csrw    mtvec, x0
    ecall

.align 4
smc_target:
    .word 0

# ===== 期望结果 =====
# [perf] elapsed≈1s  total_count≈350M  MIPS≈? (release median × 5)
# 期望 MIPS 接近 a02_7/01_bare baseline 量级 (~500-700 MIPS) — SMC trigger 数量
# 低 (500 次 vs 01 的 50000 次), 总开销 ~1/100 主导态.
# t2 = 100000*500 = 50_000_000 (跨运行恒定)
# 停机 = MDT abort exit 1
#
# 跟 01 对比 MIPS delta = SMC chain 频率曲线; 数据在 REVIEW.md b_03 收尾段.
