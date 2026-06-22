# b_03_6 / 01_smc_perf_high
#
# ===== 运行参数 =====
# RUN-RELEASE
#
# ===== 测试目的 =====
# perf 套件 · SMC chain 高频稳态开销 (b_03 T1.a + T4 综合量化):
# 每 outer iter 一次 sw 写 hot code page (写值任意, 写目标 = smc_target slot 不进
# 执行路径) → 触发 SIGSEGV → page_dirty bit_or → dispatcher 顶扫 dirty_pending
# counter → jit_invalidate_page (collect + invalidate + 状态灯扫 + backend release)
# → inner 下次入 lookup miss → 重 interpret 跨 THRESHOLD=100 → 再 JIT 编块.
# inner iter > THRESHOLD 让每 outer iter 都触发完整 SMC chain + 重编.
#
# 对照锚: a02_7/01_bare baseline (674 MIPS, 10 inst inner block + 无 SMC), 本 fixture
# 期望 MIPS 显著低 — SMC chain + recompile 主导 hot path. 跟 a02_7/01_bare 的 ratio
# 反映 SMC 每次 trigger 的端到端开销.
#
# ===== 思路 =====
# 双层循环:
#   outer (OUTER_ITER 次):
#     1. sw t6, 0(a7)      ← SMC trigger (写 smc_target slot, 跟 inner 同 4K page);
#                            mprotect R-only 让任何写都 SIGSEGV (b_03 T1.a 协议)
#     2. inner (INNER_ITER 次, > THRESHOLD=100):
#         addi t2, t2, 1
#         add  t3, t3, t2
#         xor  t4, t4, t2
#         sub  a4, a4, t2
#         or   a5, a5, t2
#         addi t5, t5, -1
#         bnez t5, inner          ← 块边界 (7 inst block)
#
# 寄存器选用 t2/t3/t4/a4/a5/t5 caller-saved 通用 reg, 避跟 s0/s1/sp (栈+outer 计数)
# 冲突. smc_target 放 .text 末未执行段, 跟 inner 同一 4K page (0x80000000-0x80001000)
# 让 sw 触发 mprotect SIGSEGV 命中同一 page bitmap.
#
# 块计数:
#   inner block: 7 inst, 重复 INNER_ITER 次
#   outer 块 (首次入 inner): sw + li t5 + (inner 首轮 7 inst) = 9 inst block
#   outer 尾: addi s1 + bnez s1 = 2 inst block
#
# 每 outer iter ≈ 9 + (INNER-1)*7 + 2 RV inst
#
# 设 OUTER=50000, INNER=200 → 每 outer iter ≈ 1404 inst, 总 ≈ 70M inst, 50000 次
# SMC trigger. SMC chain overhead 主导, 期望 release ~1s (单 SMC trigger 端到端
# 含 SIGSEGV+mprotect+invalidate+recompile 估 ~10-15μs, 50000 × ~12μs ≈ 0.6s).
#
# 注: 首次 outer iter 时 JIT 还没编块 (THRESHOLD=100 还没到), 首轮 sw 写 R/W 页 不
# SIGSEGV; inner 块达 THRESHOLD 后 JIT install + mprotect → 从第 2 outer iter 起每次
# sw 都 SIGSEGV. 1999/2000 = 99.95% trigger 率 ≈ 全量稳态.

.section .text
.option norvc
.globl _start
_start:
    lui     sp, 0x80300                # sp = 0x80300000
    li      s0, 0                      # acc placeholder
    li      s1, 50000                  # OUTER_ITER (调这个改长跑量; 50000 跑 release ~1s)
    la      a7, smc_target             # a7 = smc_target PA (BARE identity)
    li      t6, 0xdeadbeef             # SMC sw 写值 (任意值; 写 smc_target slot 不执行)

outer:
    sw      t6, 0(a7)                  # *** SMC trigger *** (page mprotect R-only → SIGSEGV)
    li      t5, 200                    # INNER_ITER (> THRESHOLD=100, JIT 编块 + reJIT)

inner:
    addi    t2, t2, 1                  # inner block 7 inst
    add     t3, t3, t2
    xor     t4, t4, t2
    sub     a4, a4, t2
    or      a5, a5, t2
    addi    t5, t5, -1                 # inner counter
    bnez    t5, inner                  # 块边界

    addi    s1, s1, -1                 # outer counter
    bnez    s1, outer                  # outer 块边界

    mv      a0, s0                     # a0 = acc (跨运行恒定)

    # ---- halt: 清 mtvec + ecall 触发 MDT abort (a02_7 perf 套件体例) ----
    csrw    mtvec, x0
    ecall

# ---- SMC sw 目标 slot (跟 inner 同 4K page, 不执行) ----
.align 4
smc_target:
    .word 0                            # 初值 0; sw 每 outer iter 写 0xdeadbeef

# ===== 期望结果 =====
# [perf] elapsed≈1s  total_count≈70M  MIPS≈? (release median × 5)
# 期望 MIPS 显著低于 a02_7/01_bare baseline (674 MIPS) — SMC chain + recompile 主导.
# t2 = 200*50000 = 10_000_000 (累加 INNER*OUTER 次 +1; 跨运行恒定)
# smc_target = 0xdeadbeef (最后 sw 后)
# 停机 = MDT abort exit 1
#
# 实测数据在 tests/review/REVIEW.md "b_03 收尾 perf — b03_06 JIT 视角 fixture" 段.
