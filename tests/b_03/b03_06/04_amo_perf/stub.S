# b_03_6 / 04_amo_perf
#
# ===== 运行参数 =====
# RUN-RELEASE
#
# ===== 测试目的 =====
# perf 套件 · AMO 稳态调用开销 (a_04 落地后 b_03 收尾补 perf trail):
# a_04 加 RV32 Zaamo 9 op (amoadd/swap/xor/or/and 走 C11 atomic_fetch_* + min/max
# CAS loop) 走 3-layer 模型 (顶层 inline + apply + walker_helper). JIT 块内
# emit `call amo_*_walker_helper` 跟 store/load 同体例 slow path. b_03 修
# dirty_pending counter 后 hot path 不再退化, 此时量化 AMO helper call 在稳态
# JIT 块内开销 — 落 a_04 perf trail 收尾 + 跟 a02_7/07_bare_store (118 MIPS)
# / 09_mmu_dense_store (114 MIPS) 体例对比 (AMO 比 store 多一次 atomic_fetch_add).
#
# 对照锚:
#   a02_7/01_bare       674 MIPS  纯 ALU 10 inst block (无 store/AMO 慢路径)
#   a02_7/07_bare_store 118 MIPS  bare store 紧循环 (store_helper call 主导)
# 期望本 fixture 比 07 略慢 (AMO multi-cycle atomic + load-store-pair).
#
# ===== 思路 =====
# 循环体 = 7 条 ALU + amoadd.w + addi + bnez = 10 inst
#   block A: 7 ALU + amoadd.w (amoadd 不是块边界, store_helper 体例 emit call) +
#            addi + bnez (块边界) = 10 inst 1 块 (跟 a02_7/01_bare 同结构)
# 跟 a02_7/01_bare 的 delta = AMO helper call 开销 (atomic_fetch_add release).
#
# AMO 目标 = atomic_var (RAM 内, .data 段 link.ld 默认放 .text 之后).
# amoadd.w t1, t0, 0(a7) — t1 接老值 (放弃), t0 加值, a7 是 atomic_var 地址.
# ITER_COUNT 调到 release ~1s 量级.

.section .text
.option norvc
.globl _start
_start:
    li      s0, 0
    li      s1, 7000000                # ITER_COUNT (调到 release ~1s; smoke 30M 跑了 4.6s @ 65 MIPS)
    li      t0, 0x12345
    la      a7, atomic_var

work_loop:
    add     s0, s0, t0                 # 7 ALU
    xor     s0, s0, t0
    sub     s0, s0, t0
    and     s0, s0, t0
    or      s0, s0, t0
    slli    s0, s0, 1
    srli    s0, s0, 1
    amoadd.w t1, t0, 0(a7)             # AMO slow path: walker_helper + atomic_fetch_add
    addi    s1, s1, -1
    bnez    s1, work_loop              # 块边界 + 回跳

    mv      a0, s0

    csrw    mtvec, x0
    ecall

.section .data
.align 4
atomic_var:
    .word 0                            # AMO target; ITER 次 atomic_fetch_add(0x12345)

# ===== 期望结果 =====
# [perf] elapsed≈1s  total_count≈70M  MIPS≈? (release median × 5)
# 期望 MIPS 介于 a02_7/01_bare (674, 全 ALU) 跟 07_bare_store (118, store 紧循环)
# 之间, 量级跟 store 系列接近 (AMO 跟 store 同走 walker_helper slow path).
#
# atomic_var 末值 = ITER_COUNT * 0x12345 = 7M * 74565 = 521,955,000,000 (mod 2^32)
# (跨运行恒定; 实际计算时再确认).
# 停机 = MDT abort exit 1
