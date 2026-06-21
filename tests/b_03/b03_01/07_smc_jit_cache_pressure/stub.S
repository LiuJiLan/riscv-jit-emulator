# b_03_1 / 07_smc_jit_cache_pressure
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验 b_03 T1.a SMC chain **多 page + 多次 SMC trigger 联动** (6 axes (v) 部分,
# (vi) 部分):
#   - 3 个 hot_loop 跨 page A/B/C; 各编 JIT (跨 THRESHOLD)
#   - sw 改 page B 1 次 + sw 改 page C 1 次 (2 次独立 SMC trigger)
#   - 期望: dispatcher 顶扫多次 invalidate (不止 1 page); page A JIT 块全程 hit
#     不被错误清; B/C 走 interpret 跑改后代码
#
# 比 fixture 06 (2 page, 1 SMC) 复杂度增量: 3 page + 2 SMC trigger; 验 dispatcher
# 顶扫 while loop 真处理多 page_dirty bit (不只一次).
#
# **不在本 fixture 验**: Flush + retry + BLACK entry collect NULL skip 路径 —
# 需要 src 改 JIT_CACHE_SIZE 调小 (file-static 宏, fixture 改不到). 推 T_收尾归
# 零时配 src 改一起做.
#
# 期望 x18 = 200(A,+1) + 200(B,+1) + 200(C,+1) + 100(A 重 hit JIT,+1) +
#            100(B 改后 interpret,+10) + 100(C 改后 interpret,+10)
#         = 600 + 100 + 1000 + 1000 = 2700

.section .text
.option norvc
.globl _start
_start:
    lui  sp, 0x80300

    li   x18, 0                     # accumulator (s2)

    # ---- phase 1: 3 个 hot_loop 各跑 200 iter (各编 JIT) ----
    li   x19, 200
    call hot_loop_A_fn
    li   x19, 200
    call hot_loop_B_fn
    li   x19, 200
    call hot_loop_C_fn

    # ---- phase 2: sw 改 page B 的 hot_loop_B_fn inst (SMC trigger 1) ----
    la   x7, hot_loop_B_fn
    li   x8, 0x00a90913             # addi x18, x18, 10
    sw   x8, 0(x7)

    # ---- phase 3: sw 改 page C 的 hot_loop_C_fn inst (SMC trigger 2) ----
    la   x7, hot_loop_C_fn
    li   x8, 0x00a90913             # addi x18, x18, 10
    sw   x8, 0(x7)

    # ---- phase 4: 3 个 hot_loop 各跑 100 iter ----
    # A: 期望直接 hit page A JIT 块 (+1 × 100 = 100)
    # B: 期望走 interpret 跑改后 (+10 × 100 = 1000)
    # C: 期望走 interpret 跑改后 (+10 × 100 = 1000)
    li   x19, 100
    call hot_loop_A_fn
    li   x19, 100
    call hot_loop_B_fn
    li   x19, 100
    call hot_loop_C_fn

    j    halt

# ============================================================================
# hot_loop_A_fn — page A (0x80000xxx)
# ============================================================================
hot_loop_A_fn:
    addi x18, x18, 1
    addi x19, x19, -1
    bne  x19, x0, hot_loop_A_fn
    ret

# ============================================================================
# hot_loop_B_fn — page B (0x80001000; .balign 4096)
# ============================================================================
.balign 4096
hot_loop_B_fn:
    addi x18, x18, 1                # phase 2 改 → addi x18, x18, 10
    addi x19, x19, -1
    bne  x19, x0, hot_loop_B_fn
    ret

# ============================================================================
# hot_loop_C_fn — page C (0x80002000; .balign 4096)
# ============================================================================
.balign 4096
hot_loop_C_fn:
    addi x18, x18, 1                # phase 3 改 → addi x18, x18, 10
    addi x19, x19, -1
    bne  x19, x0, hot_loop_C_fn
    ret

halt:
    li   x30, 0x100000
    li   x31, 0x5555
    sw   x31, 0(x30)
halt_spin:
    j    halt_spin

# ===== 期望结果 =====
# phase 1: A×200 + B×200 + C×200 = 600
# phase 2: sw 改 page B (SMC trigger 1)
# phase 3: sw 改 page C (SMC trigger 2)
# phase 4: A×100 (+1, hit JIT) + B×100 (+10, interpret) + C×100 (+10, interpret) = 2100
# 总 x18 = 2700
#
# dump:
#   x18 = 2700           ← 关键: page A JIT 没被错清, B/C 走 interpret 跑改后
#                          (=900 → 全 page 都错跑成 +1; =1800 → 部分错)
#   x30/x31 = test_dev FINISHER PASS magic
#   exit 0
#
# trace 关键:
#   phase 4 hot_loop_A 100 iter: 全 'J' (page A JIT hit)
#   phase 4 hot_loop_B / hot_loop_C 100 iter 各段: 全 '._' (interpret 跑改后)
#
# 联动 verify:
#   - dispatcher 顶扫 page_dirty bitmap 看到多个 bit set (page B + page C 各 1 bit)
#   - smc_consume_dirty while loop 真 consume 多次
#   - 每次 invalidate 走 cpu_wait_all_harts_exit_host_code (单 hart 本 hart 状态灯 NULL)
#     + backend.invalidate_block release (per host_code; n_codes 可能 > 1 if page B/C
#     各有多个 entry — 但本 fixture 每 page 只 1 块, n_codes = 1)
