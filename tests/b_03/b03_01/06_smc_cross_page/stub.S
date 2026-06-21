# b_03_1 / 06_smc_cross_page
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验 b_03 T1.a SMC chain **page 颗粒度精确性** (6 axes (iv)=跨 page):
#   - hot_loop_A_fn 在 page A (0x80000xxx), 编 JIT 块
#   - hot_loop_B_fn 在 page B (0x80001xxx; .balign 4096 强制 page-align), 编 JIT 块
#   - sw 改 page B 的 hot_loop_B_fn inst → SIGSEGV 标 page_dirty[1] (page B only)
#   - dispatcher 顶扫 → jit_invalidate_page(1) 只清 page B entry; page A 不动
#   - phase 4 重跑 hot_loop_A_fn 应直接 hit page A JIT 块 (verify 没被错误 invalidate)
#   - phase 5 重跑 hot_loop_B_fn 走 interpret 跑改后 inst (+10)
#
# 期望 x18 (s2 accumulator) = 150 + 150 + 100 (A hit JIT, +1) + 100×10 (B 改后 +10) = 1400
#
# ===== 思路 =====
# 用函数 call/ret 体例: hot_loop_A/B_fn 都是 function (ra=x1 给 call/ret 用,
# accumulator 用 x18=s2, loop counter 用 x19=s3, 避撞 t/ra).
#
# .balign 4096 强制 hot_loop_B_fn 在 page B 边界 (0x80001000). linker .text 段
# emit padding bytes 把 hot_loop_B_fn label 对齐到 4KB.

.section .text
.option norvc
.globl _start
_start:
    lui  sp, 0x80300                # 栈

    li   x18, 0                     # accumulator (全局; saved reg 避撞 t/call)
    li   x14, 0                     # phase counter (用 x14 a4, 跟 fixture 04 同体例)

    # ---- phase 1: 跑 hot_loop_A 150 iter (跨 THRESHOLD 编 JIT 在 page A) ----
    li   x19, 150
    call hot_loop_A_fn              # x1 = ra (call site 返回地址)

    # ---- phase 2: 跑 hot_loop_B 150 iter (跨 THRESHOLD 编 JIT 在 page B) ----
    li   x19, 150
    call hot_loop_B_fn

    # ---- phase 3: sw 改 page B (hot_loop_B_fn 第一条 inst) ----
    la   x7, hot_loop_B_fn
    li   x8, 0x00a90913             # addi x18, x18, 10 (原 0x00190913 = addi x18, +1)
    sw   x8, 0(x7)                  # *** SMC trigger (page B only) ***

    # ---- phase 4: 再跑 hot_loop_A 100 iter (期望直接 hit page A JIT) ----
    li   x19, 100
    call hot_loop_A_fn

    # ---- phase 5: 再跑 hot_loop_B 100 iter (期望走 interpret 跑改后 +10) ----
    li   x19, 100
    call hot_loop_B_fn

    j    halt

# ============================================================================
# hot_loop_A_fn — page A (0x80000xxx)
# ============================================================================
hot_loop_A_fn:
    addi x18, x18, 1                # 0x00190913
    addi x19, x19, -1
    bne  x19, x0, hot_loop_A_fn
    ret                             # jalr x0, x1, 0

# ============================================================================
# .balign 4096 强制 hot_loop_B_fn 到 page B (0x80001000)
# ============================================================================
.balign 4096

# ============================================================================
# hot_loop_B_fn — page B (0x80001xxx)
# ============================================================================
hot_loop_B_fn:
    addi x18, x18, 1                # 0x00190913 (phase 3 改 → 0x00a90913 = +10)
    addi x19, x19, -1
    bne  x19, x0, hot_loop_B_fn
    ret

halt:
    li   x30, 0x100000              # test_dev base
    li   x31, 0x5555                # FINISHER PASS magic
    sw   x31, 0(x30)
halt_spin:
    j    halt_spin

# ===== 期望结果 =====
# phase 1: hot_loop_A × 150 (+1) = 150
# phase 2: hot_loop_B × 150 (+1) = 150;  total = 300
# phase 3: sw 改 page B (SMC; 只 invalidate page B)
# phase 4: hot_loop_A × 100 (+1; 期望 page A JIT 仍 hit) = 100;  total = 400
# phase 5: hot_loop_B × 100 (+10; 走 interpret 改后代码) = 1000;  total = 1400
#
# 总 x18 = 1400
#
# dump:
#   x18    = 1400        ← 关键: page B SMC 不影响 page A JIT 块
#                          若 = 500 (= 150+150+100+100; B 也跑成 +1) → SMC 失败
#                          若 = 1400 但 trace 中 phase 4 出现 ._ interpret (无 J) →
#                          page A JIT 被错误 invalidate (page-级 invalidate 不精确)
#   x7     = hot_loop_B_fn vaddr  ≈ 0x80001000
#   x8     = 0x00A90913           (改后 inst encoding)
#   x30/x31 = test_dev FINISHER PASS magic
#   exit 0
#
# 关键 verify (trace):
#   phase 4 (hot_loop_A 100 iter): trace 应全 'J' (直接 hit page A JIT 块)
#   phase 5 (hot_loop_B 100 iter): trace 应全 '._' (interpret 跑改后)
