# b_03_2 / 03_fencei_no_modify
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验 b_03 T2 Case C — fence.i 调用但没改 page (块边界 verify; SMC chain 没触发).
# 关键 invariant: page_dirty bitmap 空时, dispatcher 顶扫空转, JIT 块 entry 仍
# COMPILED → lookup 继续 hit 旧块. fence.i 在我们项目仅当块边界 + lrsc_clear_self,
# 不主动 invalidate.
#
# 跟 fixture 01 区别: 01 phase 2 有 sw 改 hot_loop inst (SMC trigger); 03 phase 2
# 只调 fence.i 没改 page (无 SIGSEGV trigger, page_dirty bitmap 空).
#
# 路径 walk through:
#   1. phase 1 跑 hot_loop 200 iter → 编 JIT (跨 THRESHOLD=100)
#   2. phase 2 显式 fence.i (块边界切块 + lrsc_clear_self 副作用) — 但没 sw 改 page
#   3. fence.i 后退 JIT 块 → dispatcher 顶扫 bitmap (**空**) → 不调
#      jit_invalidate_page → JIT 块 entry 仍 COMPILED
#   4. phase 3 跑 hot_loop 100 iter → lookup 仍 hit 旧 JIT 块 → 继续跑改前代码
#      (addi +1)
#
# 期望 x6 = 1 * 200 + 1 * 100 = 300 (JIT 块继续 hit, 没被 fence.i 误清).
#
# 反例 (fixture 失败 case):
#   x6 != 300: fence.i 误触发 invalidate (BUG: fence_i_helper 实装错误调
#              jit_cache_invalidate_page) — 这是 T2 audit Q4.2.1+ 禁的行为
#
# 单 hart BARE regime. x12 phase counter (跟 fixture 01 体例).

.section .text
.option norvc
.globl _start
_start:
    # ---- 栈设置 ----
    lui  sp, 0x80300                # sp = 0x80300000

    # ---- 初始化寄存器 ----
    li   x1, 200                    # loop counter (phase 1: 200 iter)
    li   x6, 0                      # x6 = accumulator
    li   x12, 0                     # x12 = phase counter (0/1/2)

hot_loop:
    addi x6, x6, 1                  # phase 3 仍跑此 (没改)
    addi x1, x1, -1
    bne  x1, x0, hot_loop

phase_dispatch:
    addi x12, x12, 1
    li   x13, 1
    beq  x12, x13, phase_2_fencei   # x12 == 1: phase 1 末 → 走 phase 2
    j    halt                       # x12 == 2: phase 3 末 → halt

phase_2_fencei:
    # ---- *** 显式 fence.i (无 sw 改 page) *** ----
    # T2 关键 verify 点: 没有 SMC trigger (page_dirty bitmap 空), fence.i 块边界 +
    # lrsc_clear_self 副作用; dispatcher 顶扫 bitmap 不 invalidate, JIT 块继续 hit.
    fence.i

    # ---- phase 3 setup + 跳回 hot_loop ----
    li   x1, 100
    j    hot_loop

halt:
    # ---- 干净停机: test_dev FINISHER PASS → exit 0 ----
    li   x30, 0x100000              # test_dev base
    li   x31, 0x5555                # FINISHER PASS magic
    sw   x31, 0(x30)
halt_spin:
    j    halt_spin

# ===== 期望结果 =====
# phase 1: x6 += 1 * 200 = 200 (JIT 编后跑)
# phase 2: fence.i (块边界, 无 invalidate; bitmap 空)
# phase 3: x6 += 1 * 100 = 100 (JIT 块仍 hit, 跑改前代码)
# 总 x6 = 300
#
# main.c dump:
#   x1     = 0            (loop 倒数到 0)
#   x6     = 300          ← 关键: fence.i 没误触发 invalidate
#                          (若 ≠ 300 → fence.i 路径 BUG / dispatcher 顶扫误清)
#   x12    = 2            (phase counter)
#   x13    = 1            (phase 比较 reg)
#   x30    = 0x100000
#   x31    = 0x5555
#   priv   = 3            (M; 单 hart BARE 全程 M-mode)
#   停机 = test_dev FINISHER PASS → exit 0
