# b_03_2 / 01_fencei_after_smc
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验 b_03 T2 fence.i + SMC chain 串联 (Case A: 客户机改完显式 fence.i, RV spec
# conformance 路径). 关键 invariant: fence.i 不主动调 jit_cache_invalidate_page,
# 但 fence.i 块边界 + SMC SIGSEGV bitmap + dispatcher 顶扫整链协作让结果跟 T1.a
# 01_smc_single_block (不调 fence.i) 完全一致 (x6 = 1150).
#
# fence.i 路径 walk through:
#   1. phase 1 跑 hot_loop 150 iter → 编 JIT (跨 THRESHOLD=100)
#   2. phase 2 sw 改 hot_loop 第一条 inst (1 → 10) → SIGSEGV → handler 标 page_dirty
#      + unprotect → 重执行 sw 改成
#   3. phase 2 显式 fence.i → 块边界切块 (decode.h:574 → 1) + lrsc_clear_self
#      → fence_i_helper 函数体不调 jit_cache_invalidate_page (b_03 audit Q4.2.1+)
#   4. 块边界后 → 退 JIT 块 → dispatcher 顶扫 bitmap → jit_invalidate_page
#   5. phase 3 跑 hot_loop 100 iter → 重 lookup miss → interpret 兜底 / 重 compile
#      → 跑改后代码 (addi +10)
#
# 跟 T1.a 01 行为对偶 verify:
#   SMC chain 工作 + fence.i 不破: x6 = 150 + 100 * 10 = 1150 (跟 T1.a 01 一致)
#   fence.i 误触发 invalidate / 多次清: x6 仍 1150 (idempotent verify)
#   fence_i_helper 真主动调 invalidate (旧拍法): 行为不变, fixture 测不出区别 —
#     真正 verify "不主动调" 是 dispatcher 顶扫 trace + jit_cache_invalidate_page
#     调用次数, fixture 层只能 verify "整链结果正确"
#
# 单 hart BARE regime (跟 T1.a 01 体例). x12 phase counter (跟 T1.a 01 同体例).

.section .text
.option norvc
.globl _start
_start:
    # ---- 栈设置 ----
    lui  sp, 0x80300                # sp = 0x80300000

    # ---- 初始化寄存器 ----
    li   x1, 150                    # loop counter (phase 1: 150 iter)
    li   x6, 0                      # x6 = accumulator
    li   x12, 0                     # x12 = phase counter (0/1/2)

hot_loop:
    # *** 第一条 inst (phase 2 改 immediate; PA = hot_loop) ***
    addi x6, x6, 1                  # 编码 0x00130313 (phase 2 改成 0x00a30313)
    addi x1, x1, -1
    bne  x1, x0, hot_loop

phase_dispatch:
    addi x12, x12, 1
    li   x13, 1
    beq  x12, x13, phase_2_modify   # x12 == 1: phase 1 末 → 走 phase 2
    j    halt                       # x12 == 2: phase 3 末 → halt

phase_2_modify:
    # ---- 改 hot_loop 第一条 inst ----
    la   x7, hot_loop               # x7 = hot_loop PA (BARE identity)
    li   x8, 0x00a30313             # 新 inst: addi x6, x6, 10
    sw   x8, 0(x7)                  # *** SMC trigger *** (SIGSEGV → handler 标 dirty)

    # ---- *** 显式 fence.i (RV spec conformance) *** ----
    # T2 关键 verify 点: fence.i 在我们 = 块边界 + lrsc_clear_self, 不主动 invalidate.
    # 块边界保证 dispatcher 顶扫 bitmap 在 spec 时机触发 jit_invalidate_page.
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
# phase 1: x6 += 1 * 150 = 150
# phase 2: sw 改 hot_loop + 显式 fence.i (块边界切块)
# phase 3: x6 += 10 * 100 = 1000
# 总 x6 = 1150
#
# main.c dump:
#   x1     = 0            (loop 倒数到 0)
#   x6     = 1150         ← 关键: SMC + fence.i 整链正确 (跟 T1.a 01 一致)
#                          (若 = 250 → SMC chain 没生效 BUG)
#                          (若 ≠ 1150 → fence.i 误触发 invalidate / 多次清 BUG)
#   x12    = 2            (phase counter)
#   x13    = 1            (phase 比较 reg)
#   x30    = 0x100000
#   x31    = 0x5555
#   priv   = 3            (M; 单 hart BARE 全程 M-mode)
#   停机 = test_dev FINISHER PASS → exit 0
