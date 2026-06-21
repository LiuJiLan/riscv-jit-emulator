# b_03_1 / 03_smc_non_jit_page
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验 b_03 T1.a SMC chain **非 JIT page store 不破既有 JIT 块**:
#   - 跟 01/02 区别: 01/02 改自己代码 (JIT page) 验证 SMC 真触发 + invalidate;
#     03 改 data page (没装过 JIT 块, mprotect 没设) 验证 store 不破 JIT
#   - 期望: dispatcher 顶扫 page_dirty bitmap 始终空 (handler 没触发, 因为该
#     page 没 mprotect 限制 R-only), 既有 JIT 块全程 hit (phase 2 跑 + 1, 不
#     退化重 interpret/compile)
#
# 同时 verify dispatcher 顶扫 fast-path (page_dirty 全 0 时 linear scan 512 word
# 全 0 直接 return false, while 不进, hot path 无额外 cycles).
#
# 设计:
#   phase 1: 跑 hot_loop 150 iter 编 JIT (跨 THRESHOLD)
#   phase 2: sw 写 data page 0x80200000 (data 区, 没 JIT 块, 没 mprotect)
#   phase 3: 跑 hot_loop 200 iter, 期望直接 hit phase 1 编的 JIT 块, x6 += 200 * 1 = 200
#   总 x6 = 150 + 200 = 350

.section .text
.option norvc
.globl _start
_start:
    lui  sp, 0x80300

    li   x1, 150
    li   x6, 0
    li   x12, 0                     # phase counter

hot_loop:
    addi x6, x6, 1                  # 第一条 inst (本 fixture 不改; 验证 JIT 块全程 hit)
    addi x1, x1, -1
    bne  x1, x0, hot_loop

phase_dispatch:
    addi x12, x12, 1
    li   x13, 1
    beq  x12, x13, phase2_setup
    j    halt

phase2_setup:
    # ---- 改 data page (非 JIT page); 期望不触发 SMC chain ----
    # 0x80200000 = leaf data page (非代码段; mmap 全程 R/W, 没被 jit_cache_install
    # 路径调过 smc_protect_page → host kernel 没设 R-only → store 不触发 SIGSEGV
    # → page_dirty bitmap 不标 → dispatcher 顶扫 invalidate 不调).
    li   x7, 0x80200000             # data page base (远离代码段 0x80000000)
    li   x8, 0xdeadbeef             # 任意 magic value
    sw   x8, 0(x7)                  # 普通 sw, 不触发 SMC

    # phase 3: 跑 hot_loop 200 iter, 期望直接 hit phase 1 编的 JIT 块全程
    li   x1, 200
    j    hot_loop

halt:
    # ---- 干净停机 ----
    li   x30, 0x100000
    li   x31, 0x5555
    sw   x31, 0(x30)
halt_spin:
    j    halt_spin

# ===== 期望结果 =====
# phase 1: 150 × 1 = 150
# phase 2: sw 0xdeadbeef 到 0x80200000 (非 JIT page, 不触发 SMC)
# phase 3: 200 × 1 = 200 (JIT 块全程 hit, 不退化)
# 总 x6 = 350
#
# dump:
#   x1     = 0
#   x6     = 350         ← 关键: JIT 块没被错误 invalidate
#                          (若 = 350 但 trace 中 phase 3 出现新的 .__.__... interpret
#                           段, 说明 invalidate 错触发, 算 partial fail)
#   x7     = 0x80200000  (data page address)
#   x8     = 0xdeadbeef  (写入 value)
#   x12    = 1           (phase counter; 只 phase 1 + 1)
#   x13    = 1
#   x30/x31 = test_dev FINISHER PASS magic
#   exit 0
#
# trace 期望:
#   phase 1: 100 个 `._` (interpret) + 50 个 `._J` (JIT)
#   phase 3: 200 个 `._J` (全程 JIT hit; 无 `._` interpret) ← 验 JIT 块未被错误 invalidate
#   若 phase 3 出现 `._` interpret 段, 表示 invalidate 路径错触发 (bug)
