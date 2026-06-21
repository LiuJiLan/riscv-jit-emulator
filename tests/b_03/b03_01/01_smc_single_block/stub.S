# b_03_1 / 01_smc_single_block
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验 b_03 T1.a SMC chain 端到端最小闭环 (隐式 SIGSEGV 路径):
#   1. mprotect host page R-only (jit_cache_install 路径接, store COMPILED 之前)
#   2. 客户机 sw 写代码 page → 触发 SIGSEGV → handler 标 page_dirty + unprotect
#   3. dispatcher 顶扫 page_dirty bitmap → jit_invalidate_page (collect + invalidate
#      + 状态灯扫 + backend release)
#   4. 重 lookup miss → interpret 兜底 / 重 compile 跑改后代码
#
# 单 hart BARE regime (无 SV32; b_03 T1.b 加 SV32 fixture). 用 register dump 区分:
#   SMC 工作: x6 = 150 + 100 * 10 = 1150 (phase 3 跑改后 'addi x6, +10' 代码)
#   SMC 不工作 (重 compile 仍跑改前): x6 = 150 + 100 * 1 = 250
#   handler 没注册 / mprotect fail: 进程 SIGSEGV exit 139 (fixture 直接挂)
#
# ===== 思路 =====
# phase 1: 跑 hot_loop 150 iter (跨 THRESHOLD=100, 编 JIT 块)
# phase 2: sw 改 hot_loop 第一条 inst 的 immediate (1 → 10); SIGSEGV trigger SMC chain
# phase 3: 重跑 hot_loop 100 iter (期望 dispatcher 顶扫 invalidate + 跑改后代码)
#
# phase 区分: x12 (a2) flag 计数 — phase 1 后 +1, phase 3 后 +1; 用 beq 控制走向.
# halt 用 test_dev FINISHER PASS (0x100000 + 0x5555).
#
# 跟 b_02 fixture 体例:
#   - .option norvc (32-bit inst, 跟 -march=rv32im 一致)
#   - lui sp, 0x80300 (栈在 RAM 内空闲区)
#   - test_dev base 0x100000 / FINISHER PASS magic 0x5555
#   - x30/x31 用作 magic 寄存器 (跟 b_02 全部 fixture 一致)

.section .text
.option norvc
.globl _start
_start:
    # ---- 栈设置 (main.c 不用; halt 也不用; 保 ABI 一致体例) ----
    lui  sp, 0x80300                # sp = 0x80300000

    # ---- 初始化寄存器 ----
    li   x1, 150                    # loop counter (phase 1: 150 iter, 跨 THRESHOLD=100)
    li   x6, 0                      # x6 = accumulator
    li   x12, 0                     # x12 = phase counter (0/1/2)

    # ---- phase 1 entry: 跑 hot_loop 150 iter ----
    # 期望: x6 += 1 * 150 = 150; hot_loop 块跨 THRESHOLD → 编 JIT (b_02 T5 协议).
    # phase 1 末 bne fall-through 走 phase_dispatch.

hot_loop:
    # *** 第一条 inst (phase 2 改 immediate; 32-bit aligned PC = hot_loop) ***
    addi x6, x6, 1                  # 编码 0x00130313 (phase 2 改成 0x00a30313 = addi x6, x6, 10)
    addi x1, x1, -1
    bne  x1, x0, hot_loop           # 200/150 iter loop

phase_dispatch:
    # phase counter += 1, 决定走 phase 2 (改 inst) 还是 halt (phase 3 末)
    addi x12, x12, 1
    li   x13, 1
    beq  x12, x13, phase_2_modify   # x12 == 1: phase 1 末, 走 phase 2 改 inst
    # x12 == 2: phase 3 末, 走 halt
    j    halt

phase_2_modify:
    # ---- 改 hot_loop 第一条 inst ----
    # 原 inst: addi x6, x6, 1 = 0x00130313
    # 新 inst: addi x6, x6, 10 = 0x00a30313 (immediate 12 bit: 0xa = 10)
    la   x7, hot_loop               # x7 = hot_loop PA (BARE identity)
    li   x8, 0x00a30313             # 新 inst encoding
    sw   x8, 0(x7)                  # *** SMC trigger *** — host page mprotect R-only,
                                    # sw 触发 SIGSEGV → handler 标 page_dirty + unprotect →
                                    # 重执行 sw 成功改 inst.

    # ---- phase 3 setup + 跳回 hot_loop ----
    # phase 3: 跑 hot_loop 100 iter; 期望 dispatcher 顶扫 page_dirty 触发
    # jit_invalidate_page → entry 清 EMPTY → lookup miss → interpret 兜底跑
    # addi x6, x6, 10 (改后代码) → x6 += 10 * 100 = 1000.
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
# phase 2: sw 改 hot_loop 第一条 inst (SIGSEGV → SMC chain)
# phase 3: 期望 SMC 工作 → x6 += 10 * 100 = 1000; 总 x6 = 1150
#
# main.c dump (cpu_destroy 自动打):
#   x1     = 0            (loop 倒数到 0)
#   x6     = 1150         ← 关键: SMC 真生效跑改后代码 (addi +10)
#                          (若 = 250 → SMC 没生效, 仍跑改前 +1 → BUG)
#   x12    = 2            (phase counter; phase 1 + phase 3 各 +1)
#   x13    = 1            (phase 比较 reg)
#   x30    = 0x100000     (test_dev base)
#   x31    = 0x5555       (FINISHER PASS magic)
#   priv   = 3            (M; 单 hart BARE 全程 M-mode)
#   停机 = test_dev FINISHER PASS → exit 0
