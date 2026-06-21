# b_03_1 / 02_smc_loop_modify
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验 b_03 T1.a SMC chain **多轮 invalidate + 重 compile + JIT hit 改后块** 路径:
#   - 跟 01_smc_single_block 区别: 01 只跑 1 轮 SMC trigger + phase 3 全 interpret
#     (100 iter 不达 THRESHOLD 重 compile); 02 跑 3 轮 SMC + 每 phase 200 iter
#     (100 累计达 THRESHOLD trigger compile + 100 hit 新 JIT 块跑改后代码)
#   - 验 dispatcher 顶扫 N 轮均工作无 stale; 重 compile 路径每 phase 真激活
#
# 用 distinct imm (1, 10, 100) 区分各 phase 跑的代码版本:
#   phase 1: hot_loop 第一条 'addi x6, x6, 1' × 150 iter, x6 += 150
#   phase 2: 改 imm 1 → 10, 跑 200 iter, x6 += 200 * 10 = 2000
#   phase 3: 改 imm 10 → 100, 跑 200 iter, x6 += 200 * 100 = 20000
#   总 x6 = 22150 (distinct value, 不可能被其他错误路径 hit 出)
#
# 单 hart BARE regime; 跟 fixture 01 同体例 (.S only, 无 main.c).

.section .text
.option norvc
.globl _start
_start:
    lui  sp, 0x80300

    li   x1, 150
    li   x6, 0
    li   x12, 0                     # phase counter

hot_loop:
    # *** 第一条 inst (phase 2/3 改 imm; 32-bit aligned PC = hot_loop) ***
    addi x6, x6, 1                  # 编码 0x00130313
    addi x1, x1, -1
    bne  x1, x0, hot_loop

phase_dispatch:
    # phase counter += 1, 决定走 phase 2 / phase 3 / halt
    addi x12, x12, 1
    li   x13, 1
    beq  x12, x13, phase2_setup     # x12 == 1: phase 1 末
    li   x13, 2
    beq  x12, x13, phase3_setup     # x12 == 2: phase 2 末
    j    halt                       # x12 == 3: phase 3 末

phase2_setup:
    # 改 hot_loop 第一条 inst: addi x6, x6, 1 → addi x6, x6, 10 (imm = 10 = 0xa)
    la   x7, hot_loop
    li   x8, 0x00a30313             # addi x6, x6, 10
    sw   x8, 0(x7)                  # SMC trigger (round 1)
    li   x1, 200                    # phase 2: 200 iter (100 trigger compile + 100 hit JIT 改后)
    j    hot_loop

phase3_setup:
    # 改 hot_loop 第一条 inst: addi x6, x6, 10 → addi x6, x6, 100 (imm = 100 = 0x064)
    la   x7, hot_loop
    li   x8, 0x06430313             # addi x6, x6, 100
    sw   x8, 0(x7)                  # SMC trigger (round 2)
    li   x1, 200                    # phase 3: 200 iter
    j    hot_loop

halt:
    li   x30, 0x100000
    li   x31, 0x5555
    sw   x31, 0(x30)
halt_spin:
    j    halt_spin

# ===== 期望结果 =====
# phase 1: 150 × 1 = 150         (跑改前 imm=1; 100 interpret + 50 JIT)
# phase 2: 200 × 10 = 2000       (改后 imm=10; SMC invalidate phase 1 JIT 块,
#                                  100 interpret + 100 hit 重 compile 的新 JIT 块)
# phase 3: 200 × 100 = 20000     (改后 imm=100; SMC invalidate phase 2 JIT 块,
#                                  100 interpret + 100 hit 又一次重 compile)
# 总 x6  = 22150                 ← 关键 verify; distinct value 区分所有 phase 真跑
#
# main.c dump:
#   x1     = 0            (phase 3 loop 倒数到 0)
#   x6     = 22150        ← 总 accumulator
#   x12    = 3            (phase counter; 3 phase 各 +1)
#   x13    = 2            (最后 li 的值)
#   x30/x31 = test_dev FINISHER PASS magic
#   priv   = 3            (M; 单 hart BARE)
#   exit 0
#
# 如果 SMC 任何一轮失败:
#   - 仅 phase 1 SMC 失败: x6 = 150 + 200×1 + 200×1 = 550 (phase 2/3 都跑 +1)
#   - 仅 phase 2 SMC 失败: x6 = 150 + 200×10 + 200×10 = 4150 (phase 3 重 compile 失败仍 +10)
#   - 全 SMC 失败 (handler 没注册): SIGSEGV exit 139
#   - 跑出 22150 = 多轮 SMC + 重 compile + JIT hit 改后都成功
