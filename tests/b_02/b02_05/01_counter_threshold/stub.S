# b_02_5 / 01_counter_threshold
#
# ===== 测试目的 =====
# 验证 b_02 T5 heat counter 真消费 — dispatcher fork 点 lookup_or_init 拿
# COUNTING entry 时 atomic_fetch_add counter, 达 COMPILE_THRESHOLD (=100, config.h)
# 触发 jit_compile_block, COMPILED 后下次 lookup_or_init hit COMPILED 走 host_code.
#
# ===== 思路 =====
# 跑 200 iter loop, 同一 block (loop:) 反复 fetch + 自增 counter.
#   - iter 1~99: dispatcher fork 点 lookup_or_init 拿 COUNTING entry, fetch_add
#                counter (1..99), 未达 100 → 走 interpret (trace '._' 每 iter)
#   - iter 100:  fetch_add counter 100, 达 100 → needs_compile=1 → 出 RCU 后调
#                jit_compile_block (BARE BRANCH 块 backend 真编 host_code), entry
#                promote COMPILED; 本次仍走 interpret
#   - iter 101+: lookup_or_init 拿 COMPILED entry → host_code 跑 → trace 加 'J'
#                每 iter (形态 '._J._J...')
#
# trace 前段 '._' 重复 ~100 次, 后段 '._J' 重复 ~100 次. 真验证 counter 阈值
# 路径走通跟 jit_compile_block + install promote 成功.
#
# 期望块结构 (跟 a_01_4/01_branch_loop 同体例):
#   block 1 from 0x80000000: addi x1,x0,200 / addi x2,x0,0 / addi x2,x2,1 /
#                            addi x1,x1,-1 / bne taken (5 inst); 入 loop 块
#   block 2..N from 0x80000008: addi x2,x2,1 / addi x1,x1,-1 / bne taken
#                                (3 inst/block, 199 次 bne taken + 第 200 次 fall-through)
#   block N+1 from 0x80000014: ecall (1 inst, OP_UNSUPPORTED → halt)
#
# total_count = 5 (block 1) + 199 × 3 (block 2 BNE taken iter) + 3 (block 2 BNE
#               fall-through iter 200, count 3 inst 含 bne) + 1 (ecall) = 5 + 597 + 3 + 1 = 606
#
# loop 块 (PA = 0x80000008) 跑 200 次, 第 100 次触发 jit_compile_block,
# 第 101..200 次 (100 次) 走 JIT 'J'. trace 总 'J' 数 = ~100.
#
# 跟 a_01_4/01_branch_loop 差异:
#   - 计数器从 5 改 200 (跨 COMPILE_THRESHOLD=100; T5 验收路径走通)
#   - 块体 + bne 路径完全同 (BARE BRANCH 块走 T4 backend emit)

.section .text
.option norvc                       # 强制不生 RVC; loop 块体 3 条 32-bit
.globl _start
_start:
    addi    x1, x0, 200     # 0x80000000: x1 = 200    (loop counter, 跨 THRESHOLD=100)
    addi    x2, x0, 0       # 0x80000004: x2 = 0      (accumulator)
loop:
    addi    x2, x2, 1       # 0x80000008: x2 += 1
    addi    x1, x1, -1      # 0x8000000c: x1 -= 1
    bne     x1, x0, loop    # 0x80000010: if x1 != 0 goto loop (offset = -8)
    ecall                   # 0x80000014: OP_UNSUPPORTED, dispatcher halt

# ===== 期望结果 =====
# 期望执行后:
#   x1 = 0          (loop 倒计数器到 0 退出)
#   x2 = 200        (累加 200 次)
#   pc = 0x80000014 (ecall 触发地址)
#   total_count = 606
#
# trace 期望形态 (--smp 1, default):
#   前段 ~100 iter: '._._._...' 重复 (每 iter check '.' + fetch '_', counter 未达 100)
#   后段 ~100 iter: '._J._J._J...' 重复 (每 iter check '.' + fetch '_' + JIT hit 'J')
#
# 'J' 字符首次出现位置 ≈ 第 100 iter 后 (trace 中间区域). 数 'J' 字符总数 ≈ 100.
#
# 如果 trace 没 'J' (counter 路径未接通) 或 'J' 从 iter 1 开始 (counter 没真消费,
# miss 即编) 则 T5 失败.