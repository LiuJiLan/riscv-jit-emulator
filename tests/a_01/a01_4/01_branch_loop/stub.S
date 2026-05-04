# a_01_4 / 01_branch_loop
# 验证 BNE + dispatcher while(1) 多块循环 + boundary inst 自然产生块边界。
# 6 条 RV32I 指令, 24 字节 (32-bit only, .option norvc 强制不生 RVC)。
#
# 期望执行后:
#   x1 = 0          (loop 倒计数器到 0 退出)
#   x2 = 5          ← 主测试目标 (累加 5 次, x2 += 1 在 loop 体内每次 +1)
#   pc = 0x80000014 ← ecall 地址 (OP_UNSUPPORTED 时 pc 停在触发指令本身)
#
# 期望 dispatcher 行为:
#   total_count = 5 + 3*4 + 0 = 17
#     iter 1 (block from 0x80000000): addi x1,x0,5 / addi x2,x0,0 /
#                                     addi x2,x2,1 / addi x1,x1,-1 / bne taken (5 条),
#                                     boundary 命中后退出 fetch loop, pc = 0x80000008
#     iter 2-4 (block from 0x80000008 = loop): addi x2,x2,1 / addi x1,x1,-1 /
#                                              bne taken (3 条/块, 4 次累加: x2: 1→2→3→4,
#                                              第 4 次 (i.e. iter 4) 后 x1=2; 第 5 次 iter 5
#                                              进入时 x1=2)
#       Actually 仔细数:
#         iter 1 末: x1=4, x2=1, bne 4!=0 taken
#         iter 2 末: x1=3, x2=2, bne taken
#         iter 3 末: x1=2, x2=3, bne taken
#         iter 4 末: x1=1, x2=4, bne taken
#         iter 5 末: x1=0, x2=5, bne 0!=0 false, 不 taken, fall-through pc=0x80000014
#         iter 6: ecall, OP_UNSUPPORTED, count=0 → dispatcher break
#       所以 total_count = 5 + 3 + 3 + 3 + 3 + 0 = 17
#
# 注: bne x1, x0, loop 这条 boundary 是 hard limit 的"自然终结者", 没有 boundary 检查
#     dispatcher 就会一直在 iter 1 那块内死循环 (bne 把 pc 写回 loop, fetch loop 继续从
#     loop 取指, 直到 BLOCK_INST_LIMIT=64 救场)。boundary + while(1) 是这次设计的核心。

.section .text
.option norvc                       # 强制不生 RVC; 与 a01_3 fixture 同, 解释器算术路径
                                    # a_01_4 还没接 RVC branch (C.BEQZ/C.BNEZ), 都走 32-bit
.globl _start
_start:
    addi x1, x0, 5      # 0x80000000: x1 = 5      (loop counter)            (0x00500093)
    addi x2, x0, 0      # 0x80000004: x2 = 0      (accumulator)             (0x00000113)
loop:
    addi x2, x2, 1      # 0x80000008: x2 += 1                               (0x00110113)
    addi x1, x1, -1     # 0x8000000c: x1 -= 1                               (0xFFF08093)
    bne  x1, x0, loop   # 0x80000010: if x1 != 0 goto loop (offset = -8)    (0xFE009CE3)
    ecall               # 0x80000014: OP_UNSUPPORTED, dispatcher halt       (0x00000073)
