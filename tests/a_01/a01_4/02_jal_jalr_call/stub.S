# a_01_4 / 02_jal_jalr_call
# 验证 jal (call) + jalr (return) 经典 RV 函数调用模式。
# 6 条 RV32I 指令, 24 字节 (32-bit only, .option norvc 强制不生 RVC)。
#
# 同时验证:
#   - jal 正向 imm + 写 ra (rd ≠ x0): x1 ← pc+4 是返回地址
#   - jalr return-style (rs1=x1, imm=0, rd=x0): pc ← (ra+0) & ~1u, ra 不被覆盖
#   - dispatcher 处理非单调 pc (向前 jal 跳到 sub, 向后 jalr 跳回 main)
#
# 寄存器选 x1/x2/x3 (不用 a0/a1 = x10/x11 调用约定名), 与 main.c 现有 fprintf 输出对齐。
# 实际工程用 a0/a1 传参更标准, 但本 fixture 仅验证指令语义, 寄存器编号选 main.c 能直接打的。
#
# 期望执行后:
#   x1 = 0x8000000c       ← jal 写的返回地址 (= jal 自身 pc + 4 = 0x80000008 + 4)
#   x2 = 12               ← 主测试目标 (= 7 + 5, sub 内 add 算)
#   x3 = 5                ← 未动 (sub 内只读)
#   pc = 0x8000000c       ← ecall 地址 (jalr 跳回这里, ecall = OP_UNSUPPORTED)
#
# 期望 dispatcher 行为:
#   total_count = 5 = 3 + 2 + 0
#     iter 1 (block 0x80000000): addi x2 / addi x3 / jal (boundary), 3 条;
#                                 jal 后 hart->regs[0] = sub = 0x80000010
#     iter 2 (block 0x80000010 = sub): add / jalr (boundary), 2 条;
#                                       jalr 后 hart->regs[0] = ra (= 0x8000000c)
#     iter 3 (block 0x8000000c = ecall 位置): ecall = OP_UNSUPPORTED 立即 goto out, 0 条
#                                              dispatcher 看 count=0 break

.section .text
.option norvc
.globl _start
_start:
    addi x2, x0, 7      # 0x80000000: x2 = 7                                 (0x00700113)
    addi x3, x0, 5      # 0x80000004: x3 = 5                                 (0x00500193)
    jal  x1, sub        # 0x80000008: x1 = 0x8000000c, pc = sub = 0x80000010
                        #             imm = sub - here = +8; gcc 算 label 偏移
                        #             encoding = 0x008000EF (J-type, rd=1, imm=+8)
    # ----- jalr 返回到这里 -----
    ecall               # 0x8000000c: OP_UNSUPPORTED, dispatcher halt        (0x00000073)

sub:
    add  x2, x2, x3     # 0x80000010: x2 = x2 + x3 = 12                       (0x00310133)
    jalr x0, x1, 0      # 0x80000014: pc = (x1 + 0) & ~1 = 0x8000000c
                        #             rd = x0 → 不写 (走 x0_garbage),
                        #             ra (x1) 不被覆盖, 返回地址保持
                        #             encoding = 0x00008067 (I-type, rd=0, rs1=1, imm=0)
