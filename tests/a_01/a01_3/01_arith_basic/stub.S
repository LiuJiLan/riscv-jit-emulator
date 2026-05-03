# a01_3 / 01_arith_basic
# 4 条 RV32I 指令, 16 字节 (与 a01_1 / 01_bin_basic 同尺寸)。
# 末尾 ecall 触发 OP_UNSUPPORTED, 让 interpreter break 出 fetch loop。
#
# 期望执行后:
#   x1 = 42
#   x2 = 8
#   x3 = 50          ← 主测试目标
#   pc = 0x8000000c  ← ecall 地址 (OP_UNSUPPORTED 时 pc 停在触发指令本身, RV trap 语义对齐)
#   interpret_one_block 的 count_out = 3 (不含 ecall)

.section .text
# Makefile 用 -march=rv32imac (与 a01_1 一致, 包含 c = compressed 扩展)。
# 但 a01_3 解释器只支持 RV32I 32-bit 编码, 不识别 16-bit C.* 形式。
# gcc 汇编器看到 imm 适合时会自动把 addi x2, x0, 8 之类编为 C.LI x2, 8 (16-bit, 0x4121),
# 导致 4 字节 fetch 拿到混 (compressed_inst | next_inst_low16) 的奇怪 word, decode 归
# OP_UNSUPPORTED 提前 break。
# .option norvc 强制汇编器不生成 c 扩展指令, 所有 mnemonic 走 32-bit 编码。
# 未来真上 C 扩展支持时, 删这一行 + decode 加 16-bit 路径即可。
.option norvc
.globl _start
_start:
    addi x1, x0, 42      # 0x80000000: OP_ADDI, x1 = 0 + 42 = 42       (0x02A00093)
    addi x2, x0, 8       # 0x80000004: OP_ADDI, x2 = 0 + 8  = 8        (0x00800113)
    add  x3, x1, x2      # 0x80000008: OP_ADD,  x3 = x1 + x2 = 50      (0x002081B3)
    ecall                # 0x8000000c: OP_UNSUPPORTED (a01_3), break   (0x00000073)
