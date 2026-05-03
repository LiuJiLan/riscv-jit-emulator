# a01_3 / 02_arith_compressed
# 3 条 16-bit RVC 算术 + 1 条 32-bit ecall, 总 10 字节。
# 验证: decode 走 RVC 路径 (decode_rvc) + fetch loop +=2 推进 + interpreter
#       真执行 RVC 算术 (复用 OP_ADDI 通用 case, RVC 是长度变化非语义变化)。
#
# 用 c.* 显式 mnemonic 强制 RVC 编码, 不依赖 -march 默认行为 (与 01_arith_basic 的
# .option norvc 互补 — 01 测 32-bit 路径, 02 测 RVC 路径)。
#
# 期望执行后:
#   x1 = 11           (c.li x1, 8 写入 8; 然后 c.addi x1, 3 加 3 → 11)
#   x2 = 5            (c.li x2, 5)
#   x3 = 0            (没 set, memset 默认; main fprintf 仍打印验证)
#   pc = 0x80000006   (ecall 地址 = 起点 + 6 字节, 证明 3 次 +2 RVC 步进)
#   dispatcher count = 3 (3 条 RVC 真执行, ecall OP_UNSUPPORTED 不算)

.section .text
.globl _start
_start:
    c.li   x1, 8         # 0x80000000: C.LI x1, 8       (16-bit, 0x40A1)
    c.li   x2, 5         # 0x80000002: C.LI x2, 5       (16-bit, 0x4115)
    c.addi x1, 3         # 0x80000004: C.ADDI x1, 3     (16-bit, 0x008D)
    ecall                # 0x80000006: OP_UNSUPPORTED   (32-bit, 0x00000073)
