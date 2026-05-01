# 04_elf_class64_reject stub
# 内容和 02_elf_basic 相同, 唯一区别在 Makefile 用 -march=rv64imac -mabi=lp64,
# 产出 ELFCLASS64 文件, 触发 loader 的 class 校验拒绝。

.section .text
.globl _start
_start:
    .word 0xDEADBEEF
    .word 0xDEADBEEF
    .word 0x12345678
    .word 0x12345678
