# 02_elf_basic stub
# 与 01 内容相同, 16 字节签名。区别只在打包: 01 出 .bin, 02 出 .elf。

.section .text
.globl _start
_start:
    .word 0xDEADBEEF
    .word 0xDEADBEEF
    .word 0x12345678
    .word 0x12345678
