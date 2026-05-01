# 03_elf_multi_seg stub
# 两段: .text(0xDEADBEEF × 2)+ .data(0xCAFEBABE × 2)。
# 链接脚本 + PHDRS 把它们拉到不同 1MB 段, 强制产出两个 PT_LOAD 头。

.section .text
.globl _start
_start:
    .word 0xDEADBEEF
    .word 0xDEADBEEF

.section .data
    .word 0xCAFEBABE
    .word 0xCAFEBABE
