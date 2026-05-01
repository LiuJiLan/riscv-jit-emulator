# 01_bin_basic stub
# 16 字节签名, 加载后 hex dump 一眼可辨。
# 内容是纯数据(.word), 不需要是合法 RV32 指令 —— loader 不执行。

.section .text
.globl _start
_start:
    .word 0xDEADBEEF
    .word 0xDEADBEEF
    .word 0x12345678
    .word 0x12345678
