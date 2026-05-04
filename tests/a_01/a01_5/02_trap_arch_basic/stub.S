# a_01_5_b / 02_trap_arch_basic
# 验证 trap_set_state 架构语义层 + 候选 A 早 return + dispatcher while(in_trap<3) 退出。
# 6 条指令 = 24 字节 (32-bit only, .option norvc 强制不生 RVC)。
#
# 主程序 (0x80000000 ~ 0x8000000f, 16 字节):
#   addi x1, x0, 1     ; 0x80000000: 跑过标记                              (0x00100093)
#   .word 0            ; 0x80000004: illegal RVC inst (RV spec: inst[1:0]=00
#                        且 funct3 不识别 → unsupp), 触发 trap_raise_exception
#                        (cause=2 illegal, tval=0)                        (0x00000000)
#   addi x2, x0, 99    ; 0x80000008: 不应被执行 (主程序在 .word 0 处被 trap
#                        截断, 跳到 xtvec = 0x80000010)                    (0x06300113)
#   ecall              ; 0x8000000c: 不应被执行                            (0x00000073)
#
# handler (0x80000010 ~ 0x80000017, 8 字节; main.c 设 xtvec[PRIV_M] = 0x80000010):
#   addi x3, x0, 7     ; 0x80000010: handler 跑过标记                      (0x00700193)
#   ecall              ; 0x80000014: handler 末尾 (OP_UNSUPPORTED → 第二次
#                        trap, in_trap 1→2, 字段更新为 cause=2 tval=0x73
#                        epc=0x80000014); 第三次 trap (in_trap 2→3) 候选 A
#                        早 return 不 deliver, 字段保留第二次状态           (0x00000073)
#
# 期望 (main 末尾输出):
#   x1=1, x2=0, x3=7, pc=0x80000010
#   in_trap=3 mcause=2 mtval=0x00000073 mepc=0x80000014 mtvec=0x80000010
#   dispatcher halted: in_trap=3 total_count=3 pc=0x80000010

.section .text
.option norvc                       # 强制不生 RVC; 主程序所有指令都 32-bit
.globl _start
_start:
    addi x1, x0, 1      # 0x80000000: x1 = 1 (跑过标记)
    .word 0             # 0x80000004: illegal, 触发 trap → 跳 xtvec
    addi x2, x0, 99     # 0x80000008: 不该跑
    ecall               # 0x8000000c: 不该跑

handler:
    addi x3, x0, 7      # 0x80000010: handler 起点 (= main.c 设的 xtvec)
    ecall               # 0x80000014: handler 末尾 ecall (第 2 次 + 第 3 次 trap)
