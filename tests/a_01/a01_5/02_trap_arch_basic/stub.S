# a_01_5_c / 02_trap_arch_basic
# 验证 trap_set_state 架构语义层 + 候选 A 早 return + dispatcher while(in_trap<3) 退出 +
#       sigsetjmp 一次性落点 + trap_raise_exception _Noreturn longjmp 路径。
# 9 条指令 = 36 字节 (32-bit only, .option norvc 强制不生 RVC)。
#
# 起步 (a_01_5_c 起): fixture 自己用 csrw mtvec 设 handler 地址 (a_01_5_b 时由 main.c
# hard-code 设, 现已删, fixture 必须自包含)。
#
# 主程序 (0x80000000 ~ 0x80000018, 28 字节):
#   la   x4, handler      ; auipc + addi 两条 pseudo, x4 = handler 地址
#   csrw mtvec, x4        ; mtvec = handler (csrrw x0, mtvec, x4 pseudo, mask 低 2 位)
#   addi x1, x0, 1        ; 主程序跑过标记
#   .word 0               ; illegal RVC inst, 触发 trap (cause=2)
#   addi x2, x0, 99       ; 不应被执行
#   ecall                 ; 不应被执行
#
# handler (0x8000001c ~ 0x80000023, 8 字节):
#   addi x3, x0, 7        ; handler 跑过标记 (handler 起点; mtvec 指向这里)
#   ecall                 ; handler 末尾, 第 2 次 trap (cause=11 env-call from M)
#
# 期望 (main 末尾输出):
#   x1=1, x2=0, x3=7, pc=0x80000020
#     (block 4 内 addi x3 跑完 pc += 4 → 0x20, ecall 触发第 3 次 trap 候选 A 早 return
#      不动 pc; pc 停在 ecall PC = 0x80000020)
#   in_trap=3, mcause=11, mtval=0, mepc=0x80000020, mtvec=0x8000001c
#     (字段保留第 2 次 trap = ecall 的状态; mtvec 由 fixture 自己写, 不再是 0x80000010)
#   dispatcher halted: in_trap=3 total_count=3 pc=0x80000020
#     (total_count = 3 来自 block 1 的 la + csrw 三条; block 2/3/4 内 trap 触发块的 count
#      被 longjmp 跳过 dispatcher 的 total_count += 行, 不计入)
#
# 控制流追踪:
#   block 1 (0x80000000): la(2) + csrw(boundary) → count_out=3, total_count += 3 = 3
#                          mtvec = 0x8000001c
#   block 2 (0x8000000c): addi x1 + .word 0 → trap_raise(2,0) longjmp (in_trap 0→1,
#                          _cause=2/_tval=0/_epc=0x10, regs[0]=mtvec=0x1c); total_count 不变
#   block 3 (0x8000001c handler): addi x3 + ecall → trap_raise(11,0) longjmp (in_trap 1→2,
#                          _cause=11/_tval=0/_epc=0x20, regs[0]=mtvec=0x1c); total_count 不变
#   block 4 (0x8000001c handler): addi x3 + ecall → trap_raise(11,0) longjmp (in_trap 2→3,
#                          候选 A 早 return; 字段保留第 2 次 = block 3 的); total_count 不变
#   while (3<3) false → 退出, total_count = 3

.section .text
.option norvc                       # 强制不生 RVC
.globl _start
_start:
    la   x4, handler        # 0x80000000-0x80000007: auipc x4, %hi + addi x4, x4, %lo
    csrw mtvec, x4          # 0x80000008: mtvec = handler (= 0x8000001c)
    addi x1, x0, 1          # 0x8000000c: x1 = 1 (跑过标记)
    .word 0                 # 0x80000010: illegal, 触发 trap (cause=2)
    addi x2, x0, 99         # 0x80000014: 不该跑
    ecall                   # 0x80000018: 不该跑

handler:
    addi x3, x0, 7          # 0x8000001c: handler 起点 (= mtvec)
    ecall                   # 0x80000020: handler 末尾, 第 2/3 次 trap (cause=11)
