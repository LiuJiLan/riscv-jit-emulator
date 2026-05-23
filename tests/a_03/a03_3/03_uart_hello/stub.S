# a03_3 / 03_uart_hello
#
# ===== 测试目的 =====
# Hello world 纯 TX 轮询 — 不接 PLIC / 不接 trap, 纯走 UART TX fast path:
#   1. la a0, msg (pc-relative)
#   2. lbu 字节 → beqz 终止
#   3. lbu LSR (0x10000005) → andi 0x20 (THRE bit) → beqz back (永远不阻塞,
#      因 LSR_BASE = THRE|TEMT = 0x60, THRE bit 永远 1)
#   4. sb 字符 → THR (0x10000000)
#   5. addi 指针 → loop
#   6. msg 完后 ecall halt
#
# milestone "顺手 hello world" — a_03_end 边界不钉死, 这里 hello world 顺手副产物.
# pipe / file 重定向时 stdout 不卡 buffer (main setvbuf _IONBF).
#
# ===== 思路 =====
# .asciz "Hello, RISC-V!\n" 一步给 15 字符 + \0; .text section 内 msg 紧跟 ecall 之后,
# 用 la 走 auipc/addi pc-relative pair 拿地址, 跟 .text 起点 0x80000000 解耦.
#
# poll 段一定执行至少一次 (lbu LSR → andi 0x20 → bnez 跳过 sb? 不: bnez 是 "branch if
# non-zero", 我们要的是 "非零就 sb, 零就 retry"); 实际我用 beqz retry 形式 — andi 后
# 跳条件 beqz back-edge:
#   poll_loop:
#       lbu  t1, 5(a1)
#       andi t1, t1, 0x20
#       beqz t1, poll_loop      # THRE=0 retry (实际 THRE=1 永远, 一次过)

.equ UART_BASE,         0x10000000
.equ UART_REG_THR,      0
.equ UART_REG_LSR,      5

.section .text
.option norvc
.globl _start

_start:
    la      a0, msg
    li      a1, UART_BASE

loop:
    lbu     t0, 0(a0)
    beqz    t0, done

poll_loop:
    lbu     t1, UART_REG_LSR(a1)
    andi    t1, t1, 0x20            # THRE bit
    beqz    t1, poll_loop           # THRE=1 永远, 立即过

    sb      t0, UART_REG_THR(a1)
    addi    a0, a0, 1
    j       loop

done:
    csrw    mtvec, x0
    ecall

# 字符串放 ecall 之后 — la pc-relative 自动算偏移
msg:
    .asciz "Hello, RISC-V!\n"

# ===== 期望结果 =====
# 期望 stdout:
#   "Hello, RISC-V!\n"
#
# 期望 trap dump (halt 路径 triple fault):
#   in_trap = 3
#   mcause  = 1     (CAUSE_INST_ACCESS_FAULT; ecall + mtvec=0 取指 fault)
#
# 不期望:
#   - poll_loop 死循环 (LSR.THRE 一直 0 — 说明 UART read LSR 路径 bug)
#   - [bus] CAUSE_*_ACCESS_FAULT (lbu/sb size=1 应走 UART size=1 path 不 fault)
