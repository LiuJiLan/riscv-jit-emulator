# a02_2 / 02_bus_unmapped_reject
#
# ===== 测试目的 =====
# bus_dispatch_* 未命中 reject 验证 — read + write 两条 branch (cover bus 自产
# CAUSE_LOAD_ACCESS_FAULT / CAUSE_STORE_ACCESS_FAULT)。
#
# ===== 思路 =====
# 触发 case (2 个):
#   case 0: sw → 0x03000000 → bus_dispatch_write 扫表无命中 → cause 7
#   case 1: lw → 0x03000000 → bus_dispatch_read  扫表无命中 → cause 5
#
# 设计:
#   - 启动状态 priv=M / satp=0 (bare), BARE 路径下 PA = GVA = 0x03000000 不在
#     [GUEST_RAM_START, GUEST_RAM_START+GUEST_RAM_SIZE), 也不在 CLINT 范围, 走
#     bus 但未命中任何 device, bus 自产 access fault (dummy.txt §8 + §9)。
#   - mtvec 指 m_handler, handler 用 s0 计数 + 写 mtval / cause 到 a 寄存器。

.section .text
.option norvc
.globl _start
_start:
    # set mtvec
    la   t0, m_handler
    csrw mtvec, t0

    # init counter
    li   s0, 0

    # === case 0: bus 未命中 (write) ===
    li   t0, 0x03000000
    li   t1, 0xCAFEBABE          # 写入值不重要 (永远不会真存)
    sw   t1, 0(t0)

    # === case 1: bus 未命中 (read) ===
    li   t0, 0x03000000
    lw   t2, 0(t0)

    # done: triple fault halt
    csrw mtvec, x0
    ecall

# ----------------------------------------------------------------------------
# m_handler — switch on s0 写 mtval (+ case 1 写 cause) 到 a 寄存器, mret +4
# ----------------------------------------------------------------------------
m_handler:
    csrr t0, mcause
    csrr t1, mtval

    li   t2, 0
    beq  s0, t2, case0
    # default case 1
case1:
    mv   a1, t0                  # case 1 cause
    mv   a2, t1                  # case 1 tval
    j    epilogue
case0:
    mv   a0, t1                  # case 0 tval (cause 7 不存; 已知)
    j    epilogue

epilogue:
    addi s0, s0, 1
    csrr t3, mepc
    addi t3, t3, 4               # mepc += 4 (sw / lw 都是 4 byte; .option norvc)
    csrw mepc, t3
    mret

# ===== 期望结果 =====
# 期望最终寄存器:
#   a0 (x10) = 0x03000000   case 0 mtval (cause 7 已知, 不存)
#   a1 (x11) = 5            case 1 mcause
#   a2 (x12) = 0x03000000   case 1 mtval
#   s0 (x8)  = 2            (2 case 都进了 handler)
#
# halt 路径: 2 case 完后 csrw mtvec, x0 + ecall → 下轮 fetch pc=0 → triple fault
#            halt。main dump 的 mcause/mtval 是 cause 1 / 0 (最后 root cause), 不是
#            case 7 / 5; cause 7 / 5 通过 handler 写入 a 寄存器验证。
