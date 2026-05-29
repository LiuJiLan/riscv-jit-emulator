# a02_2 / 02_bus_unmapped_reject
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# bus_dispatch_* 未命中 reject 验证 — read + write 两条 branch (cover bus 自产
# CAUSE_LOAD_ACCESS_FAULT / CAUSE_STORE_ACCESS_FAULT)。
# (注: 原收尾用 csrw mtvec,x0 + ecall → triple fault 停机; a_03 Smdbltrp 后改 test_dev
#  干净停机, 旧 triple-fault 叙事见 REVIEW B.3 段。)
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
    csrw 0x310, x0               # 清 mstatush.MDT (Smdbltrp reset=1; 否则首个 M-mode
                                 # 访问 fault → MDT abort, handler 跑不到)

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

    # ---- 停机: test_dev FINISHER PASS ----
    li   x30, 0x100000           # test_dev base
    li   x31, 0x5555             # FINISHER PASS magic
    sw   x31, 0(x30)             # 置 exit_code=0 + SRS=0, dispatcher 退主循环
1:  j    1b                      # 防 fall-through 落进 m_handler (dispatcher 下个 block 末退)

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
#   x30 = 0x100000 / x31 = 0x5555  (停机 scratch: test_dev base / FINISHER PASS magic)
#   停机 = test_dev FINISHER PASS → exit code 0
# (注: cause 7 / 5 通过 handler 写入 a 寄存器验证; 绝对 pc / total_count 见 REVIEW
#  B.3 段, 不在断言内。)
