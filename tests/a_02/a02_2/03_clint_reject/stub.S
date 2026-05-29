# a02_2 / 03_clint_reject
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# CLINT 内部 reject 验证 — clint_write 4 类 branch + clint_read 1 类 sample。
# (注: 原收尾用 csrw mtvec,x0 + ecall → triple fault 停机; a_03 Smdbltrp 后改 test_dev
#  干净停机, 旧 triple-fault 叙事见 REVIEW B.3 段。)
#
# ===== 思路 =====
# clint.c read/write 是对称镜像 (cause 5 vs 7 区别, 内部 branch 完全同源); write
# 端 cover 4 类 reject branch, read 端用 1 个 sample (idx 越界) 验 cause 5 路径
# 走通 + clint_read 函数被调到 — 5 个 case 已 cover 全部 branch。
#
# 触发 case (5 个):
#   case 0: sw → 0x02000004 → clint_write msip idx 越界  (idx=1 ≥ MAX_HARTS=1) → cause 7
#   case 1: sw → 0x02004008 → clint_write mtimecmp idx 越界 (idx=1)              → cause 7
#   case 2: sw → 0x0200C000 → clint_write final else (off >= MTIME_OFF+4)        → cause 7
#   case 3: sb → 0x02000000 → clint_write size != 4                              → cause 7
#   case 4: lw → 0x02000004 → clint_read  msip idx 越界 (verify cause 5 path)    → cause 5

.section .text
.option norvc
.globl _start
_start:
    csrw 0x310, x0               # 清 mstatush.MDT (Smdbltrp reset=1; 否则首个 M-mode
                                 # 访问 fault → MDT abort, handler 跑不到)

    # set mtvec
    la   t0, m_handler
    csrw mtvec, t0

    # init
    li   s0, 0
    li   s1, 0

    # === case 0: msip idx 越界 (write) ===
    li   t0, 0x02000004          # msip[1] 不存在 (MAX_HARTS=1)
    li   t1, 0
    sw   t1, 0(t0)

    # === case 1: mtimecmp idx 越界 (write) ===
    li   t0, 0x02004008          # mtimecmp[1] 不存在
    sw   t1, 0(t0)

    # === case 2: CLINT 内 final else (write) ===
    li   t0, 0x0200C000          # off >= CLINT_MTIME_OFF+4, 走 final else
    sw   t1, 0(t0)

    # === case 3: size != 4 (sb) (write) ===
    li   t0, 0x02000000          # msip[0] 地址; sb 触发 size != 4
    sb   t1, 0(t0)

    # === case 4: msip idx 越界 (read) ===
    li   t0, 0x02000004
    lw   t2, 0(t0)

    # ---- 停机: test_dev FINISHER PASS ----
    li   x30, 0x100000           # test_dev base
    li   x31, 0x5555             # FINISHER PASS magic
    sw   x31, 0(x30)             # 置 exit_code=0 + SRS=0, dispatcher 退主循环
1:  j    1b                      # 防 fall-through 落进 m_handler (dispatcher 下个 block 末退)

# ----------------------------------------------------------------------------
# m_handler — switch on s0 写 mtval (+ case 4 写 cause) 到 a 寄存器, mret +4
# ----------------------------------------------------------------------------
m_handler:
    csrr t0, mcause
    csrr t1, mtval
    add  s1, s1, t0              # cause sanity sum

    # switch on s0 → case0..case4 (default case 4)
    li   t2, 0
    beq  s0, t2, case0
    li   t2, 1
    beq  s0, t2, case1
    li   t2, 2
    beq  s0, t2, case2
    li   t2, 3
    beq  s0, t2, case3
    # default case 4 (read; cause + tval 都存)
case4:
    mv   a4, t0                  # case 4 cause = 5
    mv   a5, t1                  # case 4 tval
    j    epilogue
case3:  mv   a3, t1
    j    epilogue
case2:  mv   a2, t1
    j    epilogue
case1:  mv   a1, t1
    j    epilogue
case0:  mv   a0, t1
    j    epilogue

epilogue:
    addi s0, s0, 1
    csrr t3, mepc
    addi t3, t3, 4               # mepc += 4 (sw / lw / sb 都是 4 byte; .option norvc)
    csrw mepc, t3
    mret

# ===== 期望结果 =====
# 期望最终寄存器:
#   a0 (x10) = 0x02000004   case 0 mtval
#   a1 (x11) = 0x02004008   case 1 mtval
#   a2 (x12) = 0x0200C000   case 2 mtval
#   a3 (x13) = 0x02000000   case 3 mtval
#   a4 (x14) = 5            case 4 mcause
#   a5 (x15) = 0x02000004   case 4 mtval
#   s0 (x8)  = 5            (5 case 都进了 handler)
#   s1 (x9)  = 4*7 + 5 = 33 (cause sanity sum)
#   x30 = 0x100000 / x31 = 0x5555  (停机 scratch: test_dev base / FINISHER PASS magic)
#   停机 = test_dev FINISHER PASS → exit code 0
# (注: cause 7/5 通过 handler 写入 a/s 寄存器 dump 验证; 绝对 pc / total_count 见
#  REVIEW B.3 段, 不在断言内。)
