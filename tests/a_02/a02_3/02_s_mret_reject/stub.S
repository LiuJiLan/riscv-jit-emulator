# a02_3 / 02_s_mret_reject
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# priv check fixture — S-mode 调 MRET reject。
# (注: 原 handler 内 csrw mtvec,x0 + ecall → triple fault 停机; a_03 Smdbltrp 后改
#  test_dev 干净停机, 旧 triple-fault 叙事见 REVIEW B.3 段。)
#
# MRET 仅在 priv >= M 时合法 (M-only system 指令); S-mode 调 MRET 触发
# PRIV_CHECK_OR_TRAP(PRIV_M) cause 2 illegal + tval=raw_inst (interpreter case
# 入口判)。
#
# ===== 思路 =====
# 触发 case (1 个):
#   case 0: S-mode mret → cause 2, tval = 0x30200073
#
# raw_inst:
#   MRET = 0011000_00010_00000_000_00000_1110011 = 0x30200073
#
# 设计:
#   - 启动 priv=M; 写 mstatus.MPP=PRIV_S=1 + mepc=s_test, mret 一次切到 S-mode
#   - S-mode mret → PRIV_CHECK_OR_TRAP(PRIV_M) trap (S=1 < M=3)
#   - mtvec 指 m_handler; medeleg=0 (默认; 显式确保 illegal-inst trap deliver 到 M
#     而不是经 S 委托回 S 形成循环)
#
# halt: handler 内 test_dev FINISHER PASS → SRS=0, dispatcher 退主循环 (exit 0)。

.section .text
.option norvc
.globl _start
_start:
    # mtvec → m_handler
    la   t0, m_handler
    csrw mtvec, t0

    # medeleg=0: illegal-instruction trap (cause 2) deliver 到 M, 不走 S 委托
    csrw medeleg, x0

    # init counter
    li   s0, 0

    # 切到 S-mode: 写 mstatus.MPP=PRIV_S=1 (MPP=01, MPP 在 bits [12:11])
    csrr t0, mstatus
    li   t1, 0x1800              # MSTATUS_MPP mask
    not  t1, t1
    and  t0, t0, t1              # 清 MPP
    li   t1, 0x0800              # PRIV_S=1 << MPP_SHIFT(11) = 0x800
    or   t0, t0, t1
    csrw mstatus, t0

    # mepc = s_test, mret 跳 S-mode
    la   t0, s_test
    csrw mepc, t0
    mret

# ----------------------------------------------------------------------------
# s_test — S-mode 一条 priv-required 指令 (MRET)
# ----------------------------------------------------------------------------
s_test:
    mret                          # case 0: S < M → PRIV_CHECK_OR_TRAP(PRIV_M) trap
    # 不应到达 (handler 内 halt)

# ----------------------------------------------------------------------------
# m_handler — 写 cause / tval 到 a0/a1, 直接 halt (单 case, 无 switch)
# ----------------------------------------------------------------------------
m_handler:
    csrr t0, mcause
    csrr t1, mtval

    mv   a0, t0
    mv   a1, t1
    addi s0, s0, 1

    # ---- 停机: test_dev FINISHER PASS (handler 内, M-mode) ----
    li   x30, 0x100000           # test_dev base
    li   x31, 0x5555             # FINISHER PASS magic
    sw   x31, 0(x30)             # 置 exit_code=0 + SRS=0, dispatcher 退主循环
1:  j    1b                      # 防 fall-through (dispatcher 下个 block 末退)

# ===== 期望结果 =====
# 期望最终寄存器:
#   a0 = 2          mcause (illegal instruction)
#   a1 = 0x30200073 mtval  (mret raw_inst)
#   s0 = 1          (handler 进过 1 次)
#   x30 = 0x100000 / x31 = 0x5555  (停机 scratch: test_dev base / FINISHER PASS magic)
#   停机 = test_dev FINISHER PASS → exit code 0
# (注: 绝对 pc / total_count 见 REVIEW B.3 段; 不在断言内。)
