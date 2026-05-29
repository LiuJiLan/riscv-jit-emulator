# a02_3 / 01_u_priv_reject
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# priv check fixture — U-mode 调 SYSTEM priv-required 指令 (MRET / SRET /
# SFENCE.VMA) reject 验证。
# (注: 原 case 2 handler 内 csrw mtvec,x0 + ecall → triple fault 停机; a_03 Smdbltrp
#  后改 test_dev 干净停机, 旧 triple-fault 叙事见 REVIEW B.3 段。)
#
# ===== 思路 =====
# 触发 case (3 个; 都期望 cause=2 illegal + tval=raw_inst):
#   case 0: U-mode mret           → cause 2, tval = 0x30200073
#   case 1: U-mode sret           → cause 2, tval = 0x10200073
#   case 2: U-mode sfence.vma x0,x0 → cause 2, tval = 0x12000073
#
# raw_inst 来源 (RV32 SYSTEM funct12/rs2/rs1/funct3/rd/opcode 编码):
#   MRET           = 0011000_00010_00000_000_00000_1110011 = 0x30200073
#   SRET           = 0001000_00010_00000_000_00000_1110011 = 0x10200073
#   SFENCE.VMA r,r = 0001001_rs2__rs1__000_00000_1110011
#     rs1=x0 rs2=x0 → 0x12000073
#
# 设计:
#   - 启动 priv=M (cpu_create 默认); 写 mstatus.MPP=PRIV_U=0 + mepc=u_test, mret
#     一次切到 U-mode (M-mode mret 走 PRIV_CHECK_OR_TRAP(PRIV_M) 通过, M >= M)
#   - U-mode 三条指令依次 trap; 每次 PRIV_CHECK_OR_TRAP 触发 cause 2 + tval=raw_inst
#     (interpreter case 入口判)
#   - mtvec 指 m_handler, handler 用 s0 计数分流, 写 cause / tval 到 a 寄存器
#   - medeleg=0 (默认; cpu_create 时 trap._medeleg=0), illegal-inst trap 全 deliver 到 M
#
# halt 路径: case 2 handler 内 test_dev FINISHER PASS → SRS=0, dispatcher 退主循环
#            (exit 0); case 0/1/2 的 cause/tval 在 a 寄存器上读出。

.section .text
.option norvc
.globl _start
_start:
    # mtvec → m_handler
    la   t0, m_handler
    csrw mtvec, t0

    # init counter
    li   s0, 0

    # 切到 U-mode: 清 mstatus.MPP (bits [12:11] = PRIV_U = 0b00)
    csrr t0, mstatus
    li   t1, 0x1800              # MSTATUS_MPP mask
    not  t1, t1
    and  t0, t0, t1              # 清 MPP → MPP=0=PRIV_U
    csrw mstatus, t0

    # mepc = u_test, mret 跳 U-mode
    la   t0, u_test
    csrw mepc, t0
    mret

# ----------------------------------------------------------------------------
# u_test — U-mode 三条 priv-required 指令; 每条 trap 后由 handler mret +4 推进
# ----------------------------------------------------------------------------
u_test:
    mret                          # case 0: PRIV_CHECK_OR_TRAP(PRIV_M); U < M → trap
    sret                          # case 1: PRIV_CHECK_OR_TRAP(PRIV_S); U < S → trap
    sfence.vma x0, x0             # case 2: PRIV_CHECK_OR_TRAP(PRIV_S); U < S → trap
    # 不应到达 (case 2 handler 内 halt)

# ----------------------------------------------------------------------------
# m_handler — switch on s0 写 cause / tval 到 a 寄存器, mret +4 (3 条都 4 byte)
# ----------------------------------------------------------------------------
m_handler:
    csrr t0, mcause
    csrr t1, mtval

    li   t2, 0
    beq  s0, t2, case0
    li   t2, 1
    beq  s0, t2, case1
    # s0 == 2: case 2
case2:
    mv   a4, t0
    mv   a5, t1
    # ---- 停机: test_dev FINISHER PASS (handler 内, M-mode) ----
    li   x30, 0x100000           # test_dev base
    li   x31, 0x5555             # FINISHER PASS magic
    sw   x31, 0(x30)             # 置 exit_code=0 + SRS=0, dispatcher 退主循环
1:  j    1b                      # 防 fall-through 落进 case0 (dispatcher 下个 block 末退)

case0:
    mv   a0, t0
    mv   a1, t1
    j    epilogue
case1:
    mv   a2, t0
    mv   a3, t1
    j    epilogue

epilogue:
    addi s0, s0, 1
    csrr t3, mepc
    addi t3, t3, 4               # 3 条触发指令都是 4-byte (RV32 SYSTEM, .option norvc)
    csrw mepc, t3
    mret

# ===== 期望结果 =====
# 期望最终寄存器:
#   a0 = 2          case 0 mcause (illegal instruction)
#   a1 = 0x30200073 case 0 mtval  (mret raw_inst)
#   a2 = 2          case 1 mcause
#   a3 = 0x10200073 case 1 mtval  (sret raw_inst)
#   a4 = 2          case 2 mcause
#   a5 = 0x12000073 case 2 mtval  (sfence.vma raw_inst)
#   s0 = 2          (case 2 在 epilogue 自增前就 halt; 3 case 都进过 handler 但 s0 停在 2;
#                    原 doc 写 s0=3 是 epilogue 自增的旧叙述, 实跑一直是 2, 见 REVIEW B.3)
#   x30 = 0x100000 / x31 = 0x5555  (停机 scratch: test_dev base / FINISHER PASS magic)
#   停机 = test_dev FINISHER PASS → exit code 0
# (注: 绝对 pc / total_count 见 REVIEW B.3 段; 不在断言内。)
