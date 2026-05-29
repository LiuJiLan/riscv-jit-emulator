# a_01_7 / 02_u_mode_csr_priv_reject
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验证 csr_op 入口 priv 检查激活: U 模式写 M-only csr (mtvec 0x305, bits 9:8 = 11 = M-required)
# → trap cause=2 (illegal instruction)。
# 切 U 路径: M 模式设 mstatus.MPP=PRIV_U=0 + csrw mepc, u_target + mret → priv 切 U + pc=u_target。
# handler 抓 mcause/mtval/mepc, 验毕 test_dev FINISHER PASS 干净停机。
# (注: 原停机靠 csrw mtvec,x0 + ecall 强制 triple fault — a_03 Smdbltrp 后 triple fault 变
#  MDT abort, 且 cpu_reset MDT=1 会让首个 trap 就 abort、handler 跑不到; 故 _start 先清 MDT,
#  收尾改 test_dev 干净停机。原 triple-fault/in_trap=3 收尾叙事见 REVIEW B.3 段。)
#
# ===== 思路 =====
# 控制流:
#   _start: 清 MDT → la+csrw mtvec=handler → csrw mstatus,x0 (MPP=U) → la+csrw mepc=u_target → mret 切 U
#   u_target (U): csrw mtvec,x4 → priv 检查 fail → trap cause=2 → handler
#   handler (M): csrr x1=mcause(2), x2=mtval(raw_inst), x3=mepc → test_dev FINISHER PASS → 退主循环

.section .text
.option norvc
.globl _start
_start:
    csrw 0x310, x0          # 清 mstatush.MDT (否则首个 trap 撞 MDT=1 直接 abort, handler 跑不到)
    la   x4,  handler       # la pseudo (8 字节)
    csrw mtvec, x4          # mtvec = handler

    # ---- 设 mstatus.MPP = PRIV_U = 0 (mret 切 U 的关键) ----
    # 显式 csrw mstatus, x0 把所有 mstatus 字段清零 (cpu_create memset 后已经是 0, 但显式更
    # 清晰; 也复用了 a_01 fixture 启动协议风格)
    csrw mstatus, x0        # mstatus = 0 (MPP=U=0, MPIE=0, MIE=0)

    # ---- 设 mepc = u_target 地址 (mret 跳的目标) ----
    la   x6,  u_target      # la pseudo (8 字节)
    csrw mepc, x6           # mepc = u_target

    # ---- mret 切 priv → U; pc → u_target ----
    mret                    # priv = MPP = U; pc = mepc = u_target

u_target:
    # ---- priv = U; 试图写 M-only csr → trap cause=2 ----
    csrw mtvec, x4          # U 模式不能写 mtvec (priv 不足) → trap cause=2
    # ↑ csr_op 入口判 priv 检查 → trap_raise_exception(2, raw_inst); 不到此处

handler:
    csrr x1,  mcause        # x1 = mcause = 2 (illegal instruction)
    csrr x2,  mtval         # x2 = mtval = 0x30521073 (raw_inst of csrw mtvec, x4)
    csrr x3,  mepc          # x3 = mepc = u_target 处 csrw 的 PC

    # ---- 停机: test_dev FINISHER PASS ----
    li   x30, 0x100000      # test_dev base
    li   x31, 0x5555        # FINISHER PASS magic
    sw   x31, 0(x30)        # 干净停机: test_dev 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望:
#   x1 = 2          (mcause = illegal instruction)
#   x2 = 0x30521073 = 810684531
#                    (mtval = raw_inst of `csrw mtvec, x4`; 编码:
#                     csr=0x305 << 20 = 0x30500000
#                     rs1=4 << 15  = 0x00020000
#                     funct3=001 << 12 = 0x00001000
#                     rd=0 << 7    = 0
#                     opcode=0x73  = 0x00000073
#                     总 = 0x30521073)
#   x3 = u_target 处 csrw mtvec 的 PC (绝对值随 prelude 漂, 见 REVIEW)
#   x30 = 0x100000 / x31 = 0x5555 (停机用 scratch: test_dev base / FINISHER PASS magic)
#   priv = 3        (handler 在 M 模式跑; trap 切 priv 已切 M)
#   停机 = test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
# (注: 绝对 pc / total_count 见 REVIEW B.3 段; 不在断言内。)
