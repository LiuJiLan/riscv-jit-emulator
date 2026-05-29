# a_01_7 / 01_trap_priv_save_restore
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验证 trap_set_state 真切 priv + 写 mstatus.MPP/MPIE/MIE; OP_MRET 真切 priv (从 MPP 恢复)。
# M 自我 trap chain (M → ecall → handler → mret → M); 当前 deliver_priv hard-code M, 切前后
# priv 都是 M, 但 mstatus 字段保存/恢复的代码路径完整跑通。
# 收尾改 test_dev FINISHER PASS 干净停机。
# (注: 原停机靠 csrw mtvec,x0 + 2nd ecall 强制 triple fault — a_03 Smdbltrp 后 triple fault
#  变 MDT abort; 故收尾改 test_dev 干净停机。_start 0x0c 处已有 csrw 0x310 清 MDT (首个 trap
#  handler 能跑), 不重复加。原 triple-fault/in_trap=3 收尾叙事见 REVIEW B.3 段。)
#
# ===== 思路 =====
# 控制流:
#   _start: la+csrw mtvec=handler → 清 MDT → 设 mstatus.MIE=1 → ecall(1st trap)
#   1st trap → handler: csrr x1=mstatus(0x1880), csrr x5=mepc, x5+4, csrw mepc, mret → 回主程序
#   主程序续: csrr x2=mstatus(0x88) → test_dev FINISHER PASS → dispatcher 退主循环
# 验证点: x1=0x1880 = trap 后 mstatus 保存; x2=0x88 = mret 后恢复。

.section .text
.option norvc
.globl _start
_start:
    la   x4,  handler       # auipc + addi (la pseudo, 2 条 4 字节)
    csrw mtvec, x4          # mtvec = handler

    # ---- 设 mstatus.MIE = 1 (验 trap 时 MPIE 真保存了 MIE 的旧值) ----
    csrw 0x310, x0          # mstatush 清 MDT (否则下条 MIE 写被强制 0, 且首个 trap 撞 MDT=1 abort)
    addi x10, x0, 8         # x10 = 0x8 = MSTATUS_MIE 位
    csrw mstatus, x10       # mstatus = 0x8

    # ---- 触发 1st trap (ecall from M, cause=11) ----
    ecall                   # trap → handler

    # ---- mret 回来后的落点 (handler 末 csrw mepc x5 让 mepc 跳过原 ecall) ----
    csrr x2,  mstatus       # x2 = mstatus 验 mret 后恢复 (= 0x88)

    # ---- 停机: test_dev FINISHER PASS ----
    li   x30, 0x100000      # test_dev base
    li   x31, 0x5555        # FINISHER PASS magic
    sw   x31, 0(x30)        # 干净停机: test_dev 置 exit_code=0 + SRS=0, dispatcher 退主循环
1:  j    1b                 # 防 fall-through 落进 handler 重跑污染 x1 (dispatcher 下个 block 末退)

handler:
    csrr x1,  mstatus       # x1 = mstatus 验 trap 后保存 (= 0x1880)
    csrr x5,  mepc          # x5 = mepc (= 1st ecall PC)
    addi x5,  x5, 4         # x5 = ecall PC + 4 (跳过 ecall, 指向下一条)
    csrw mepc, x5           # mepc = 下一条 PC
    mret                    # 切 priv from MPP = M; pc = mepc

# ===== 期望结果 =====
# 期望 (test_dev PASS 干净停机时):
#   x1 = 0x1880 = 6272      (handler 内 csrr mstatus: MPP=M=3<<11=0x1800 + MPIE=1<<7=0x80)
#   x2 = 0x88   = 136       (mret 后主程序 csrr mstatus: MIE=1 (恢复 from MPIE) + MPIE=1 (RV spec) +
#                             MPP=PRIV_U=0 (RV spec least-priv reset))
#   x3 = 0                  (没设)
#   x5 = 1st ecall PC + 4   (handler 内 mepc+4; 绝对值随 layout 漂, 见 REVIEW)
#   x30 = 0x100000 / x31 = 0x5555 (停机用 scratch: test_dev base / FINISHER PASS magic)
#   priv = 3                (M; mret 切回 M)
#   停机 = test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
# (注: 绝对 pc / total_count 见 REVIEW B.3 段; 不在断言内。)
