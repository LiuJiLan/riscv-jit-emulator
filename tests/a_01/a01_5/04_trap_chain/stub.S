# a_01_5 / 04_trap_chain
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验证完整的 trap → handler → mret → 主程序继续 流程:
#   - ecall 触发 trap, _epc 写 ecall PC; handler 内手动 csrw mepc, mepc+4 让 mret 跳过 ecall
#     (RV spec ECALL: mret 默认跳 mepc, software 必须自己 +4 跳过 ecall)
#   - mret 真复位 in_trap=0 + 跳 xepc; mret 后主程序 addi x2 真跑 (= mret 路径生效证据)
#   - 主程序末尾 test_dev FINISHER PASS 干净停机 (SRS=0 退主循环, exit code 0)
# (注: 原停机靠 csrw mtvec,x0 + ecall 强制 triple fault — a_03 Smdbltrp 后 triple fault 变
#  MDT abort, 且 cpu_reset MDT=1 会让首个 trap 就 abort、handler 跑不到; 故 _start 先清 MDT,
#  收尾改 test_dev 干净停机。原 13 指令 triple-fault 收尾叙事见 REVIEW B.3 段。)
#
# ===== 思路 =====
# 控制流:
#   _start: 清 MDT → la+csrw 设 mtvec=handler → x1=1 → ecall (1st trap)
#   1st trap → handler: x3=7, csrr x5=mepc, x5+4, csrw mepc, mret → 回主程序 addi x2
#   主程序续: x2=99 → test_dev FINISHER PASS → dispatcher 下个 block 末 atomic_load SRS=0 退主循环
# 验证点: x2=99 = mret 真生效 (没 mret 跑不到这里); x3=7 = handler 真跑过。

.section .text
.option norvc
.globl _start
_start:
    csrw 0x310, x0          # 清 mstatush.MDT (Smdbltrp reset=1, 否则首个 trap 直接 MDT abort)
    la   x4, handler        # auipc + addi pseudo
    csrw mtvec, x4          # mtvec = handler 地址
    addi x1, x0, 1          # x1 = 1 (跑过标记)
    ecall                   # 1st trap → handler
    addi x2, x0, 99         # mret 回来后跑 (= mret 真生效的证据)
    li   x30, 0x100000      # test_dev base
    li   x31, 0x5555        # FINISHER PASS magic
    sw   x31, 0(x30)        # 干净停机: test_dev 置 exit_code=0 + SRS=0, dispatcher 退主循环
1:  j       1b                      # 防 fall-through: SRS=0 下个 block 边界退

handler:
    addi x3, x0, 7          # x3 = 7 (handler 跑过标记)
    csrr x5, mepc           # x5 = mepc (= 1st ecall PC)
    addi x5, x5, 4          # x5 = ecall PC + 4 (跳过 ecall, 指向 addi x2)
    csrw mepc, x5           # mepc = addi x2 的 PC
    mret                    # 跳 mepc, 复位 in_trap; 回主程序 addi x2

# ===== 期望结果 =====
# 期望 (test_dev PASS 干净停机时):
#   x1 = 1     (主程序起点 addi)
#   x2 = 99    (mret 回来后跑; mret 真生效的证据 — 没 mret 就跑不到这里)
#   x3 = 7     (handler 跑过)
#   x30 = 0x100000 / x31 = 0x5555 (停机用 scratch: test_dev base / FINISHER PASS magic)
#   停机 = test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
# (注: 绝对 pc / total_count 随 prelude + test_dev 收尾漂, 见 REVIEW B.3 段; 不在断言内。)
