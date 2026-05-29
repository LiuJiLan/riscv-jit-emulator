# a_01_5 / 02_trap_arch_basic
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验证 trap_set_state 架构语义层 + trap_raise_exception _Noreturn longjmp 路径 +
#       sigsetjmp 一次性落点: illegal 指令 (.word 0) → trap (cause=2) → 跳 mtvec handler。
# (注: 原 fixture 还验"候选 A 早 return + dispatcher while(in_trap<3) 退出" — a_03 Smdbltrp
#  后 in_trap 字段废、triple-fault 计数机制改 MDT abort; 故收尾改 test_dev 干净停机 + 开头
#  清 MDT prelude 让 illegal trap 能投递到 handler。旧 in_trap 叙事见 REVIEW B.3 段。)
#
# ===== 思路 =====
# 起步: fixture 自己用 csrw mtvec 设 handler 地址 (main.c 不 hard-code, fixture 自包含)。
# 控制流:
#   _start: 清 MDT → la+csrw 设 mtvec=handler → x1=1 → .word 0 (illegal, trap cause=2)
#   trap → handler: x3=7 (handler 跑过标记) → test_dev FINISHER PASS 干净停机
#   addi x2 / ecall 在 .word 0 之后, 永不执行 (x2 保持 0 = "trap 真打断主程序" 的证据)

.section .text
.option norvc                       # 强制不生 RVC
.globl _start
_start:
    csrw 0x310, x0          # 清 mstatush.MDT (否则 illegal trap 撞 MDT=1 直接 abort, handler 跑不到)
    la   x4, handler        # auipc + addi pseudo, x4 = handler 地址
    csrw mtvec, x4          # mtvec = handler
    addi x1, x0, 1          # x1 = 1 (跑过标记)
    .word 0                 # illegal, 触发 trap (cause=2) → 跳 handler
    addi x2, x0, 99         # 不该跑 (trap 已打断)
    ecall                   # 不该跑

handler:
    addi x3, x0, 7          # x3 = 7 (handler 起点; mtvec 指向这里)
    li   x30, 0x100000      # test_dev base
    li   x31, 0x5555        # FINISHER PASS magic
    sw   x31, 0(x30)        # 干净停机: test_dev 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望 (test_dev PASS 干净停机时):
#   x1 = 1     (主程序起点 addi)
#   x2 = 0     (.word 0 trap 打断主程序, addi x2 永不执行 — trap 真生效的证据)
#   x3 = 7     (handler 跑过)
#   x30 = 0x100000 / x31 = 0x5555  (停机 scratch)
#   停机 = test_dev FINISHER PASS → exit code 0 (illegal→handler 投递 OK, 非 abort)
# (注: 绝对 pc / total_count 见 REVIEW B.3 段; 不在断言内。)
