# a_01_5 / 05_triple_fault_halt
#
# ===== 测试目的 =====
# 验证项目内部停机协议: M-mode 下触发 trap 而 mstatus.MDT=1 (cpu_reset 默认) → Smdbltrp
# 双重 trap → dispatcher MDT abort 停机。2 条指令 = 8 字节 (32-bit only, .option norvc)。
# (注: 本 fixture 是停机协议本身的演示, 故意 *不* 清 MDT prelude — 旧实现靠 mtvec=0 +
#  ecall 触发 triple fault 链 (in_trap 计数到 3 退出); a_03 Smdbltrp 落地后该机制改为
#  "trap 撞 MDT=1 → 直接 MDT abort", 本 fixture 随之改测后者。其余借 triple-fault 当停机
#  的 fixture 已改 test_dev 干净停机, 唯独本 fixture 保留作 abort 路径的回归点。)
#
# ===== 思路 =====
# cpu_reset 把 mstatus.MDT=1 (Smdbltrp spec §3.1.6.2)。guest 第一条 trap (这里 ecall)
# 进 M-mode 时撞 MDT=1 → 双重 trap → dispatcher 检出 HART_MDT → ABORT_MASK cleanup 退出。
# csrw mtvec, x0 是历史残留 (旧 triple-fault 起点); MDT abort 不走 mtvec, 此行现在无实际作用,
# 保留以最小化 diff + 留 trail。
# 控制流:
#   block 1: csrw mtvec, x0 (boundary)   count=1, total=1
#   block 2: ecall → trap 进 M 撞 MDT=1 → MDT abort 停机 (pc 停在 ecall PC)

.section .text
.option norvc
.globl _start
_start:
    csrw mtvec, x0          # mtvec = 0 (历史残留; MDT abort 不走 mtvec)
    ecall                   # 触发 trap 进 M → 撞 MDT=1 → MDT abort 停机

# ===== 期望结果 =====
# 期望:
#   x1 = x2 = x3 = 0       (没设)
#   停机 = MDT abort (mstatus.MDT=1; Smdbltrp 双重 trap)。dump 显 mstatus.MDT=1。
#     旧叙事 "in_trap=3 / triple fault / mcause=1 mmu fail at 0" 已废 (in_trap 字段 a_03 删;
#     MDT abort 抢占在 cause 写入前, mcause 停 0)。
# (注: 绝对 pc / total_count 见 REVIEW B.3 段; 不在断言内。)
