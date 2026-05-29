# a_01_7 / 04_u_mode_ecall_chain
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验证完整的 syscall-like 端到端路径: M 设 handler → mret 切 U → U 跑程序 → ecall(U) cause=8 →
# M handler 处理 → mret 回 U → U 继续跑 → 再次 ecall(U) → handler 决定停机 (test_dev PASS)。
#
# 覆盖范围:
#   1. ecall from U → cause=8 (验 8 + PRIV_U=0 公式; 之前 fixture 都只验 cause=11 from M)
#   2. mret 真切回 U (cross-priv mret, 跟 a_01_5 04_trap_chain M 自我 mret 风格不同)
#   3. U 多次 ecall 都走同 m_handler (handler 复用)
#   4. U mode GPR (x1/x2) 跨 trap/mret 保留 (precise trap 语义; trap_set_state / mret 不动 GPR)
#   5. handler GPR (x3) 跨多次 trap/mret 调用累计 (host 路径不动 guest GPR)
# (注: 原停机靠 csrw mtvec,x0 + ecall 强制 triple fault — a_03 Smdbltrp 后 triple fault 变
#  MDT abort, 且 cpu_reset MDT=1 会让首个 trap 就 abort、handler 跑不到; 故 _start 先清 MDT,
#  收尾改 test_dev 干净停机。原 triple-fault/in_trap=3 收尾叙事见 REVIEW B.3 段。)
#
# ===== 思路 =====
# 控制流:
#   _start: 清 MDT → la+csrw mtvec=m_handler → csrw mstatus,x0 (MPP=U) → la+csrw mepc=u_program → mret 切 U
#   u_program (U): addi x1=1 → ecall(1st) → m_handler → mret 回 → addi x2=2 → ecall(2nd) → m_handler
#   m_handler (M): x3++ → beq x3==2? 第 1 次 mret 回 U; 第 2 次走 halt → test_dev FINISHER PASS → 退主循环
# 验证点: x1=1/x2=2 (U GPR 跨 trap 保留 + mret 真生效); x3=2 (handler 累计进 2 次)。

.section .text
.option norvc
.globl _start
_start:
    csrw 0x310, x0          # 清 mstatush.MDT (否则首个 trap 撞 MDT=1 直接 abort, handler 跑不到)

    # ---- 设 trap handler ----
    la   x4, m_handler      # la pseudo (8 字节)
    csrw mtvec, x4          # mtvec = m_handler

    # ---- 设 mstatus.MPP = PRIV_U = 0 (mret 切 U 的关键) ----
    csrw mstatus, x0        # mstatus = 0 (MPP=U, MPIE=0, MIE=0)

    # ---- 设 mret 目标 = u_program ----
    la   x6, u_program      # la pseudo (8 字节)
    csrw mepc, x6           # mepc = u_program

    # ---- mret → priv=U, pc=u_program ----
    mret

u_program:
    # ---- 1st ecall from U ----
    addi x1, x0, 1          # x1 = 1 (syscall arg-like 标识)
    ecall                   # cause=8 (ecall from U) → m_handler

    # ---- mret 回来后跑这条 (handler 把 mepc +4 跳过原 ecall) ----
    addi x2, x0, 2          # x2 = 2 (验 mret 真生效)

    # ---- 2nd ecall from U ----
    ecall                   # cause=8 → m_handler (此次 handler 不 mret, 直接停机)

    # ---- 不到此处 ----

m_handler:
    # x3 是 handler 累计计数器 (host 路径不动 guest GPR, 跨 trap/mret 累计)
    addi x3, x3, 1          # x3++ (1, 然后 2)
    addi x13, x0, 2         # x13 = 2 (停机阈值)
    beq  x3, x13, halt      # x3==2 → halt (第 2 次到)

    # ---- 第 1 次 (x3=1 != 2) 走这里, mret 回 U 继续 ----
    csrr x5, mepc           # x5 = mepc = 1st ecall PC
    addi x5, x5, 4          # x5 += 4 (跳过 ecall, 指向 next inst = addi x2)
    csrw mepc, x5           # mepc = next inst
    mret                    # 切 priv=MPP=U; pc=mepc

halt:
    # ---- 第 2 次到 (x3==2): handler 不 mret, test_dev FINISHER PASS 干净停机 ----
    li   x30, 0x100000      # test_dev base
    li   x31, 0x5555        # FINISHER PASS magic
    sw   x31, 0(x30)        # 干净停机: test_dev 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望 (main.c dump):
#   x1 = 1                  (U 1st ecall 前设)
#   x2 = 2                  (mret 真生效证据 — 没 mret 跑不到这条)
#   x3 = 2                  (handler 累计计数器, ecall 进 handler 2 次)
#   x13 = 2                 (停机阈值比较 reg)
#   x30 = 0x100000 / x31 = 0x5555 (停机用 scratch: test_dev base / FINISHER PASS magic)
#   priv = 3                (handler 在 M 模式)
#   停机 = test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
# (注: 绝对 pc / total_count 见 REVIEW B.3 段; 不在断言内。)
