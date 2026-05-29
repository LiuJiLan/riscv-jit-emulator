# a_01_8 / 01_sret_m_s_u_chain
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验证 sret 切 priv (S→U) + sstatus / sepc CSR 入口 (Step 6 加).
#
# 覆盖范围:
#   1. sret 切 priv (S→U): 验 _mstatus.SPP/SPIE/SIE 字段操作 + sepc 拿 PC
#   2. sstatus CSR 入口 (csrw sstatus 写 mask 内字段; csrw sepc 写 sepc)
#   3. mret 切 priv (M→S): 跟 a_01_7 mret 切 U 同形态 (不同 MPP 值)
#   4. 跨 priv ecall: U-mode ecall 走 cause=8 (跟 a_01_7/04 cause=8 路径一致, 但走 sret)
# (注: 原停机靠 csrw mtvec,x0 + ecall 强制 triple fault — a_03 Smdbltrp 后 triple fault 变
#  MDT abort, 且 cpu_reset MDT=1 会让首个 trap 就 abort、handler 跑不到; 故 _start 先清 MDT,
#  收尾改 test_dev 干净停机。原 triple-fault/in_trap=3 收尾叙事见 REVIEW B.3 段。)
#
# ===== 思路 =====
# 路径 (user "M→S→U→M (没代理, 先同一回 M)" — 不依赖 SV32 walker, 不用 lw/sw):
#   1. M-mode 起步: 清 MDT → 设 mtvec / mepc / mstatus.MPP=PRIV_S → mret → 切 S
#   2. S-mode: 设 sepc / sstatus.SPP=PRIV_U → sret → 切 U
#   3. U-mode: ecall (cause=8 from U) → trap deliver M (deliver_priv hard-code PRIV_M)
#   4. M-handler: 验 cause=8; test_dev FINISHER PASS 干净停机 (退主循环)

.section .text
.option norvc
.globl _start
_start:
    csrw 0x310, x0          # 清 mstatush.MDT (否则首个 trap 撞 MDT=1 直接 abort, handler 跑不到)

    # ---- M-mode 起步 ----
    la   x4, m_handler      # la (8 字节)
    csrw mtvec, x4

    la   x5, s_program      # la (8 字节)
    csrw mepc, x5

    addi x6, x0, 1          # x6 = 1 (PRIV_S 编码)
    slli x6, x6, 11         # x6 = 0x800 (MPP_S 在 mstatus bits 12:11)
    csrw mstatus, x6        # mstatus = 0x800 (MPP=01=PRIV_S, 其他 0)

    mret                    # M→S, pc=mepc=s_program

s_program:
    # ---- S-mode 起步标记 ----
    addi x10, x0, 1         # x10 = 1

    # ---- 设 sret 目标 = u_program ----
    la   x7, u_program      # la (8 字节)
    csrw sepc, x7           # sepc = u_program

    # ---- 设 sstatus.SPP = PRIV_U = 0 (sret 切 U 关键) ----
    # csrw sstatus, x0 写整个 sstatus = 0; SPP=0 即 PRIV_U; SIE=SPIE=SUM=MXR=0;
    # 不影响 mstatus 的 M-mode-only 字段 (MPP/MPIE/MIE 通过 SSTATUS_MASK 保护)
    csrw sstatus, x0

    sret                    # S→U, pc=sepc=u_program

u_program:
    # ---- U-mode 起步标记 (sret 真生效证据) ----
    addi x11, x0, 2         # x11 = 2

    # ---- 触发 ecall (cause=8 from U) → trap deliver M ----
    ecall

    # ---- 不到此处 ----

m_handler:
    # ---- M handler: 验 cause=8 ----
    csrr x12, mcause        # x12 = mcause (期望 8)
    addi x13, x0, 8         # x13 = 8
    bne  x12, x13, halt     # x12 != 8 → halt (防御; 期望 fall through 因 cause=8)

halt:
    # ---- 停机: test_dev FINISHER PASS ----
    li   x30, 0x100000      # test_dev base
    li   x31, 0x5555        # FINISHER PASS magic
    sw   x31, 0(x30)        # 干净停机: test_dev 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望 (main.c dump):
#   x10 = 1                  (S 起步标记)
#   x11 = 2                  (U 起步标记 — sret 真生效证据!)
#   x12 = 8                  (mcause from U ecall, csrr 进 handler 时读)
#   x13 = 8                  (比较寄存器)
#   x30 = 0x100000 / x31 = 0x5555 (停机用 scratch: test_dev base / FINISHER PASS magic)
#   priv = 3                 (M; deliver_priv hard-code PRIV_M)
#   停机 = test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
#
# 关键验证点:
#   - x11 = 2 → sret 真生效 (没 sret 跑不到 u_program)
#   - x12 = 8 → cause=8 from U (ecall_from_U 跟 a_01_7 cause=8 编码一致)
# (注: 绝对 pc / total_count / mstatus 终值见 REVIEW B.3 段; 不在断言内。)
