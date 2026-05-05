# a_01_7 / 04_u_mode_ecall_chain
# 验证完整的 syscall-like 端到端路径: M 设 handler → mret 切 U → U 跑程序 → ecall(U) cause=8 →
# M handler 处理 → mret 回 U → U 继续跑 → 再次 ecall(U) → handler 决定停机 (triple fault)。
#
# 覆盖范围:
#   1. ecall from U → cause=8 (验 8 + PRIV_U=0 公式; 之前 fixture 都只验 cause=11 from M)
#   2. mret 真切回 U (cross-priv mret, 跟 a_01_5_c 04_trap_chain M 自我 mret 风格不同)
#   3. U 多次 ecall 都走同 m_handler (handler 复用)
#   4. U mode GPR (x1/x2) 跨 trap/mret 保留 (precise trap 语义; trap_set_state / mret 不动 GPR)
#   5. handler GPR (x3) 跨多次 trap/mret 调用累计 (host 路径不动 guest GPR)
#
# 21 条指令 (la×2 算 4 条) = 84 字节 (32-bit only, .option norvc)。
#
# 字节地址布局:
#   _start:      0x00 la x4(8); 0x08 csrw mtvec; 0x0c csrw mstatus,x0;
#                0x10 la x6(8); 0x18 csrw mepc;  0x1c mret
#   u_program (0x20):
#                0x20 addi x1,x0,1; 0x24 ecall(1st);
#                0x28 addi x2,x0,2; 0x2c ecall(2nd)
#   m_handler (0x30):
#                0x30 addi x3,x3,+1; 0x34 addi x13,x0,2; 0x38 beq x3,x13,halt;
#                0x3c csrr x5,mepc; 0x40 addi x5,x5,4; 0x44 csrw mepc,x5; 0x48 mret
#   halt (0x4c):
#                0x4c csrw mtvec,x0; 0x50 ecall (triple fault)
#
# 期望 (main.c dump):
#   x1 = 1                  (U 1st ecall 前设)
#   x2 = 2                  (mret 真生效证据 — 没 mret 跑不到这条)
#   x3 = 2                  (handler 累计计数器, ecall 进 handler 2 次)
#   pc = 0
#   in_trap = 3
#   mcause = 11             (字段保留第 2 次 trap = handler 末 ecall from M)
#   mtval = 0
#   mepc = 0x80000050       (handler 末 ecall 的 PC)
#   mtvec = 0
#   priv = 3                (handler 在 M 模式; 第 2 次 trap 切 M, 候选 A 早 return 不动 priv)
#   mstatus = 0x1800        (handler 末 ecall trap 时 MPP=M=3<<11=0x1800; MPIE=MIE=0)
#   total_count = 19
#
# 控制流追 (带 0x80000000 RAM 起点偏移):
#   block 1 (RAM+0x00): la(2)+csrw mtvec(boundary)         count=3,  total=3
#                        mtvec = RAM+0x30
#   block 2 (RAM+0x0c): csrw mstatus,x0(boundary)          count=1,  total=4
#                        mstatus = 0 (MPP=U=0, MPIE=0, MIE=0)
#   block 3 (RAM+0x10): la(2)+csrw mepc(boundary)          count=3,  total=7
#                        mepc = RAM+0x20 (u_program)
#   block 4 (RAM+0x1c): mret(boundary)                     count=1,  total=8
#                        priv=MPP=U=0; MIE=MPIE=0; MPIE=1; MPP=U=0; mstatus = 0x80
#                        regs[0] = mepc = RAM+0x20
#   block 5 (RAM+0x20, U): addi x1=1 + ecall → trap longjmp count++ 不到, total=8
#                        addi x1=1 计入 count 但 longjmp 跳过 *count_out 写入
#                        in_trap 0→1, xcause=8 (ecall from U), xtval=0, xepc=RAM+0x24
#                        mstatus 切: MPP=U=0 (caller); MPIE=MIE=0 (current MIE=0); MIE=0 → 0
#                        priv=M (deliver_priv); regs[0]=mtvec=RAM+0x30
#   block 6 (RAM+0x30, M): addi x3 + addi x13 + beq(boundary) count=3, total=11
#                        x3=1, x13=2; beq 1 != 2 not-taken → fall-through (regs[0]=RAM+0x3c)
#   block 7 (RAM+0x3c): csrr x5,mepc(boundary)             count=1,  total=12   x5=RAM+0x24
#   block 8 (RAM+0x40): addi x5+4 + csrw mepc(boundary)    count=2,  total=14
#                        x5=RAM+0x28; mepc=RAM+0x28
#   block 9 (RAM+0x48): mret(boundary)                     count=1,  total=15
#                        priv=MPP=U; MIE=MPIE=0; MPIE=1; MPP=U=0; mstatus=0x80
#                        regs[0] = mepc = RAM+0x28
#   block 10 (RAM+0x28, U): addi x2=2 + ecall → trap longjmp count++ 不到, total=15
#                        in_trap 0→1 (mret 已复位), xcause=8, xtval=0, xepc=RAM+0x2c
#                        mstatus 切: MPP=U=0, MPIE=MIE=0, MIE=0 → 0
#                        priv=M, regs[0]=mtvec=RAM+0x30
#   block 11 (RAM+0x30): addi x3 + addi x13 + beq(boundary) count=3, total=18
#                        x3=2, x13=2; beq 2 == 2 taken → regs[0]=halt=RAM+0x4c
#   block 12 (RAM+0x4c): csrw mtvec,x0(boundary)           count=1,  total=19   mtvec=0
#   block 13 (RAM+0x50): ecall → trap longjmp              count++ 不到, total=19
#                        in_trap 1→2, xcause=11 (ecall from M), xtval=0, xepc=RAM+0x50
#                        mstatus 切: MPP=M=3, MPIE=MIE=0, MIE=0 → 0x1800
#                        regs[0] = mtvec = 0
#   block 14 (0x0):     mmu fail → trap_set_state(1, 0)    in_trap 2→3 候选 A
#                        字段保留第 2 次 = ecall (cause=11, tval=0, epc=RAM+0x50, mstatus=0x1800)
#   while 退出, total_count=19

.section .text
.option norvc
.globl _start
_start:
    # ---- 设 trap handler ----
    la   x4, m_handler      # 0x00-0x07: la pseudo (8 字节)
    csrw mtvec, x4          # 0x08:      mtvec = m_handler

    # ---- 设 mstatus.MPP = PRIV_U = 0 (mret 切 U 的关键) ----
    csrw mstatus, x0        # 0x0c:      mstatus = 0 (MPP=U, MPIE=0, MIE=0)

    # ---- 设 mret 目标 = u_program ----
    la   x6, u_program      # 0x10-0x17: la pseudo (8 字节)
    csrw mepc, x6           # 0x18:      mepc = u_program

    # ---- mret → priv=U, pc=u_program ----
    mret                    # 0x1c

u_program:
    # ---- 1st ecall from U ----
    addi x1, x0, 1          # 0x20:      x1 = 1 (syscall arg-like 标识)
    ecall                   # 0x24:      cause=8 (ecall from U) → m_handler

    # ---- mret 回来后跑这条 (handler 把 mepc +4 跳过原 ecall) ----
    addi x2, x0, 2          # 0x28:      x2 = 2 (验 mret 真生效)

    # ---- 2nd ecall from U ----
    ecall                   # 0x2c:      cause=8 → m_handler (此次 handler 不 mret, 直接停机)

    # ---- 不到此处 ----

m_handler:
    # x3 是 handler 累计计数器 (host 路径不动 guest GPR, 跨 trap/mret 累计)
    addi x3, x3, 1          # 0x30:      x3++ (1, 然后 2)
    addi x13, x0, 2         # 0x34:      x13 = 2 (停机阈值)
    beq  x3, x13, halt      # 0x38:      x3==2 → halt (第 2 次到); offset=0x14

    # ---- 第 1 次 (x3=1 != 2) 走这里, mret 回 U 继续 ----
    csrr x5, mepc           # 0x3c:      x5 = mepc = RAM + (1st ecall PC = 0x24)
    addi x5, x5, 4          # 0x40:      x5 += 4 (跳过 ecall, 指向 next inst = addi x2)
    csrw mepc, x5           # 0x44:      mepc = next inst
    mret                    # 0x48:      切 priv=MPP=U; pc=mepc

halt:
    # ---- 第 2 次到 (x3==2): handler 不 mret, 走 triple fault 退出协议 ----
    csrw mtvec, x0          # 0x4c:      mtvec = 0
    ecall                   # 0x50:      ecall from M (cause=11) → 跳 0 → mmu fail → triple fault
