# a_01_7 / 02_u_mode_csr_priv_reject
# 验证 csr_op 入口 priv 检查激活: U 模式写 M-only csr (mtvec 0x305, bits 9:8 = 11 = M-required)
# → trap cause=2 (illegal instruction)。
# 切 U 路径: M 模式设 mstatus.MPP=PRIV_U=0 + csrw mepc, u_target + mret → priv 切 U + pc=u_target。
# 14 条指令 (la×2 算 4 条) = 56 字节 (32-bit only, .option norvc)。
#
# 字节地址布局:
#   _start: 0x00 la x4, handler(8); 0x08 csrw mtvec; 0x0c csrw mstatus,x0;
#           0x10 la x6, u_target(8); 0x18 csrw mepc; 0x1c mret
#   u_target = 0x20:
#       0x20 csrw mtvec, x4 (U 模式触发 trap)
#   handler = 0x24:
#       0x24 csrr x1 mcause; 0x28 csrr x2 mtval; 0x2c csrr x3 mepc;
#       0x30 csrw mtvec,x0; 0x34 ecall (triple fault)
#
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
#   x3 = 0x80000020 = 2147483680 (mepc = u_target 处 csrw mtvec 的 PC)
#   pc = 0
#   in_trap = 3
#   mcause = 11     (字段保留第 2 次 trap = handler 末 ecall)
#   mtval = 0
#   mepc = 0x80000034 (handler 末 ecall PC)
#   mtvec = 0
#   priv = 3        (handler 在 M 模式跑; trap 切 priv 已切 M)
#   mstatus = 0x1800 (handler 末 ecall trap 时 MPP=M=3<<11=0x1800; MPIE=MIE=0 因为
#                      MIE 在前一次 trap 时已 0)
#
# 控制流追:
#   block 1 (RAM+0x00): la(2)+csrw mtvec(boundary)        count=3,  total=3
#                        mtvec = RAM+0x24 (handler)
#   block 2 (RAM+0x0c): csrw mstatus,x0(boundary)         count=1,  total=4
#                        mstatus = 0 (MPP=U=0; MPIE=0; MIE=0)
#   block 3 (RAM+0x10): la(2)+csrw mepc(boundary)         count=3,  total=7
#                        x6 = RAM+0x20 (u_target); mepc = RAM+0x20
#   block 4 (RAM+0x1c): mret(boundary)                    count=1,  total=8
#                        priv = MPP = U; MIE=MPIE=0; MPIE=1; MPP=U=0; mstatus = 0x80
#                        regs[0] = xepc = RAM+0x20
#   block 5 (RAM+0x20, priv=U): csrw mtvec,x4 → priv 检查 fail → trap longjmp
#                        count++ 不到, total=8
#                        in_trap 0→1, xcause=2, xtval=0x30521073, xepc=RAM+0x20
#                        mstatus 切: MPP=U=0 (caller priv), MPIE=MIE=0, MIE=0 → mstatus_lo=0
#                        priv = M (deliver_priv); regs[0] = mtvec = RAM+0x24
#   block 6 (RAM+0x24): csrr x1, mcause(boundary)         count=1,  total=9   x1=2
#   block 7 (RAM+0x28): csrr x2, mtval(boundary)          count=1,  total=10  x2=0x30521073
#   block 8 (RAM+0x2c): csrr x3, mepc(boundary)           count=1,  total=11  x3=RAM+0x20
#   block 9 (RAM+0x30): csrw mtvec,x0(boundary)           count=1,  total=12  mtvec=0
#   block 10 (RAM+0x34): ecall → trap longjmp              count++ 不到, total=12
#                         in_trap 1→2, xcause=11, xepc=RAM+0x34
#                         mstatus 切: MPP=M=3, MPIE=MIE=0, MIE=0 → 0x1800
#   block 11 (0x0):     mmu fail → trap                   in_trap 2→3 候选 A
#                        字段保留第 2 次: cause=11, mstatus=0x1800
#   while 退出, total_count=12

.section .text
.option norvc
.globl _start
_start:
    la   x4,  handler       # 0x00-0x07: la pseudo (8 字节)
    csrw mtvec, x4          # 0x08:      mtvec = handler

    # ---- 设 mstatus.MPP = PRIV_U = 0 (mret 切 U 的关键) ----
    # 显式 csrw mstatus, x0 把所有 mstatus 字段清零 (cpu_create memset 后已经是 0, 但显式更
    # 清晰; 也复用了 a_01 fixture 启动协议风格)
    csrw mstatus, x0        # 0x0c:      mstatus = 0 (MPP=U=0, MPIE=0, MIE=0)

    # ---- 设 mepc = u_target 地址 (mret 跳的目标) ----
    la   x6,  u_target      # 0x10-0x17: la pseudo (8 字节)
    csrw mepc, x6           # 0x18:      mepc = u_target

    # ---- mret 切 priv → U; pc → u_target ----
    mret                    # 0x1c:      priv = MPP = U; pc = mepc = u_target

u_target:
    # ---- priv = U; 试图写 M-only csr → trap cause=2 ----
    csrw mtvec, x4          # 0x20:      U 模式不能写 mtvec (priv 不足) → trap cause=2
    # ↑ csr_op 入口判 priv 检查 → trap_raise_exception(2, raw_inst); 不到此处

handler:
    csrr x1,  mcause        # 0x24:      x1 = mcause = 2 (illegal instruction)
    csrr x2,  mtval         # 0x28:      x2 = mtval = 0x30521073 (raw_inst of csrw mtvec, x4)
    csrr x3,  mepc          # 0x2c:      x3 = mepc = RAM+0x20 (u_target 处 csrw 的 PC)

    # ---- 停机 ----
    csrw mtvec, x0          # 0x30:      mtvec = 0
    ecall                   # 0x34:      trap → 跳 0 → mmu fail x2 → triple fault 退出
