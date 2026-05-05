# a_01_7 / 01_trap_priv_save_restore
# 验证 trap_set_state 真切 priv + 写 mstatus.MPP/MPIE/MIE; OP_MRET 真切 priv (从 MPP 恢复)。
# M 自我 trap chain (M → ecall → handler → mret → M); 当前 deliver_priv hard-code M, 切前后
# priv 都是 M, 但 mstatus 字段保存/恢复的代码路径完整跑通。
# 13 条指令 (la 算 2 条) = 52 字节 (32-bit only, .option norvc)。
#
# 字节地址布局 (la 是 pseudo, 展开 auipc+addi 占 8 字节):
#   _start: 0x00 la(8); 0x08 csrw mtvec; 0x0c addi; 0x10 csrw mstatus; 0x14 ecall(1st);
#           0x18 csrr x2 mstatus; 0x1c csrw mtvec,x0; 0x20 ecall(2nd, triple fault)
#   handler 起点 = 0x24:
#       0x24 csrr x1 mstatus; 0x28 csrr x5 mepc; 0x2c addi x5 +4; 0x30 csrw mepc x5; 0x34 mret
#
# 期望:
#   x1 = 0x1880 = 6272      (handler 内 csrr mstatus: MPP=M=3<<11=0x1800 + MPIE=1<<7=0x80)
#   x2 = 0x88   = 136       (mret 后主程序 csrr mstatus: MIE=1 (恢复 from MPIE) + MPIE=1 (RV spec) +
#                             MPP=PRIV_U=0 (RV spec least-priv reset))
#   x3 = 0                  (没设)
#   pc = 0
#   in_trap = 3
#   mcause = 1, mtval = 0, mepc = 0, mtvec = 0
#     (字段保留第 2 次 trap = 主程序末 ecall 后 mmu fail at 0; cause=1 access fault on fetch
#      —— 跟 a_01_5_c 05_triple_fault_halt 同形态)
#   priv = 3                (M; mret 切回 M, 末 ecall 后 trap 切到 M=deliver_priv 不变)
#   mstatus = 0x1800        (字段保留第 2 次 trap 设的 MPP=M=3<<11=0x1800; MPIE=MIE=0 因为
#                             末 ecall 时 mstatus.MIE 已经 0)
#   total_count = 12
#     (block 1: la(2)+csrw mtvec=3; block 2: addi+csrw mstatus=2 → total=5; ecall longjmp 不计;
#      block 3-7: 5 条 csr/mret 各独立 boundary, 加 1 个 addi+csrw mepc 块 = 1+1+2+1 = 5 → total=10;
#      mret 后 csrr x2(boundary)=1 → 11; csrw mtvec(boundary)=1 → 12; ecall longjmp; mmu fail x2
#      不计)
#
# 控制流追 (带 0x80000000 RAM 起点偏移):
#   block 1 (RAM+0x00): la(2)+csrw mtvec(boundary)        count=3,  total=3
#                        mtvec = 0x80000024 (handler)
#   block 2 (RAM+0x0c): addi+csrw mstatus(boundary)        count=2,  total=5
#                        mstatus = 0x8 (MIE=1, MPP=0, MPIE=0)
#   block 3 (RAM+0x14): ecall(1st) → trap longjmp           count++ 不到, total=5
#                        in_trap 0→1, xcause=11, xtval=0, xepc=RAM+0x14, regs[0]=mtvec=RAM+0x24
#                        mstatus 切: MPP=M=3, MPIE=MIE=1, MIE=0 → 0x1880
#                        priv 不变 (deliver_priv=M)
#   block 4 (RAM+0x24): csrr x1, mstatus(boundary)          count=1,  total=6   x1 = 0x1880
#   block 5 (RAM+0x28): csrr x5, mepc(boundary)             count=1,  total=7   x5 = RAM+0x14
#   block 6 (RAM+0x2c): addi x5+4 + csrw mepc(boundary)     count=2,  total=9
#                        x5 = RAM+0x18; mepc = RAM+0x18 (mask IALIGN 偶数不变)
#   block 7 (RAM+0x34): mret(boundary)                      count=1,  total=10
#                        priv = MPP = M; MIE=MPIE=1; MPIE=1; MPP=U=0; mstatus = 0x88
#                        regs[0] = xepc = RAM+0x18; in_trap = 0
#   block 8 (RAM+0x18): csrr x2, mstatus(boundary)          count=1,  total=11  x2 = 0x88
#   block 9 (RAM+0x1c): csrw mtvec, x0(boundary)            count=1,  total=12  mtvec=0
#   block 10 (RAM+0x20): ecall(2nd) → trap longjmp           count++ 不到, total=12
#                         in_trap 0→1 (mret 已复位), xcause=11, xepc=RAM+0x20, regs[0]=mtvec=0
#                         mstatus 切: MPP=M, MPIE=MIE=1, MIE=0 → 0x1880
#   block 11 (0x0):     mmu fail → trap                    in_trap 1→2
#                        cause=1, tval=0, epc=0; mstatus 切: MPP=M, MPIE=MIE=0, MIE=0 → 0x1800
#   block 12 (0x0):     mmu fail → trap (候选 A 早 return) in_trap 2→3
#                        字段保留第 2 次状态: cause=1, mstatus=0x1800
#   while 退出, total_count=12

.section .text
.option norvc
.globl _start
_start:
    la   x4,  handler       # 0x00-0x07: auipc + addi (la pseudo, 2 条 4 字节)
    csrw mtvec, x4          # 0x08:      mtvec = handler

    # ---- 设 mstatus.MIE = 1 (验 trap 时 MPIE 真保存了 MIE 的旧值) ----
    addi x10, x0, 8         # 0x0c:      x10 = 0x8 = MSTATUS_MIE 位
    csrw mstatus, x10       # 0x10:      mstatus = 0x8

    # ---- 触发 1st trap (ecall from M, cause=11) ----
    ecall                   # 0x14:      trap → handler

    # ---- mret 回来后的落点 (handler 末 csrw mepc x5 让 mepc=0x18 跳过原 ecall) ----
    csrr x2,  mstatus       # 0x18:      x2 = mstatus 验 mret 后恢复 (= 0x88)

    # ---- 停机 (项目内部 triple fault 协议) ----
    csrw mtvec, x0          # 0x1c:      mtvec = 0
    ecall                   # 0x20:      2nd trap → 跳 0 → mmu fail x2 → triple fault 退出

handler:
    csrr x1,  mstatus       # 0x24:      x1 = mstatus 验 trap 后保存 (= 0x1880)
    csrr x5,  mepc          # 0x28:      x5 = mepc = 0x80000014 (1st ecall PC)
    addi x5,  x5, 4         # 0x2c:      x5 = 0x80000018 (跳过 ecall, 指向下一条)
    csrw mepc, x5           # 0x30:      mepc = 0x80000018
    mret                    # 0x34:      切 priv from MPP = M; pc = mepc = 0x80000018
