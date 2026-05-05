# a_01_7 / 03_csr_ro_write_reject
# 验证 csr_op 入口 RO 写检查激活: M 模式写 RO csr (mhartid 0xF14, bits 11:10 = 11 = RO)
# → trap cause=2 (illegal instruction); 即使 priv=M 够 (mhartid 是 unprivileged csr, bits 9:8=00),
# RO 写仍然 block。
# 9 条指令 (la 算 2 条) = 36 字节 (32-bit only, .option norvc)。
#
# 字节地址布局:
#   _start: 0x00 la(8); 0x08 csrw mtvec; 0x0c csrw mhartid (trap)
#   handler = 0x10:
#       0x10 csrr x1 mcause; 0x14 csrr x2 mtval; 0x18 csrr x3 mepc;
#       0x1c csrw mtvec,x0; 0x20 ecall (triple fault)
#
# 期望:
#   x1 = 2          (mcause = illegal instruction)
#   x2 = 0xF1451073 = 4047835251
#                    (mtval = raw_inst of `csrw mhartid, x10`; 编码:
#                     csr=0xF14 << 20 = 0xF1400000
#                     rs1=10 << 15 = 0x00050000
#                     funct3=001 << 12 = 0x00001000
#                     rd=0 << 7  = 0
#                     opcode=0x73
#                     总 = 0xF1451073)
#   x3 = 0x8000000c = 2147483660 (mepc = csrw mhartid 的 PC)
#   pc = 0
#   in_trap = 3
#   mcause = 11     (字段保留第 2 次 trap = handler 末 ecall)
#   mtval = 0
#   mepc = 0x80000020 (handler 末 ecall PC)
#   mtvec = 0
#   priv = 3        (handler 在 M 模式跑)
#   mstatus = 0x1800
#
# 控制流追:
#   block 1 (RAM+0x00): la(2)+csrw mtvec(boundary)        count=3,  total=3
#                        mtvec = RAM+0x10 (handler)
#   block 2 (RAM+0x0c): csrw mhartid → RO 检查 fail → trap longjmp
#                        count++ 不到, total=3
#                        in_trap 0→1, xcause=2, xtval=0xF1451073, xepc=RAM+0x0c
#                        mstatus 切: MPP=M=3, MPIE=MIE=0, MIE=0 → 0x1800
#                        regs[0] = mtvec = RAM+0x10
#   block 3 (RAM+0x10): csrr x1, mcause(boundary)         count=1,  total=4   x1=2
#   block 4 (RAM+0x14): csrr x2, mtval(boundary)          count=1,  total=5   x2=0xF1451073
#   block 5 (RAM+0x18): csrr x3, mepc(boundary)           count=1,  total=6   x3=RAM+0x0c
#   block 6 (RAM+0x1c): csrw mtvec,x0(boundary)           count=1,  total=7   mtvec=0
#   block 7 (RAM+0x20): ecall → trap longjmp              count++ 不到, total=7
#                        in_trap 1→2, xcause=11, xepc=RAM+0x20
#                        mstatus: MPP=M, MPIE=MIE=0 → 0x1800 (不变)
#   block 8 (0x0):      mmu fail → trap                   in_trap 2→3 候选 A
#                        字段保留第 2 次: cause=11, mstatus=0x1800
#   while 退出, total_count=7

.section .text
.option norvc
.globl _start
_start:
    la   x4,  handler       # 0x00-0x07: la pseudo (8 字节)
    csrw mtvec, x4          # 0x08:      mtvec = handler

    # ---- M 模式写 RO csr mhartid (bits 11:10 = 11 = RO) → trap cause=2 ----
    # x10 当前 = 0 (memset), 但 csrrw 永远算"写" (CSR_OP_RW 不看 new_val); 即使 x10=0 也 trap。
    csrw mhartid, x10       # 0x0c:      RO 检查 fail → trap_raise(2, raw_inst); 不到下一条

handler:
    csrr x1,  mcause        # 0x10:      x1 = mcause = 2 (illegal instruction)
    csrr x2,  mtval         # 0x14:      x2 = mtval = 0xF1451073 (raw_inst of csrw mhartid)
    csrr x3,  mepc          # 0x18:      x3 = mepc = RAM+0x0c

    # ---- 停机 ----
    csrw mtvec, x0          # 0x1c:      mtvec = 0
    ecall                   # 0x20:      trap → 跳 0 → mmu fail x2 → triple fault 退出
