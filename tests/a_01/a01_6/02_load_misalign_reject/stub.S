# a_01_6 / 02_load_misalign_reject
# 验证 load_helper 的 misalign 检查 (Spike 风格): lw 一个奇地址 → trap_raise(4 load misalign)。
# 跳 handler 后 csrr 验 mcause/mtval/mepc, 然后 ecall + triple fault 退出。
# 11 条指令 (la 算 2 条) = 44 字节 (32-bit only, .option norvc)。
#
# misalign 触发条件 (lsu.h load_helper Step 1):
#   (gva & (size - 1)) != 0
# lw size=4, mask=3; gva=0x80001001 → 0x80001001 & 3 = 1 != 0 → trap cause=4, tval=0x80001001。
#
# 期望 (handler 内 csrr 拿到 trap 当时的 csr 值):
#   x1 = 4          (mcause = load address misaligned, RV spec §3.1.16)
#   x2 = 0x80001001 (mtval = 触发 misalign 的 GVA)
#   x3 = 0x80000014 (mepc = lw 指令的 PC = RAM 起点 + 0x14)
#   pc = 0          (handler 末 ecall + triple fault 退出, 字段被覆盖)
#   in_trap = 3
#   mcause = 11     (字段保留第 2 次 trap = handler 末 ecall, cause 11 ECALL_FROM_M)
#                    注意: x1=4 是 handler 内 csrr 拿到的 trap 当时值, mcause 是 trap dump 时
#                    被 ecall 覆盖的值; 两者一个验"misalign 真触发", 一个是项目停机协议副作用。
#   mtval = 0       (ecall tval=0)
#   mepc = 0x80000028 (handler 末 ecall PC)
#   mtvec = 0
#   total_count = 7
#     (block 1 la(2)+csrw=3; block 2 lui+addi+lw → lw misalign longjmp, count++ 不到, total 不变;
#      block 3 csrr x1=1; block 4 csrr x2=1; block 5 csrr x3=1; block 6 csrw mtvec=1; total 累加 = 7;
#      block 7 ecall longjmp; mmu fail x2 → triple fault 退出)
#
# 字节地址 (la 是 pseudo, 展开 auipc+addi 占 8 字节):
#   _start: 0x00 la(8) → 0x00-0x07; 0x08 csrw mtvec; 0x0c lui; 0x10 addi; 0x14 lw (trap here)
#   handler 起点 = 0x18:
#       0x18 csrr x1 mcause; 0x1c csrr x2 mtval; 0x20 csrr x3 mepc; 0x24 csrw mtvec,x0; 0x28 ecall
#
# 控制流追 (带 0x80000000 RAM 起点偏移):
#   block 1 (RAM+0x00): la(2)+csrw mtvec(boundary)       count=3,  total=3
#                        mtvec = 0x80000018 (handler)
#   block 2 (RAM+0x0c): lui + addi + lw → trap longjmp    count++ 不到, total=3
#                        in_trap 0→1, _cause=4, _tval=0x80001001, _epc=RAM+0x14, regs[0]=mtvec=RAM+0x18
#   block 3 (RAM+0x18): csrr x1, mcause(boundary)         count=1,  total=4   x1=4
#   block 4 (RAM+0x1c): csrr x2, mtval(boundary)          count=1,  total=5   x2=0x80001001
#   block 5 (RAM+0x20): csrr x3, mepc(boundary)           count=1,  total=6   x3=0x80000014
#                        (handler 内 csrr 之间不连成一块, 跟 a_01_5_c 03_csr_rw 一致)
#   block 6 (RAM+0x24): csrw mtvec, x0(boundary)          count=1,  total=7   mtvec=0
#   block 7 (RAM+0x28): ecall → trap longjmp              count++ 不到, total=7
#                        in_trap 1→2, _cause=11, _tval=0, _epc=RAM+0x28, regs[0]=mtvec=0
#   block 8 (0x0):      mmu fail → trap                   in_trap 2→3, 候选 A
#   while 退出, total_count=7

.section .text
.option norvc
.globl _start
_start:
    la   x4,  handler       # 0x00-0x07: auipc + addi (la pseudo, 2 条 4 字节)
    csrw mtvec, x4          # 0x08:      mtvec = handler

    # ---- 触发 misalign load ----
    lui  x6,  0x80001       # 0x0c:      x6 = 0x80001000
    addi x6,  x6, 1         # 0x10:      x6 = 0x80001001 (奇地址, 不 4-byte 对齐)
    lw   x9,  0(x6)         # 0x14:      lw misalign → trap cause=4, tval=0x80001001
    # ↑ trap_raise_exception _Noreturn longjmp; 不到此处

handler:
    csrr x1,  mcause        # 0x18:  x1 = mcause = 4 (load address misaligned)
    csrr x2,  mtval         # 0x1c:  x2 = mtval = 0x80001001 (触发 ea)
    csrr x3,  mepc          # 0x20:  x3 = mepc = 0x80000014 (lw 的 PC)

    # ---- 停机 (项目内部 triple fault 协议) ----
    csrw mtvec, x0          # 0x24:  mtvec = 0
    ecall                   # 0x28:  trap → 跳 0 → mmu fail x2 → triple fault 退出
