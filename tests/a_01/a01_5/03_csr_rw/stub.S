# a_01_5_c / 03_csr_rw
# 验证 csr.c 6 小 helper 真读写 (csrw 写 → csrr 读回相同值; WARL 截断对 mtvec/mepc 真生效)。
# 8 条指令 = 32 字节 (32-bit only, .option norvc)。
#
# csrw / csrr 是 pseudo:
#   csrw csr, rs1 = csrrw x0, csr, rs1  (op=RW, do_write=true)
#   csrr rd, csr  = csrrs rd, csr, x0   (op=RS, new_val=0, do_write=false → 只读不写)
#
# WARL 截断验证:
#   - mtvec write 0x42 (0b01000010) → mask 低 2 位 → 0x40 (Vectored 不实现, 强制 Direct)
#   - mepc write 0x42 → mask IALIGN_MASK (=1) → 0x42 (低 1 位本来就 0, 不变)
#   - mcause / mtval 无 WARL, 接受任意值
#
# 期望:
#   x1 = 0x40   (csrr mtvec 读回 csrw 写的; 验 mtvec WARL 真截断 0x42→0x40)
#   x2 = 0x42   (csrr mcause 读回; 验 mcause 无 WARL)
#   x3 = 0x42   (csrr mepc 读回; 验 mepc WARL 对偶数不影响)
#   ecall 末尾触发 trap → 跳 mtvec=0x40 → mmu fail → triple fault → 退出
#   in_trap=3, mcause=1, mtval=0x40, mepc=0x40, mtvec=0x40
#   pc=0x40, total_count=7
#
# dispatcher 行为:
#   block 1 (0x80000000): addi + csrw mtvec (boundary)        count=2, total += 2 = 2
#   block 2 (0x80000008): csrr mtvec (boundary)               count=1, total += 1 = 3
#   block 3 (0x8000000c): csrw mcause (boundary)              count=1, total += 1 = 4
#   block 4 (0x80000010): csrr mcause (boundary)              count=1, total += 1 = 5
#   block 5 (0x80000014): csrw mepc (boundary)                count=1, total += 1 = 6
#   block 6 (0x80000018): csrr mepc (boundary)                count=1, total += 1 = 7
#   block 7 (0x8000001c): ecall → trap (cause=11) longjmp     total 不变 = 7
#                         regs[0] = mtvec = 0x40
#   block 8 (0x40):       mmu fail → trap (cause=1, in_trap 1→2)
#   block 9 (0x40):       mmu fail → trap (in_trap 2→3, 候选 A 早 return)
#   while 退出, total_count = 7

.section .text
.option norvc
.globl _start
_start:
    addi x10, x0, 0x42      # 0x80000000: x10 = 0x42 (csr 写源)
    csrw mtvec, x10          # 0x80000004: mtvec = 0x42 → WARL → 0x40
    csrr x1, mtvec           # 0x80000008: x1 = mtvec = 0x40 (验真读写)
    csrw mcause, x10         # 0x8000000c: mcause = 0x42 (无 WARL)
    csrr x2, mcause          # 0x80000010: x2 = mcause = 0x42
    csrw mepc, x10           # 0x80000014: mepc = 0x42 → WARL IALIGN=1 → 0x42 (低 1 位 0)
    csrr x3, mepc            # 0x80000018: x3 = mepc = 0x42
    ecall                    # 0x8000001c: 触发 trap → 跳 mtvec=0x40 → mmu fail → triple
