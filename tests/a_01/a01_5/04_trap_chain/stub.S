# a_01_5_c / 04_trap_chain
# 验证完整的 trap → handler → mret → 主程序继续 流程:
#   - ecall 触发 trap, _epc 写 ecall PC; handler 内手动 csrw mepc, mepc+4 让 mret 跳过 ecall
#     (RV spec ECALL: mret 默认跳 mepc, software 必须自己 +4 跳过 ecall)
#   - mret 真复位 in_trap=0 + 跳 xepc; mret 后主程序 addi x2 真跑 (= mret 路径生效证据)
#   - 主程序末尾 csrw mtvec,x0 + ecall 强制 triple fault 退出 (= 项目内部停机协议)
# 13 条指令 = 52 字节 (32-bit only, .option norvc)。
#
# 期望:
#   x1 = 1     (主程序起点 addi)
#   x2 = 99    (mret 回来后跑; 这是 mret 真生效的证据 — 没 mret 就跑不到这里)
#   x3 = 7     (handler 跑过)
#   pc = 0     (主程序末尾 mtvec=0, ecall trap → 跳 0 → mmu fail x2 → 候选 A 早 return,
#                regs[0] 停在 0)
#   in_trap = 3
#   mcause = 1, mtval = 0, mepc = 0, mtvec = 0
#     (字段保留第 2 次 trap = mmu fail at 0 的状态; 第 3 次候选 A 早 return 不动)
#   total_count = 10
#     (block 1 la+csrw=3 + block 3 handler addi+csrr=2 + block 4 addi+csrw=2 +
#      block 5 mret=1 + block 6 主程序 addi x2 + csrw mtvec=2; 共 10)
#
# 控制流追:
#   block 1 (0x00):    la(2) + csrw mtvec(boundary)         count=3,  total=3
#                       mtvec = 0x80000020 (handler)
#   block 2 (0x0c):    addi x1 + ecall → trap longjmp        count=ignored, total=3
#                       in_trap 0→1, _epc=0x10 (1st ecall PC), regs[0]=mtvec=0x20
#   block 3 (handler 0x20): addi x3 + csrr x5,mepc(boundary) count=2,  total=5
#                       x3=7, x5 = mepc = 0x10
#   block 4 (0x28):    addi x5,4 + csrw mepc,x5(boundary)    count=2,  total=7
#                       x5 = 0x14, mepc write = 0x14 (mask IALIGN=1 不影响)
#   block 5 (0x30):    mret(boundary)                        count=1,  total=8
#                       in_trap=0 (复位), regs[0] = xepc = 0x14
#   block 6 (0x14):    addi x2 + csrw mtvec,x0(boundary)     count=2,  total=10
#                       x2=99, mtvec=0
#   block 7 (0x1c):    ecall → trap longjmp                  count=ignored, total=10
#                       in_trap 0→1 (mret 已复位过), _epc=0x1c, regs[0]=mtvec=0
#   block 8 (0x0):     mmu fail → trap                       in_trap 1→2
#   block 9 (0x0):     mmu fail → trap (候选 A 早 return)     in_trap 2→3
#   while 退出

.section .text
.option norvc
.globl _start
_start:
    la   x4, handler        # 0x00-0x07: auipc + addi pseudo
    csrw mtvec, x4          # 0x08:      mtvec = handler 地址
    addi x1, x0, 1          # 0x0c:      x1 = 1 (跑过标记)
    ecall                   # 0x10:      1st trap → handler
    addi x2, x0, 99         # 0x14:      mret 回来后跑 (= mret 真生效的证据)
    csrw mtvec, x0          # 0x18:      mtvec = 0 (准备 triple fault)
    ecall                   # 0x1c:      2nd trap → 跳 0 → mmu fail x2 → triple fault

handler:
    addi x3, x0, 7          # 0x20:      x3 = 7 (handler 跑过标记)
    csrr x5, mepc           # 0x24:      x5 = mepc (= 1st ecall PC = 0x10)
    addi x5, x5, 4          # 0x28:      x5 = 0x14 (跳过 ecall, 指向 addi x2)
    csrw mepc, x5           # 0x2c:      mepc = 0x14
    mret                    # 0x30:      跳 mepc=0x14, in_trap=0; 回主程序 addi x2
