# a_01_5_c / 05_triple_fault_halt
# 验证项目内部停机协议: csrw mtvec,x0 + ecall 触发 triple fault 链 → in_trap=3 dispatcher 退出。
# 2 条指令 = 8 字节 (32-bit only, .option norvc)。
#
# 这是项目当前的"标准停机机制" (dummy.txt §1 + 候选 A): hart 被 reset 时对应 mtvec=0 的状态,
# guest 触发任何 trap 都会跳 0, fetch 0 失败, 又 trap, 直到 in_trap >= 3 dispatcher 退出。
# 未来 MMIO bus / sifive_test 起来后, 改 MMIO finisher 路径 (例如 sifive_test 0x100000 写
# finisher value), 但当前 a_01_5_c 用 triple fault 当停机协议。
#
# 期望:
#   x1 = x2 = x3 = 0       (没设)
#   pc = 0
#   in_trap = 3
#   mcause = 1, mtval = 0, mepc = 0, mtvec = 0
#     (字段保留第 2 次 trap = mmu fail at 0 的状态)
#   total_count = 1        (block 1 csrw mtvec=1; block 2 ecall + block 3/4 mmu fail 都被
#                            longjmp / continue 跳过 total_count +=)
#
# 控制流追:
#   block 1 (0x00): csrw mtvec, x0 (boundary)              count=1, total=1
#                    mtvec = 0
#   block 2 (0x04): ecall → trap longjmp                    in_trap 0→1, total 不变 = 1
#                    _cause=11, _epc=0x4, regs[0] = mtvec = 0
#   block 3 (0x0):  mmu fail → trap_set_state(1, 0)         in_trap 1→2
#                    _cause=1, _tval=0, _epc=0; regs[0] = mtvec = 0
#   block 4 (0x0):  mmu fail → trap (候选 A 早 return)       in_trap 2→3, 字段保留第 2 次
#   while (3<3) false → 退出

.section .text
.option norvc
.globl _start
_start:
    csrw mtvec, x0          # 0x80000000: mtvec = 0 (强制 reset 状态; 项目停机协议起点)
    ecall                   # 0x80000004: 触发 trap 链 → triple fault → 退出
