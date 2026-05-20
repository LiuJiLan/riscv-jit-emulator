# a_02_2 / 05_sv32_bus_unmapped_reject
#
# ===== 测试目的 =====
# 验 SV32 经 mmu 翻译后落 bus 未注册 PA, mmio_*_helper 扫表未命中 → access fault
# (a_02_2/02 BARE 版的 SV32 版本)。
#
# ===== 思路 =====
# 跟 a_02_2/02 比, 多走一遍 walker; mmio_*_helper 行为应一致 (cause 5/7, mtval=gva)。
#
# 触发 case (2 个, mret +4 chain):
#   case 0: sw → vaddr 0x03000000 (经 walker → pa 0x03000000) → 未命中 → cause 7
#   case 1: lw → vaddr 0x03000000 → 未命中 → cause 5

.equ CSR_TOHOST, 0x800
.equ CSR_PRIVRD, 0xCC0

.section .text
.option norvc
.globl _start
_start:
    lui  sp, 0x80300

    la   x4, m_handler
    csrw mtvec, x4

    call setup_pt

    lui  x6, 0x80080
    addi x6, x6, 0x100              # satp = 0x80080100
    csrw satp, x6
    sfence.vma

    la   x7, s_target
    csrw mepc, x7

    addi x4, x0, 1
    slli x4, x4, 11
    csrw mstatus, x4

    li   s0, 0                      # M-mode 设 s0=0, mret 后 priv 切 S, regs 不变

    mret                            # M→S

s_target:
    # === case 0: sw → 0x03000000 (bus 未命中) ===
    lui  x4, 0x03000                # vaddr 0x03000000
    li   x5, 0xCAFEBABE
    sw   x5, 0(x4)                  # store_access_fault (cause 7, mtval=gva)

    # === case 1: lw → 0x03000000 (bus 未命中) ===
    lui  x4, 0x03000
    lw   x6, 0(x4)                  # load_access_fault (cause 5, mtval=gva)

    # case 1 后 handler 内 detect s0==2 → 不 mret, M-mode 内 halt; 不需要 S phase
    # 后续 csrw mtvec 0 + ecall (S 不能 csrw mtvec; 跟 a_02_2/02 BARE 不同, 05 SV32
    # 切了 S 后 halt 必须在 handler 内 M-mode 完成)。
    # 防御: 真跑到这里 = handler chain 没正确 detect halt, 走 ebreak 留 debug 信号。
    ebreak

# ----------------------------------------------------------------------------
# m_handler — switch on s0 写 mtval (+ case 1 写 cause) 到 a 寄存器, mret +4
# 跟 a_02_2/02 同形态
# ----------------------------------------------------------------------------
m_handler:
    csrr x7,  mcause
    csrr x28, mtval                 # x28 = t3 (临时, 不进 a 寄存器)

    li   x29, 0
    beq  s0, x29, case0
    # default case 1
case1:
    mv   a1, x7                     # a1 = case 1 mcause
    mv   a2, x28                    # a2 = case 1 mtval
    j    epilogue
case0:
    mv   a0, x28                    # a0 = case 0 mtval (cause 7 已知, 不存)
    j    epilogue

epilogue:
    addi s0, s0, 1
    li   x31, 2                     # 2 case 完成阈值
    beq  s0, x31, m_halt            # case chain 完 → M-mode 内 halt (避免 S 跑 csrw mtvec)
    csrr x30, mepc
    addi x30, x30, 4                # mepc += 4 (sw / lw 都是 4 byte; .option norvc)
    csrw mepc, x30
    mret

# ----------------------------------------------------------------------------
# m_halt — case chain 完成后从 handler 直接走 halt 路径 (still in M-mode); 不 mret
# 回 S phase (因 S 不能 csrw mtvec)。模式: csrw mtvec, 0 + ecall_from_M → cause 11 →
# pc=mtvec=0 → fetch fail → in_trap 链路到 3 → triple fault halt。
# ----------------------------------------------------------------------------
m_halt:
    csrw mtvec, x0
    ecall                           # ecall_from_M (cause 11) → triple fault path

# ===== 期望结果 =====
# 期望 main.c dump:
#   a0 (x10) = 0x03000000   case 0 mtval
#   a1 (x11) = 5            case 1 mcause
#   a2 (x12) = 0x03000000   case 1 mtval
#   s0 (x8)  = 2            (2 case 都进 handler)
#   in_trap=3 mcause=11 priv=3
#
# halt: 2 case 完后 csrw mtvec, x0 + ecall → 下轮 fetch pc=0 → triple fault。
# main dump 的 mcause/mtval 是 1/0 (最后 root cause, 不是 7/5; 7/5 走 a 寄存器)。
