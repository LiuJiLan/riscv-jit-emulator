# a_02_2 / 07_sv32_leaf_pt_in_mmio
#
# ===== 测试目的 =====
# 验 SV32 walker level=0 PT 落 CLINT (MMIO) 区时报 access fault (cause 5; load 路径)
# — a_01_8/08 (PT 落完全 RAM 外 0xC0000000) 的 MMIO 版本。
#
# ===== 思路 =====
# 验证 mmu_walk 内 IS_GPA_RAM(pte0_pa) 检查对 "PT 落 MMIO" 跟 "PT 落完全不存在" 行为
# 一致 (都拒 → access fault); P3 后 walker 不应该把 MMIO 上的 PT 当合法 PT 走 mmio
# dispatch 读出来当 PTE 用 (那会让 PT walker 走 device 端 read 拿到的"假 PTE")。
#
# 触发路径:
#   walker walk vaddr 0x90000000:
#     pte1 = 0x00800001 (pointer → CLINT 0x02000000)
#     pte0_pa = 0x02000000 (!IS_GPA_RAM → fault)
#     fault_cause = CAUSE_LOAD_ACCESS_FAULT (5)
#
# 控制流:
#   1. M setup: sp + mtvec; call setup_pt; satp + sfence; mret 切 S
#   2. S phase:
#      lui  x4, 0x90000             # vaddr 0x90000000
#      lw   x10, 0(x4)              # walker level=0 PT 检查失败 → trap 5 (longjmp)
#      ecall                        # 防御
#   3. M handler: csrr mcause/mepc/mtval → x12/x13/x14; halt

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

    mret                            # M→S

s_target:
    lui  x4, 0x90000                # vaddr 0x90000000 (远段; PT level=0 落 CLINT)
    lw   x10, 0(x4)                 # walker → access fault → trap 5 (longjmp)
    ecall                           # 防御

m_handler:
    csrr x12, mcause                # x12 = 5
    csrr x13, mepc
    csrr x14, mtval                 # x14 = 0x90000000

halt:
    csrw mtvec, x0
    ecall                           # cause=11 from M → triple fault

# ===== 期望结果 =====
# 期望 main.c dump:
#   x10 = 0                        (lw 失败没写)
#   x12 = 5                        (LOAD_ACCESS_FAULT)
#   x13 = 失败 lw 的 PC            (mepc; 看反汇编, 在 s_target 内)
#   x14 = 0x90000000               (mtval = 失败 lw 的 GVA)
#   in_trap=3 mcause=11 priv=3
