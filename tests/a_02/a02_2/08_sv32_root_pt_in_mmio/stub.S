# a_02_2 / 08_sv32_root_pt_in_mmio
# 验 SV32 root PT (satp.PPN 指 CLINT 区) 时报 access fault (cause 1 INST_ACCESS_FAULT;
# fetch 路径) — a_01_8/09 (root PT 落完全 RAM 外 0xC0000000) 的 MMIO 版本。
#
# 跟 fixture 07 (leaf PT 落 MMIO, load cause 5) 互补; 07/08 加起来覆盖 mmu_walk 两段
# IS_GPA_RAM 检查 (level=1 + level=0) 对 "落 MMIO" 的行为。
#
# 触发路径 (mmu_translate_pc SV32 路径, 跟 walker_helper_load 失败路径不同):
#   mret 切 S 后 dispatcher 调 mmu_translate_pc(hart, current_tlb=tlb_table[1][0]);
#   mmu_translate_pc 内 SV32 段 → mmu_walk(MMU_PERM_X) →
#     pte1_pa = satp.PPN<<12 + vpn1*4 = 0x02000000 + 0x200*4 = 0x02000800;
#     IS_GPA_RAM(0x02000800) false (CLINT 不在 RAM 区) → fail;
#     fault_cause = af_cause_for(MMU_PERM_X) = CAUSE_INST_ACCESS_FAULT = 1;
#   mmu_translate_pc 用 trap_set_state (路径 2b, 不长跳) → return → dispatcher
#   continue, 重 fetch from mtvec = m_handler。
#
# satp 设计:
#   MODE = Sv32 (bit 31 = 1)
#   ASID = 0
#   PPN  = 0x02000 (root_pt 故意指 CLINT 0x02000000, 不在 RAM)
#   satp = (1<<31) | 0x02000 = 0x80002000
#
# 控制流:
#   1. M setup: sp + mtvec; call setup_pt() (空); csrw satp = 0x80002000 + sfence;
#      mepc = s_target + mstatus.MPP=S; mret 切 S
#   2. mret 后: dispatcher 调 mmu_translate_pc → walker level=1 PT RAM 检查 fail
#      → trap 1 (路径 2b); s_target 内任何指令都不会执行
#   3. M handler: csrr mcause/mepc/mtval → x12/x13/x14; halt
#
# 期望 main.c dump:
#   x10 = 0                        (s_target 永未执行)
#   x12 = 1                        (mcause = INST_ACCESS_FAULT)
#   x13 = 0x80000040 附近          (mepc = s_target PA, 失败 fetch 的 vaddr; 看反汇编)
#   x14 = 0x80000040 附近          (mtval = s_target PA, trap_set_state 第 2 参 = gva)
#   in_trap=3 mcause=11 priv=3
#
# 关键验证点:
#   - x12 = 1 → mmu_translate_pc 真在 PT-落-MMIO 路径报 INST_ACCESS_FAULT (跟 cause 12
#     page fault 区分)
#   - x14 = s_target PA → trap_set_state 路径 mtval 协议跟 trap_raise_exception 一致

.equ CSR_TOHOST, 0x800
.equ CSR_PRIVRD, 0xCC0

.section .text
.option norvc
.globl _start
_start:
    lui  sp, 0x80300

    la   x4, m_handler
    csrw mtvec, x4

    call setup_pt                   # 空函数, 占位

    # satp.PPN = 0x02000 (root_pt 故意指 CLINT, 不在 RAM)
    # satp = (1<<31) | 0x02000 = 0x80002000
    lui  x6, 0x80002                # x6 = 0x80002000
    csrw satp, x6
    sfence.vma

    la   x7, s_target
    csrw mepc, x7

    addi x4, x0, 1
    slli x4, x4, 11
    csrw mstatus, x4

    mret                            # M→S; fetch s_target → walker level=1 fail → trap 1

s_target:
    # 永未执行段 (mret 后 fetch 这条之前就 trap)
    ecall                           # 防御

m_handler:
    csrr x12, mcause                # x12 = 1
    csrr x13, mepc
    csrr x14, mtval

halt:
    csrw mtvec, x0
    ecall                           # cause=11 from M → triple fault
