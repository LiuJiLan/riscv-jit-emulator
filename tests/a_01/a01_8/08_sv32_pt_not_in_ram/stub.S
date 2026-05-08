# a_01_8 / 08_sv32_pt_not_in_ram
# 验 SV32 walker 内 PT 不在 RAM 时报 access fault (cause 5; load 路径)。
# 项目 a_01 不实现 PMP, 用 RAM 区检查代替 (mmu.c walker level=0 PT 读检查); 未来 PMP
# 接入时 cause 不变, 只是 RAM 检查改为 PMP allow 检查。
#
# PT 设计:
#   代码段 (vaddr 0x80000000 段, vpn1=0x200) 走 4MB superpage identity, 在 RAM 内, fetch OK
#   数据段 (vaddr 0x90000000 段, vpn1=0x240) 走 pointer-to-next-level, leaf_pt PA 故意指
#     到 0xC0000000 (RAM 外, RAM 范围 0x80000000~0x88000000) → walker level=0 PT 读
#     RAM 检查失败 → access fault
#
# PT layout:
#   root_pt @ PA 0x80100000:
#     [0x200] = 0x200000CF (4MB superpage leaf, identity, 代码段; 跟 fixture 07 同)
#     [0x240] = 0x30000001 (pointer-to-next-level, leaf_pt PA 0xC0000000 不在 RAM)
#                = (0xC0000 << 10) | V = 0x30000000 | 1
#   leaf_pt 不分配 (PA 0xC0000000 故意不存在; walker 检查 fail 不会真去读)
#
# 触发路径 (mmu.c walker level=0 段):
#   walker walk vaddr 0x90000000:
#     vpn1=0x240, vpn0=0x000
#     pte1_pa = root_pt + 0x240*4 = 0x80100900 (RAM 内, 读 OK)
#     pte1 = 0x30000001 (V=1, R=W=X=0 → pointer-to-next-level)
#     pte1_full_pa = 0xC0000 << 12 = 0xC0000000
#     pte0_pa = 0xC0000000 + 0*4 = 0xC0000000
#     (uint32_t)(0xC0000000 - 0x80000000) = 0x40000000 >= GUEST_RAM_SIZE (0x08000000) → fail
#     fault_cause = af_cause_for(R) = CAUSE_LOAD_ACCESS_FAULT = 5
#
# 控制流:
#   1. M setup: sp + mtvec; call setup_pt(); satp + sfence; mret 切 S
#   2. S phase:
#      lui  x4, 0x90000          # vaddr = 0x90000000 (远段 vpn1=0x240)
#      lw   x10, 0(x4)            # walker(R) level=0 PT RAM 检查失败 → trap 5 (longjmp)
#      ecall                      # 防御
#   3. M handler: csrr mcause/mepc/mtval → x12/x13/x14; halt
#
# 期望 main.c dump:
#   x10 = 0                  (lw 失败没写)
#   x12 = 5                  (mcause = LOAD_ACCESS_FAULT)
#   x13 = 失败 lw 的 PC      (mepc; 实际值看反汇编)
#   x14 = 0x90000000         (mtval = 失败 lw 的 GVA)
#   mcause = 11              (handler 末 ecall_from_M 第 2 次)
#   priv = 3, in_trap = 3
#
# 关键验证点:
#   - x12 = 5 → walker 真在 PT-not-in-RAM 路径报 LOAD_ACCESS_FAULT (跟 page fault 区分)
#   - x14 = 0x90000000 → mtval = GVA (跟 page fault fixture 06 同协议)
#   - 跟 fixture 06 (page fault, cause 13) 互补; 06 验 PTE.V=0, 08 验 PT 物理不可达

.equ CSR_TOHOST, 0x800
.equ CSR_PRIVRD, 0xCC0

.section .text
.option norvc
.globl _start
_start:
    # ---- 设 sp ----
    lui  sp, 0x80300

    # ---- 设 M handler ----
    la   x4, m_handler
    csrw mtvec, x4

    # ---- C 函数建 PT ----
    call setup_pt

    # ---- 设 satp: MODE=Sv32 + PPN=0x80100 ----
    lui  x6, 0x80080
    addi x6, x6, 0x100
    csrw satp, x6
    sfence.vma

    # ---- 设 mret 目标 = s_target identity vaddr ----
    la   x7, s_target
    csrw mepc, x7

    # ---- 设 mstatus.MPP = PRIV_S ----
    addi x4, x0, 1
    slli x4, x4, 11
    csrw mstatus, x4

    mret                            # M→S

s_target:
    # ---- S phase, 触发 LOAD_ACCESS_FAULT ----
    lui  x4, 0x90000                # x4 = 0x90000000 (远段 vaddr; PT level=0 在 RAM 外)
    lw   x10, 0(x4)                 # walker(R) → access fault → trap 5 (longjmp)
    ecall                           # 防御

m_handler:
    csrr x12, mcause                # x12 = 5 (LOAD_ACCESS_FAULT)
    csrr x13, mepc                  # x13 = 失败 lw 的 PC
    csrr x14, mtval                 # x14 = 0x90000000

halt:
    csrw mtvec, x0
    ecall                           # cause=11 from M → triple fault
