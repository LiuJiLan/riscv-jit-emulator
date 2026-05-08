# a_01_8 / 06_sv32_load_page_fault
# 验 SV32 walker → trap 路径 (load page fault, cause 13)。
# PT 故意构造 leaf_pt[1] PTE.V=0; S phase lw vaddr 0x80001000 触发 walker fail →
# walker_helper_load 内 trap_raise_exception(13, gva) 长跳 → dispatcher trap deliver →
# M handler csrr 验 mcause/mtval/mepc。
#
# PT layout (4KB 页路径, 数据段 V=0):
#   root_pt @ PA 0x80100000:
#     [0x200] = 0x20040401 (pointer-to-next-level, 跟 fixture 04/05 同)
#   leaf_pt @ PA 0x80101000:
#     [0]  = 0x2000004B (代码段, S 跑 fetch 用)
#     [1]  = 0x00000000 (V=0; walker level=0 看到 V=0 → fault_cause = pf_cause(R) = 13)
#
# walker 失败路径 (mmu.c level=0 leaf 段): 看到 PTE.V=0 时 fault_cause 设为
# pf_cause_for(perm); load 路径 perm=R, pf_cause = CAUSE_LOAD_PAGE_FAULT = 13.
#
# 控制流:
#   1. M setup: 跟 04/05/07 同, sp + mtvec; call setup_pt(); satp + sfence; mret 切 S
#   2. S phase:
#      lui  x4, 0x80001          # vaddr 数据段 (PT 里 V=0)
#      lw   x10, 0(x4)            # → walker(R) 失败 → trap 13 (不 return; longjmp)
#      ecall                      # 防御 (期望被上一条 lw 触发 trap 提前; 不会跑到这里)
#   3. M handler:
#      csrr x12, mcause           # 期望 13 (LOAD_PAGE_FAULT)
#      csrr x13, mepc             # lw 指令 PC (期望 0x80000044, 见反汇编)
#      csrr x14, mtval            # 期望 0x80001000 (lw 的 GVA)
#      ecall halt → triple fault (但 mcause/mtval/mepc 会被第 2 次 trap 覆盖, GPR 保留)
#
# 期望 main.c dump:
#   x10 = 0                  (lw 失败没写; 默认 0)
#   x12 = 13                 (mcause = LOAD_PAGE_FAULT, csrr 到 GPR 保留)
#   x13 = 0x80000044         (lw 指令 PC, 实际值看反汇编)
#   x14 = 0x80001000         (mtval = lw GVA)
#   mcause = 11              (handler 末 ecall_from_M, 字段 in_trap=3 第 2 次保留)
#   mtval  = 0               (ecall_from_M 不存 tval)
#   priv = 3, in_trap = 3
#
# 关键验证点:
#   - x12 = 13 → walker 失败正确报 LOAD_PAGE_FAULT
#   - x14 = 0x80001000 → mtval 写的是失败 lw 的 GVA (跟 trap_raise_exception 第 2 参一致)
#   - x13 = lw PC → mepc 写的是失败 lw 指令的 PC

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
    # ---- S phase, 触发 LOAD_PAGE_FAULT ----
    lui  x4, 0x80001                # x4 = 0x80001000 (PT 里 V=0)
    lw   x10, 0(x4)                 # walker(R) 失败 → trap 13 (不 return)
    ecall                           # 防御; 期望 lw 上面就 trap, 这条不该跑

m_handler:
    csrr x12, mcause                # x12 = 13 (LOAD_PAGE_FAULT)
    csrr x13, mepc                  # x13 = 失败 lw 的 PC
    csrr x14, mtval                 # x14 = 0x80001000 (失败 lw 的 GVA)

halt:
    csrw mtvec, x0
    ecall                           # cause=11 from M → triple fault
