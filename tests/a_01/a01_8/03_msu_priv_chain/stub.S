# a_01_8 / 03_msu_priv_chain
# 综合 fixture (f) sret 切 priv + (g) SV32 walker, 验 MSU 三态都能正确通过 mmu fetch
# + csrr CSR_PRIVRD (0xCC0) 验当前 priv 值 + csrw CSR_TOHOST (0x800) 输出.
#
# RV spec check_perm 矛盾点: 同一个 PTE 不能同时让 S 和 U 跑 fetch:
#   - S-mode 默认能访问 PTE.U=0 (kernel page); 不能 fetch X-on-PTE.U=1 (X 路径不享 SUM)
#   - U-mode 必须 PTE.U=1 (user page)
# 解决: 用两个不同 4MB superpage, 不同 PTE.U:
#   root_pt[0x200] (vaddr 0x80000000 段) = PTE.U=0 → PA 0x80000000  (S/M identity 用)
#   root_pt[0x210] (vaddr 0x84000000 段) = PTE.U=1 → PA 0x80000000  (U 远 vaddr 用)
# 两段 vaddr 都映射到 PA 0x80000000 (= 物理 fixture program 起点); fixture 体在 PA RAM 区,
# S 用 identity vaddr fetch; U 用远 vaddr fetch (= 同 PA, 但走不同 root_pt entry 的 PTE.U=1).
#
# 路径:
#   1. M setup: build PT (两个 superpage 不同 U) + csrw satp + sfence + csrr 0xCC0=3 →
#      x10 + csrw 0x800 (tohost=3) + 设 mepc=s_target identity vaddr + mstatus.MPP=S → mret 切 S
#   2. S phase (走 walker root_pt[0x200] PTE.U=0): csrr 0xCC0=1 → x11 + csrw 0x800 (tohost=1)
#      + 设 sepc=u_target 远 vaddr + sstatus.SPP=U → sret 切 U
#   3. U phase (走 walker root_pt[0x210] PTE.U=1): csrr 0xCC0=0 → x12 + csrw 0x800 (tohost=0)
#      + ecall (cause=8 from U)
#   4. M handler: csrr 0xCC0=3 → x13 + csrw 0x800 (tohost=3) + 验 cause=8 + halt triple fault
#
# 期望 main.c dump:
#   x10 = 3                      (M phase priv)
#   x11 = 1                      (S phase priv, 经 SV32 walker fetch)
#   x12 = 0                      (U phase priv, 经 SV32 walker fetch + PTE.U=1)
#   x13 = 3                      (M handler priv)
#   x14 = 8                      (handler csrr mcause)
#   x15 = 8                      (比较 reg)
#   tohost = 3                   (handler 末 csrw, 覆盖之前)
#   pc = 0
#   in_trap = 3
#   mcause = 11                  (handler 末 ecall, 字段保留第 2 次)
#   mepc = 0x800000xx            (handler 末 ecall PC, 实际值跑出来核对)
#   mtvec = 0
#   priv = 3
#   mstatus = 0x18A0 (?)         (推算: SPIE=1 sret 残留 + MPP=M=0x1800 + ... 跑出来核对)
#
# 关键验证点:
#   - x10/x11/x12/x13 = 3/1/0/3 → MSU 三态都被正确 csrr 0xCC0 拿到 priv (= csr_privrd_read 对)
#   - x11 = 1 + S phase fetch 实际经 SV32 walker (root_pt[0x200] PTE.U=0, S 默认允许)
#   - x12 = 0 + U phase fetch 实际经 SV32 walker (root_pt[0x210] PTE.U=1, U 强制 U=1)
#   - tohost = 3 → handler 写最后

# CSR alias (gcc as 接受 .equ 定义后跟 named csr 一致用法; 直接 numeric 也 OK 但 .equ 可读)
.equ CSR_TOHOST, 0x800
.equ CSR_PRIVRD, 0xCC0

.section .text
.option norvc
.globl _start
_start:
    # ---- 设 M handler ----
    la   x4, m_handler
    csrw mtvec, x4

    # ---- Build root PT @ PA 0x80100000 ----
    # base x4 调到 root_pt + 0x800 (entry 0x200), 0 偏移 sw
    lui  x4, 0x80101
    addi x4, x4, -0x800         # x4 = 0x80100800

    # PTE_S (U=0; V|R|W|X|A|D = 0xCF), PPN=0x80000 → PTE = 0x200000CF
    lui  x5, 0x20000
    addi x5, x5, 0xCF
    sw   x5, 0(x4)              # root_pt[0x200] = PTE_S → vaddr 0x80000000 段, S/M 用

    # PTE_U (U=1; PTE_S | 0x10 = 0xDF), PPN=0x80000 → PTE = 0x200000DF
    addi x5, x5, 0x10           # x5 = 0x200000DF
    addi x4, x4, 0x40           # x4 = 0x80100840 (entry 0x210)
    sw   x5, 0(x4)              # root_pt[0x210] = PTE_U → vaddr 0x84000000 段, U 用

    # ---- 设 satp: MODE=Sv32 + PPN=0x80100 ----
    lui  x6, 0x80080
    addi x6, x6, 0x100          # satp = 0x80080100
    csrw satp, x6
    sfence.vma

    # ---- M phase ----
    csrr x10, CSR_PRIVRD        # x10 = 3 (M)
    csrw CSR_TOHOST, x10        # tohost = 3

    # ---- 设 mret 目标 = s_target identity vaddr (走 walker root_pt[0x200] PTE.U=0) ----
    la   x7, s_target           # x7 = s_target PA = identity vaddr (linker 钉 0x800000xx)
    csrw mepc, x7

    # ---- 设 mstatus.MPP = PRIV_S ----
    addi x4, x0, 1
    slli x4, x4, 11             # x4 = 0x800
    csrw mstatus, x4

    mret                        # M→S, pc=identity vaddr

s_target:
    # ---- S phase (走 walker U=0, S 默认允许) ----
    csrr x11, CSR_PRIVRD        # x11 = 1 (S)
    csrw CSR_TOHOST, x11        # tohost = 1

    # ---- 设 sret 目标 = u_target 远 vaddr (= u_target_PA + 0x04000000) ----
    la   x4, u_target
    lui  x5, 0x4000             # x5 = 0x04000000
    add  x4, x4, x5             # x4 = u_target 远 vaddr (走 walker root_pt[0x210] PTE.U=1)
    csrw sepc, x4

    # ---- 设 sstatus.SPP = PRIV_U = 0 ----
    csrw sstatus, x0            # sstatus = 0 (SPP=0)

    sret                        # S→U, pc=远 vaddr

u_target:
    # ---- U phase (走 walker U=1) ----
    csrr x12, CSR_PRIVRD        # x12 = 0 (U)
    csrw CSR_TOHOST, x12        # tohost = 0

    ecall                       # cause=8 from U; trap deliver M

m_handler:
    # ---- M handler ----
    csrr x13, CSR_PRIVRD        # x13 = 3 (handler M, deliver_priv hard-code)
    csrw CSR_TOHOST, x13        # tohost = 3

    csrr x14, mcause            # 期望 cause=8
    addi x15, x0, 8
    bne  x14, x15, halt         # cause != 8 → halt 防御

halt:
    csrw mtvec, x0
    ecall                       # cause=11 from M → triple fault
