# a_01_8 / 07_sv32_superpage_lw_sw
#
# ===== 测试目的 =====
# 验 lsu SV32 在 4MB superpage (level=1 leaf) 上 lw/sw 走通 + fast path 命中。
# 跟 fixture 04 (level=1 → level=0 4KB 路径) 互补, 验 walker 另一条主路径。
# 跟 fixture 02 (superpage fetch) 互补, 02 只验 fetch, 07 加 lw/sw 验 lsu superpage。
#
# ===== 思路 =====
# PT layout (4MB superpage, identity):
#   root_pt @ PA 0x80100000:
#     [0x200] (vaddr 0x80000000 段 4MB) = 4MB superpage leaf
#       PTE = (PPN[1]=0x200<<20) | (PPN[0]=0<<10) | V|R|W|X|A|D = 0x200000CF
#       数据/代码 identity 映射: vaddr 0x80000000-0x803FFFFF → PA 同
#   不设 leaf_pt (4MB superpage 不进 level=0)
#   数据段经 superpage identity → vaddr 0x80001000 → PA 0x80001000 (RAM 内空闲, 不
#   跟代码段冲突 — fixture 体积 < 0x1000)
#
# 注意 TLB 行为 (mmu.h D18 #9): walker 走 superpage 算出 PA 后 fill TLB 仍按 4KB 粒度
# (host_ptr 抹低 12 位); 同 4KB 内连续 sw/lw 命中 fast path, 跨 4KB 边界要重 walk。
# fixture 07 sw/lw 全在同 4KB page (vaddr 0x80001000 + 0/4 offset), fast path 命中。
#
# 控制流:
#   1. M setup: sp + mtvec; call setup_pt() (写 root_pt[0x200] superpage leaf, 不设
#      leaf_pt); csrw satp + sfence; mret 切 S
#   2. S phase 走 walker superpage (跟 fixture 04 同序列):
#      sw 0xCAFEBABE → vaddr 0x80001000 (TLB miss, walker 走 superpage 路径, fill TLB)
#      lw  vaddr 0x80001000 → x10 (fast path)
#      sw 0x12345678 → vaddr 0x80001004 (fast path 命中, 同 4KB)
#      lw  vaddr 0x80001004 → x11 (fast path)
#      ecall (cause=9 from S)
#   3. M handler: csrr mcause → x12; halt triple fault

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

    # ---- C 函数建 PT (BARE M-mode, identity, walker 未激活) ----
    call setup_pt

    # ---- 设 satp: MODE=Sv32 + PPN=0x80100 ----
    lui  x6, 0x80080
    addi x6, x6, 0x100              # satp = 0x80080100
    csrw satp, x6
    sfence.vma

    # ---- 设 mret 目标 = s_target identity vaddr ----
    la   x7, s_target
    csrw mepc, x7

    # ---- 设 mstatus.MPP = PRIV_S ----
    addi x4, x0, 1
    slli x4, x4, 11                 # x4 = 0x800
    csrw mstatus, x4

    mret                            # M→S

s_target:
    # ---- S phase, 走 walker 经 root_pt[0x200] 4MB superpage (identity) ----
    lui  x4, 0x80001                # x4 = 0x80001000 (vaddr; superpage identity → PA 同)

    # 第一次 sw + lw —— TLB miss → walker 走 superpage 路径 → fill TLB (4KB 粒度) →
    # host store; lw fast path 命中 (V+tag+R)
    li   x5, 0xCAFEBABE
    sw   x5, 0(x4)
    lw   x10, 0(x4)                 # x10 = 0xCAFEBABE

    # 第二次 sw + lw 同 4KB —— 全 fast path
    li   x6, 0x12345678
    sw   x6, 4(x4)
    lw   x11, 4(x4)                 # x11 = 0x12345678

    ecall                           # cause=9 from S; trap deliver M

m_handler:
    csrr x12, mcause                # x12 = 9

halt:
    csrw mtvec, x0
    ecall                           # cause=11 from M → triple fault

# ===== 期望结果 =====
# 期望 main.c dump (跟 fixture 04 同 — 只是 PT 路径不同, S phase 行为相同):
#   x10 = 0xCAFEBABE
#   x11 = 0x12345678
#   x12 = 9
#   mcause = 11, priv = 3, in_trap = 3, mtvec = 0
#
# 关键验证点:
#   - x10 / x11 都对 → walker superpage 路径正确 + fill TLB 4KB 粒度 OK
#   - 跟 fixture 04 比 (4KB leaf 路径), 07 走 superpage 路径; 两个加起来覆盖 mmu_walk
#     两条主路径 (level=1 leaf 4MB + level=1→level=0 leaf 4KB)
