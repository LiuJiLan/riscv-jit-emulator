# a_01_10 / 02_sum_store_fast_path
#
# ===== 测试目的 =====
# 验 lsu store fast path SUM corner case (a01_9 (6) → a_01_10 修):
#   S+PTE_U=1+SUM=1 sw user page → check_perm(W) 通过 → 成功
#   S+PTE_U=1+SUM=0 sw user page → check_perm(W) 拒绝 → store page fault (cause 15)
#
# ===== 思路 =====
# 跟 fixture 01 (load SUM 验证) 互补 — 02 验 store 路径; 验证哲学跟 01 同。
#
# store fast path 哲学注 (跟 lsu.h load fast path 不同):
#   store_helper 整体是 slow path (extern 函数 helper call), 内部 "TLB hit fast path" 段
#   是 helper 内的快速路径, 不是真 fast path (load_helper static inline 才是真 fast path)。
#   命名澄清见 dummy.txt §1 F1 段。
#   本 fixture 验的是 store_helper 内"V+tag+D+check_perm(W)" 命中段对 SUM corner case 的
#   正确处理 — store 不存在 "fast path 错放过" 单独路径, 但 helper 内命中段仍要走 check_perm
#   保证跟 walker 同源, 不让 SUM 漏检。
#
# PT layout (跟 fixture 01 几乎一致, leaf_pt[1] 加 PTE_W+PTE_D):
#   root_pt @ PA 0x80100000:
#     [0x200] = pointer-to-next-level → leaf_pt PPN | V          = 0x20040401
#   leaf_pt @ PA 0x80101000:
#     [0]     (vaddr 0x80000000) = PPN 0x80000 | V|R|X|A         = 0x2000004B  (代码段)
#     [1]     (vaddr 0x80001000) = PPN 0x80200 | V|R|W|U|A|D     = 0x200800D7  (**PTE_U+W+D**)
#   数据 page @ PA 0x80200000 (sw 写, 不预填; SUM=1 sw 后 lw 回读验)
#
# satp = (1<<31) | 0x80100 = 0x80080100
#
# 控制流:
#   1. M setup: 跟 fixture 01 同模板; mstatus.SUM=1 initial; 预设 x10=x11=0
#      mret 切 S, pc=s_phase_sum1
#   2. s_phase_sum1 (S+SUM=1): sw 0xCAFEBABE → vaddr 0x80001000 (期望成功)
#      lw x10, 0(x4) → x10 = 0xCAFEBABE (回读验 sw 写入)
#      ecall → cause=9 deliver M
#   3. m_handler phase1 (cause=9): csrrc 清 mstatus.SUM=0; mepc=s_phase_sum0; mret
#   4. s_phase_sum0 (S+SUM=0):
#      li x11, 0xDEADBEEF (预设比较值, 验 sw 不影响 GPR 副效)
#      sw x11, 0(x4) → 期望 store page fault cause 15 (check_perm(W) 拒绝)
#                        → mem 内容不变 (但本 fixture 不验内存, 只验 cause/mtval)
#   5. m_handler phase2 (cause=15): dump x12=mcause(15)/x13=mtval(0x80001000)/x14=mepc;
#      csrw mtvec=0; ecall → triple fault

.equ CSR_TOHOST,    0x800
.equ CSR_PRIVRD,    0xCC0
.equ MSTATUS_MPP_S, (1 << 11)
.equ MSTATUS_SUM,   (1 << 18)

.section .text
.option norvc
.globl _start
_start:
    # ---- 设 sp + mtvec ----
    lui  sp, 0x80300
    la   x4, m_handler
    csrw mtvec, x4

    # ---- C 函数建 PT ----
    call setup_pt

    # ---- 设 satp ----
    lui  x6, 0x80080
    addi x6, x6, 0x100              # satp = 0x80080100
    csrw satp, x6
    sfence.vma

    # ---- 设 mret 目标 ----
    la   x7, s_phase_sum1
    csrw mepc, x7

    # ---- 设 mstatus.MPP=PRIV_S | mstatus.SUM=1 (initial) ----
    li   x4, MSTATUS_MPP_S
    li   x5, MSTATUS_SUM
    or   x4, x4, x5                 # x4 = 0x40800
    csrw mstatus, x4

    # ---- 预设 x10/x11 标记 ----
    li   x10, 0
    li   x11, 0

    mret                            # M→S, pc=s_phase_sum1

# ============================================================================
# s_phase_sum1: S 态 + SUM=1, sw user page 期望成功 + lw 回读验
# ============================================================================
s_phase_sum1:
    lui  x4, 0x80001                # x4 = 0x80001000 (vaddr 数据段)
    li   x5, 0xCAFEBABE
    sw   x5, 0(x4)                  # SUM=1 sw, 期望成功
    lw   x10, 0(x4)                 # 回读: 期望 x10 = 0xCAFEBABE

    ecall                           # cause=9 deliver M; m_handler phase1 切 SUM=0

# ============================================================================
# s_phase_sum0: S 态 + SUM=0, sw user page 期望 store page fault cause 15
# ============================================================================
s_phase_sum0:
    li   x11, 0xDEADBEEF            # 预设比较值 (sw page fault 不动 GPR, x11 保留此值)
    sw   x11, 0(x4)                 # SUM=0 sw, 期望 store page fault cause 15
    j    halt                       # 防御 (不应到达)

# ============================================================================
# m_handler: 按 cause 分流到 phase1 / phase2
# ============================================================================
m_handler:
    csrr t1, mcause
    li   t2, 9
    beq  t1, t2, m_handler_phase1
    li   t2, 15
    beq  t1, t2, m_handler_phase2
    j    halt                       # 防御 (other cause)

m_handler_phase1:
    # 清 mstatus.SUM=0
    li   t1, MSTATUS_SUM
    csrrc x0, mstatus, t1

    la   t1, s_phase_sum0
    csrw mepc, t1
    mret                            # 回 S, pc=s_phase_sum0, SUM=0

m_handler_phase2:
    # 保留 x10/x11; dump 关键 csr 到 GPR
    csrr x12, mcause                # 期望 15 (store page fault)
    csrr x13, mtval                 # 期望 0x80001000 (sw GVA)
    csrr x14, mepc                  # sw PC (s_phase_sum0 行)
    j    halt

halt:
    csrw mtvec, x0
    ecall                           # cause=11 from M → mmu fail → triple fault

# ===== 期望结果 =====
# 期望 main.c dump:
#   x10    = 0xCAFEBABE   (SUM=1 sw 成功, lw 回读到 sw 写入值 — 核心验证点 1)
#   x11    = 0xDEADBEEF   (预设比较值; sw page fault 不影响 GPR — 核心验证点 2 间接)
#   x12    = 15           (CAUSE_STORE_PAGE_FAULT — phase2 mcause)
#   x13    = 0x80001000   (mtval = sw GVA — phase2 mtval)
#   x14    = s_phase_sum0 sw 指令 PC (具体值看 gcc 编出来)
#   in_trap = 3
#   mcause = 11           (handler 末 ecall_from_M 第 2 次, 字段保留)
#   priv   = 3            (M)
#
# 关键验证点:
#   - x10 = 0xCAFEBABE → SUM=1 路径 sw 通过 store_helper 命中段 (check_perm(W) 通过) +
#                          lw 回读对 (验 sw 真把值写进了 PA 0x80200000)
#   - x12 = 15 → cause 正确 (store page fault, 不是 store access fault 7)
#   - x13 = 0x80001000 → mtval = sw GVA (跟 mmu_walker_helper_store 内 trap_raise tval 一致)
#
# 跟 fixture 01 比较 (load vs store cause/路径):
#   01: cause 13 (load page fault), trap_raise 在 mmu_walker_helper_load 内
#   02: cause 15 (store page fault), trap_raise 在 mmu_walker_helper_store 内
#       (store_helper 命中段 check_perm 拒绝 → fall back walker → walker check_perm 拒绝
#        → trap_raise(15) 长跳)
