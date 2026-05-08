# a_01_10 / 01_sum_load_fast_path
# 验 lsu load fast path SUM corner case (a01_9 (6) → a_01_10 修):
#   S+PTE_U=1+SUM=1 lw user page → fast path / walker check_perm(R) 通过 → 成功
#   S+PTE_U=1+SUM=0 lw user page → fast path / walker check_perm(R) 拒绝 → load page fault (cause 13)
#
# 跟 a_01_8 fixture 04 (lw_sw_basic, leaf_pt[1] PTE_U=0) 区别:
#   - leaf_pt[1] PTE_U=1 — 核心验证点, S 模式访问受 mstatus.SUM 控制
#   - 单 fixture 跑两 phase: 顺序 SUM=1 (期望 lw 成功) → SUM=0 (期望 page fault)
#   - phase 间通过 ecall 回 M handler 切 SUM (m_handler_phase1 清 SUM mret 回 phase2)
#
# 验证哲学:
#   - a_01_8 时 lsu fast path 简化命中条件 (V+tag+R), 错放过 S+PTE_U=1+SUM=0 corner case
#     (旧 fast path 看 R=1 直接 host load, 但 spec 规定应 page fault)
#   - a_01_10 (6) 修复: check_perm 提到 mmu.h 作 static inline, fast path 命中条件改成
#     V+tag+check_perm(R), 跟 walker 同源
#   - 第 2 次 lw (SUM=0) 正是 a_01_10 (6) 修复的 fast path 路径 — TLB 已被第 1 次 lw fill,
#     entry.pte_flags 含 PTE_U=1; 第 2 次 lw 走 fast path 命中 V+tag, check_perm(R) 拒绝
#     (S+PTE_U=1+SUM=0) → fall back walker → walker check_perm 同样拒绝 → trap_raise(13) 长跳
#
# PT layout (跟 fixture 04 几乎一致, leaf_pt[1] 加 PTE_U):
#   root_pt @ PA 0x80100000:
#     [0x200] = pointer-to-next-level → leaf_pt PPN | V    = 0x20040401
#   leaf_pt @ PA 0x80101000:
#     [0]     (vaddr 0x80000000) = PPN 0x80000 | V|R|X|A   = 0x2000004B  (代码段, PTE_U=0)
#     [1]     (vaddr 0x80001000) = PPN 0x80200 | V|R|U|A   = 0x20080053  (数据段, **PTE_U=1**)
#   数据 page @ PA 0x80200000 预填 0xDEADBEEF
#
# satp = (1<<31) | 0x80100 = 0x80080100
#
# 控制流:
#   1. M setup: sp + mtvec; call setup_pt; csrw satp + sfence.vma;
#      mepc=s_phase_sum1; mstatus.MPP=PRIV_S | mstatus.SUM=1 (initial); 预设 x10=x11=0; mret
#   2. s_phase_sum1 (S+SUM=1): lw x10, 0(x4) → 期望 0xDEADBEEF (check_perm 通过)
#      ecall → cause=9 deliver M
#   3. m_handler phase1 (cause=9): csrrc 清 mstatus.SUM=0; mepc=s_phase_sum0; mret
#   4. s_phase_sum0 (S+SUM=0): lw x11, 0(x4) → 期望 page fault cause 13
#      (check_perm 拒绝, x11 不写, 保留预设 0)
#   5. m_handler phase2 (cause=13): dump x12=mcause(13)/x13=mtval(0x80001000)/x14=mepc;
#      csrw mtvec=0; ecall → cause=11 from M → mmu fail → triple fault
#
# 期望 main.c dump:
#   x10    = 0xDEADBEEF   (SUM=1 lw 成功 — 核心验证点 1)
#   x11    = 0            (SUM=0 lw page fault, x11 不写, 保留预设 0 — 核心验证点 2)
#   x12    = 13           (CAUSE_LOAD_PAGE_FAULT — phase2 mcause)
#   x13    = 0x80001000   (mtval = lw GVA — phase2 mtval)
#   x14    = s_phase_sum0 PC (lw 指令 PC, 具体值看 gcc 编出来)
#   in_trap = 3
#   mcause = 11           (handler 末 ecall_from_M 第 2 次, 字段保留)
#   priv   = 3            (M)
#
# 关键验证点:
#   - x10 = 0xDEADBEEF → SUM=1 路径 lw 通过 fast path / walker (TLB miss → walker fall back)
#   - x11 = 0 → SUM=0 路径 lw 触发 page fault, fast path 命中 V+tag 但 check_perm 拒绝
#                (验 a_01_10 (6) 修复: 旧形态 V+tag+R 命中会错放过)
#   - x12 = 13 → cause 正确 (load page fault, 不是 access fault 5)
#   - x13 = 0x80001000 → mtval = lw GVA (跟 mmu_walker_helper_load 内 trap_raise tval 一致)

.equ CSR_TOHOST,    0x800
.equ CSR_PRIVRD,    0xCC0
.equ MSTATUS_MPP_S, (1 << 11)
.equ MSTATUS_SUM,   (1 << 18)

.section .text
.option norvc
.globl _start
_start:
    # ---- 设 sp (栈在 RAM 内空闲区) ----
    lui  sp, 0x80300

    # ---- 设 M handler ----
    la   x4, m_handler
    csrw mtvec, x4

    # ---- C 函数建 PT + 预填数据段 (BARE M-mode, identity, walker 未激活) ----
    call setup_pt

    # ---- 设 satp: MODE=Sv32 + PPN=0x80100 ----
    lui  x6, 0x80080
    addi x6, x6, 0x100              # satp = 0x80080100
    csrw satp, x6
    sfence.vma                      # 让新 satp 生效

    # ---- 设 mret 目标 = s_phase_sum1 ----
    la   x7, s_phase_sum1
    csrw mepc, x7

    # ---- 设 mstatus.MPP=PRIV_S | mstatus.SUM=1 (initial 让 phase1 lw 通过) ----
    li   x4, MSTATUS_MPP_S
    li   x5, MSTATUS_SUM
    or   x4, x4, x5                 # x4 = 0x800 | 0x40000 = 0x40800
    csrw mstatus, x4

    # ---- 预设 x10/x11 标记 (lw 不写时保留 0; SUM=0 phase lw page fault 不写 x11) ----
    li   x10, 0
    li   x11, 0

    mret                            # M→S, pc=s_phase_sum1

# ============================================================================
# s_phase_sum1: S 态 + SUM=1, lw user page 期望成功
# ============================================================================
s_phase_sum1:
    lui  x4, 0x80001                # x4 = 0x80001000 (vaddr 数据段, leaf_pt[1] PTE_U=1)
    lw   x10, 0(x4)                 # 期望 x10 = 0xDEADBEEF (SUM=1 通过)

    ecall                           # cause=9 deliver M; m_handler phase1 切 SUM=0

# ============================================================================
# s_phase_sum0: S 态 + SUM=0, lw user page 期望 page fault cause 13
# ============================================================================
s_phase_sum0:
    lw   x11, 0(x4)                 # 期望 page fault cause 13 (check_perm 拒绝, x11 不写)
    j    halt                       # 防御 (不应到达)

# ============================================================================
# m_handler: 按 cause 分流到 phase1 (ecall_from_S 切 SUM) / phase2 (page fault dump)
# ============================================================================
m_handler:
    csrr t1, mcause
    li   t2, 9
    beq  t1, t2, m_handler_phase1
    li   t2, 13
    beq  t1, t2, m_handler_phase2
    j    halt                       # 防御 (other cause)

m_handler_phase1:
    # 清 mstatus.SUM=0 (csrrc 不读旧值进 x0)
    li   t1, MSTATUS_SUM
    csrrc x0, mstatus, t1

    # mepc = s_phase_sum0 (覆盖 ecall 的 mepc, 因 ecall 不 advance, 我们要跳到下一段)
    la   t1, s_phase_sum0
    csrw mepc, t1
    mret                            # 回 S, pc = s_phase_sum0, SUM=0

m_handler_phase2:
    # 保留 x10/x11; dump 关键 csr 到 GPR
    csrr x12, mcause                # 期望 13 (load page fault)
    csrr x13, mtval                 # 期望 0x80001000 (lw GVA)
    csrr x14, mepc                  # lw PC (s_phase_sum0 行)
    j    halt

halt:
    csrw mtvec, x0                  # mtvec=0
    ecall                           # cause=11 from M → mmu fail → triple fault
