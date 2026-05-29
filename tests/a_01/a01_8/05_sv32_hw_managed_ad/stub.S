# a_01_8 / 05_sv32_hw_managed_ad
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验 SV32 walker 内 hw-managed A/D —— PT init PTE.A=0 D=0; 经一次 store 后 walker 应
# set A=1 + D=1 写回 PT。fixture 04 是 PTE.A=1+D=1 直接走 fast path; 05 让起步是 0,
# 强制 walker fall back 路径 + 验 PT 真被改。设计点 D18 #5 (hw-managed A/D) 单测。
#
# ===== 思路 =====
# ============================================================================
# ⚠ Milestone-tracked 实现演进 (期望值不变, 但实现细节有变)
# ============================================================================
# 本 fixture 期望 (x15 = 0x200800C7) 在 a_02_002 ~ a_02_003 之间**保持不变** (RAM
# 路径行为兼容), 但 mmu A/D 写回点经过演进:
#
#   a_02_002 ~ a_02_002 末 (P2 修前):
#     mmu_walk 内部直接 memcpy 写回 PT (set A+D 后); 单点写回, walker 完成所有工作
#
#   a_02_003 起 (P2 修后, session_005):
#     mmu_walk 只**算**建议 set 后的 new_pte, 不真写回; 通过新 out 参数
#     (pte_wb_pa_out, pte_wb_new_out) 把"写回任务"返给 caller; 三个 caller
#     (mmu_translate_pc / mmu_walker_helper_load / store) 在 RAM 路径才 memcpy 写回。
#     本 fixture 走 RAM 路径 (leaf_pt[1] PPN=0x20080 → 0x80080000 RAM 区), 所以
#     caller 仍会写回, 期望 x15 = 0x200800C7 不变 — fixture **行为兼容**。
#
#   对偶 fixture: a_02_2/09 (leaf MMIO 版) 在 P2 后期望变 (MMIO 路径不写回, PTE
#     保留起步值, 跟 Spike fail-不-set 一致简化版); 本 fixture (RAM 版) 仍是
#     正向"set+写回"形态。两个 fixture 共同验 walker_helper IS_GPA_RAM 分流正确解
#     耦 RAM/MMIO 写回路径。
#
# 以下 doc 中提到"walker 内 set A+D 写回 PT" / "walker 真在 PT 上 set" 等描述,
# a_02_003 起精确说应是"walker 算 + caller (mmu_walker_helper_store) 在 RAM 路径
# 写回"。本 doc 为兼容 a_02_002 前的注释风格保持原措辞, 实现细节见 src/core/mmu.h
# "hw-managed A/D 设计" 段。
# ============================================================================
#
# 跟 fixture 04 同模板 (.S+.c, 4KB 页路径, 同 PT 位置 + 同 vaddr 数据段); 区别两处:
#   1. main.c setup_pt(): leaf_pt[1] PTE 起步 = 0x20080007 (V|R|W, A=0 D=0)
#   2. stub.S handler 段加 lw 验 PT @ leaf_pt[1] 已被 walker set 到 0x200800C7
#
# PT layout (跟 fixture 04 同, 仅 leaf_pt[1] 起步 A=0 D=0):
#   root_pt @ PA 0x80100000:
#     [0x200] = 0x20040401 (pointer-to-next-level)
#   leaf_pt @ PA 0x80101000:
#     [0]  = 0x2000004B (代码段, V|R|X|A; A=1 让 fetch fast path 命中, 不验 fetch hw-managed)
#     [1]  = 0x20080007 (数据段, V|R|W; A=0 D=0 起步, 验 store hw-managed)
#
# 控制流:
#   1. M setup: sp + mtvec; call setup_pt(); csrw satp + sfence; mret 切 S
#   2. S phase 走 walker leaf_pt[1] 4KB 页:
#      sw 0xCAFEBABE → vaddr 0x80001000:
#        TLB miss → store_helper fall back walker → check_perm OK →
#        set A+D 写回 PT (leaf_pt[1]: 0x20080007 → 0x200800C7) →
#        fill TLB → host store
#      lw  vaddr 0x80001000 → x10:
#        TLB hit fast path (V+tag+R 命中, A 不查) → host load → x10 = 0xCAFEBABE
#      ecall (cause=9 from S)
#   3. M handler:
#      csrr mcause → x12 (期望 9)
#      lui x14, 0x80101 → x14 = leaf_pt PA (M-mode BARE identity)
#      lw  x15, 4(x14)  → x15 = leaf_pt[1] PTE 当前值 (期望 0x200800C7, A+D 都 1)
#      test_dev FINISHER PASS 干净停机

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

    mret                            # M→S, pc=identity vaddr

s_target:
    # ---- S phase, 走 walker hw-managed A/D 路径 ----
    lui  x4, 0x80001                # x4 = 0x80001000 (数据段 vaddr)

    # 第一次 sw —— TLB miss → store_helper fall back walker → walker check_perm OK
    # → set A+D 写回 PT (leaf_pt[1]: 0x20080007 → 0x200800C7) → fill TLB → host store
    li   x5, 0xCAFEBABE
    sw   x5, 0(x4)

    # lw —— TLB hit fast path (上次 sw 已 fill; V+tag+R 命中, A 不查) → host load
    lw   x10, 0(x4)                 # x10 = 0xCAFEBABE

    ecall                           # cause=9 from S; trap deliver M

m_handler:
    # ---- M handler ----
    csrr x12, mcause                # x12 = 9 (ecall from S)

    # 读 PT 验 leaf_pt[1] 已被 walker set 到 A+D=1
    lui  x14, 0x80101               # x14 = 0x80101000 (leaf_pt PA, M BARE identity)
    lw   x15, 4(x14)                # x15 = leaf_pt[1] = 0x200800C7 (期望)

halt:
    li   x30, 0x100000              # test_dev base
    li   x31, 0x5555                # FINISHER PASS magic
    sw   x31, 0(x30)                # 干净停机: test_dev 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望 main.c dump:
#   x10 = 0xCAFEBABE     (sw 写进数据 page, lw 读出来)
#   x12 = 9              (ecall_from_S)
#   x14 = 0x80101000     (leaf_pt PA)
#   x15 = 0x200800C7     (PT 内 leaf_pt[1] 经 walker set A+D 后的值; 起步是 0x20080007)
#   x30 = 0x100000       (停机用 scratch: test_dev base)
#   x31 = 0x5555         (停机用 scratch: FINISHER PASS magic)
#   priv = 3
#   停机 = test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
# (注: 绝对 pc / total_count 随 test_dev 收尾漂, 见 REVIEW B.3 段; 不在断言内。)
#
# 关键验证点:
#   - x10 对 → sw → walker → host store 链路真走通 (而不是 walker 跳过没实际写)
#   - x15 = 0x200800C7 → walker 真在 PT 上 set A+D=1 写回 (起始 0x20080007, set 后高位 0xC7)
#   - 跟 fixture 04 比 (PT init 已 A+D=1), 05 强制 walker fall back 路径
