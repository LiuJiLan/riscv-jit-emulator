# a_02_2 / 09_sv32_mmio_hw_managed_ad
#
# ===== 测试目的 =====
# 验 SV32 walker hw-managed A/D 在 leaf PA 是 MMIO 时的行为
# (a_01_8/05 RAM 版的 MMIO 版本)。
#
# ===== 思路 =====
# 关键 (a_02_003 后): A/D 是 PTE 描述的"page 被 access/dirty", 跟 page 内容
#        (RAM/MMIO) 在概念上无关 — 但 P2 简化哲学下, MMIO 路径不写回 PT 让 walker_helper
#        的"成功写回"路径只在 RAM 路径走 (跟 Spike fail-不-set 一致简化版)。
#
# PT layout (跟 a_02_2/04 同, 但 leaf_pt_mmio[4] 起步 A=0 D=0):
#   leaf_pt_mmio[4] = 0x00801007    (V|R|W; 起步 A=0 D=0; PPN=0x02004 → mtimecmp page)
#
# 控制流:
#   1. M setup: sp + mtvec; call setup_pt; satp + sfence; mret 切 S
#   2. S phase:
#      sw 0xDEADBEEF → vaddr 0x02004000 (mtimecmp 低半):
#        TLB miss (MMIO 不进 TLB) → lsu_store_helper SV32 路径 fall back
#        mmu_walker_helper_store → mmu_walk check_perm OK + 算建议 set A+D
#        (但 P2 后**不**写回 PT) → walker_helper 判 IS_GPA_RAM(pa)=false → MMIO 路径
#        不写回 PT (P2 简化) → mmio_write_helper → CLINT mtimecmp[0] 低半 = 0xDEADBEEF
#      ecall (cause=9 from S)
#   3. M handler (M-mode BARE):
#      csrr mcause → x12 (期望 9)
#      lui x4, 0x02004 + lw x10, 0(x4): 直接读 CLINT mtimecmp[0] 低半 (走 BARE
#        lsu_load_helper → IS_GPA_RAM false → mmio_read_helper)
#        期望 x10 = 0xDEADBEEF (S phase 写进去了)
#      lui x14, 0x80102 + lw x15, 16(x14):
#        x14 = 0x80102000 (leaf_pt_mmio PA); leaf_pt_mmio[4] offset = 4*4 = 16
#        x15 = 当前 PTE 值, 期望 0x00801007 (P2 后 MMIO 路径不写回 = 起步原值)

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
    # 第一次 sw —— TLB miss → walker → set A+D 写回 PT → mmio_write_helper → CLINT
    lui  x4, 0x02004                # vaddr 0x02004000 (mtimecmp 低半)
    li   x5, 0xDEADBEEF
    sw   x5, 0(x4)

    ecall                           # cause=9 from S

m_handler:
    csrr x12, mcause                # x12 = 9 (ecall from S)

    # 读 CLINT mtimecmp[0] 低半 — M BARE → lsu_load_helper BARE → mmio_read_helper
    lui  x4, 0x02004
    lw   x10, 0(x4)                 # x10 = 0xDEADBEEF (期望; S phase 写进去的)

    # 读 leaf_pt_mmio[4] 验 walker set A+D 后值
    lui  x14, 0x80102               # x14 = 0x80102000 (leaf_pt_mmio PA, M BARE identity)
    lw   x15, 16(x14)               # leaf_pt_mmio[4] offset 16; 期望 0x008010C7

halt:
    csrw mtvec, x0
    ecall                           # cause=11 from M → triple fault

# ===== 期望结果 =====
# 期望 main.c dump (a_02_003 后):
#   x10 = 0xDEADBEEF             (M handler 读 CLINT 验 S phase 写成功)
#   x12 = 9                      (ecall_from_S)
#   x14 = 0x80102000             (leaf_pt_mmio PA)
#   x15 = 0x00801007             (PT 内 leaf_pt_mmio[4]; P2 后 MMIO 路径不写回 PT,
#                                  PTE 保留起步值 0x00801007, A=0 D=0)
#   in_trap=3 mcause=11 priv=3
#
# 关键验证点 (a_02_003 后):
#   - x10 = 0xDEADBEEF → S phase sw 经 walker → mmio_write_helper 真到 CLINT, 后续
#     M-mode BARE lw 经 mmio_read_helper 读出来对 (MMIO 写读链路通)
#   - x15 = 0x00801007 → P2 后 walker 算了建议 A+D 但 caller (walker_helper) 因 MMIO
#     路径不写回 PT, PTE 保持起步原值 (跟 Spike "fail 不 set" 一致简化版)
#   - 跟 a_01_8/05 (leaf RAM 时 set A+D 写回, P2 后 caller 在 RAM 路径写回) **形态
#     对偶** → 证 walker_helper 内 IS_GPA_RAM 分流正确解耦 RAM/MMIO 写回路径
#
# ============================================================================
# ⚠ Milestone-tracked 期望 (回头跑前先看这段)
# ============================================================================
# 本 fixture 期望 (主要是 x15 验 PT 内容) 随 mmu A/D 写回时机演进过两套:
#
#   a_02_002 ~ a_02_002 末 (P2 修前):
#     x15 = 0x008010C7
#     语义: walker 在 mmu_walk 内直接 memcpy 写回 PT, 跟 leaf 是 RAM/MMIO 完全
#            解耦 (PT 在 RAM, 写回 PT 是写 RAM); MMIO 失败 trap 时 PTE.A/D 已 set
#            (跟 Spike "fail 不 set" 不一致, RV spec implementation-defined 不违规)
#     design intent: 验"walker 内 set A+D 写回 PT 跟 leaf 类型解耦", 跟 a_01_8/05
#                     (RAM 版) 形态完全一致
#
#   a_02_003 起 (P2 修后, session_005):
#     x15 = 0x00801007    ← **当前期望, 跟现 src 一致**
#     语义: mmu_walk 只**算**建议 set 后的 new_pte, 不真写回 PT; caller
#            (mmu_walker_helper_store) 在 RAM 路径才 memcpy 写回, MMIO 路径不写回。
#            跟 Spike "fail 不 set" 一致简化版 (MMIO 成功访问也不 set, 因 MMIO PTE
#            不缓存到 TLB, 每次重 walk 时按 PT 原值, 行为正确)。
#     design intent: 验"P2 后 MMIO 路径不写回 PT", 跟 a_01_8/05 (RAM 版仍 set+写回)
#                     **形态对偶** — 同一份 walker 算 A/D 逻辑, RAM/MMIO 分流由 caller
#                     决定写回与否 (walker_helper_store IS_GPA_RAM 分支)。
#
# 行为变化无 src 兼容性问题 (a_01_8 + a_01_10 + a_02_2/04-08 都 PASS 无碍, 因它们
# 不在 SV32+MMIO 写路径之外验 PT 内容); 仅本 fixture (09) 期望值变。
# ============================================================================
