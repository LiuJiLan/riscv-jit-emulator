# a_02_2 / 09_sv32_mmio_hw_managed_ad
# 验 SV32 walker hw-managed A/D 在 leaf PA 是 MMIO 时仍正确 set + 写回 PT
# (a_01_8/05 RAM 版的 MMIO 版本)。
#
# 关键: A/D 是 PTE 描述的"page 被 access/dirty", 跟 page 内容 (RAM/MMIO) 无关;
#        PT 永远在 RAM, set A+D 写回 PT 是写 RAM 不是写 MMIO; 即使 leaf 是 MMIO,
#        walker 仍 set A+D 写回 PT, 然后调 mmio_write_helper 完成 MMIO 写。
#
# PT layout (跟 a_02_2/04 同, 但 leaf_pt_mmio[4] 起步 A=0 D=0):
#   leaf_pt_mmio[4] = 0x00801007    (V|R|W; 起步 A=0 D=0; PPN=0x02004 → mtimecmp page)
#
# 控制流:
#   1. M setup: sp + mtvec; call setup_pt; satp + sfence; mret 切 S
#   2. S phase:
#      sw 0xDEADBEEF → vaddr 0x02004000 (mtimecmp 低半):
#        TLB miss (MMIO 不进 TLB) → store_helper SV32 路径 fall back walker →
#        check_perm OK → walker set A+D 写回 PT (leaf_pt_mmio[4]: 0x00801007 →
#        0x008010C7) → 判 MMIO → mmio_write_helper → CLINT mtimecmp[0] 低半 = 0xDEADBEEF
#      ecall (cause=9 from S)
#   3. M handler (M-mode BARE):
#      csrr mcause → x12 (期望 9)
#      lui x4, 0x02004 + lw x10, 0(x4): 直接读 CLINT mtimecmp[0] 低半 (走 BARE
#        lsu_load_helper → IS_GPA_RAM false → mmio_read_helper)
#        期望 x10 = 0xDEADBEEF (S phase 写进去了)
#      lui x14, 0x80102 + lw x15, 16(x14):
#        x14 = 0x80102000 (leaf_pt_mmio PA); leaf_pt_mmio[4] offset = 4*4 = 16
#        x15 = 当前 PTE 值, 期望 0x008010C7 (起步 0x00801007 + A | D = 0xC0)
#
# 期望 main.c dump:
#   x10 = 0xDEADBEEF             (M handler 读 CLINT 验 S phase 写成功)
#   x12 = 9                      (ecall_from_S)
#   x14 = 0x80102000             (leaf_pt_mmio PA)
#   x15 = 0x008010C7             (PT 内 leaf_pt_mmio[4] 经 walker set A+D 后; A=0 D=0
#                                  起步 0x00801007 + 0xC0 = 0x008010C7)
#   in_trap=3 mcause=11 priv=3
#
# 关键验证点:
#   - x10 = 0xDEADBEEF → S phase sw 经 walker → mmio_write_helper 真到 CLINT, 后续
#     M-mode BARE lw 经 mmio_read_helper 读出来对
#   - x15 = 0x008010C7 → walker 真在 PT 上 set A+D (起步 0x00801007 + 0xC0 = 0x008010C7),
#     证 hw-managed A/D 对 leaf 是 MMIO 的情况仍走通 (PT 在 RAM, 写回 PT 是写 RAM,
#     不被 leaf 是 MMIO 影响)
#   - 跟 a_01_8/05 (leaf 是 RAM 时同样 set A+D 写回) 形态完全一致 → 证 mmu_walk 的
#     A/D 逻辑跟 leaf 类型解耦

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
