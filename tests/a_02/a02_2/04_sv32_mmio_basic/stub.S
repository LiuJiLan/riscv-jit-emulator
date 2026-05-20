# a_02_2 / 04_sv32_mmio_basic
#
# ===== 测试目的 =====
# 验 SV32 经 mmu 翻译后到 CLINT MMIO 的读写路径 — a_02_2/01 (BARE) 的 SV32 版本,
# 同时也是 a_01_8/04 (SV32 + RAM lw/sw) 的 MMIO 版本。
#
# ===== 思路 =====
# P3 后路径 (跟 BARE 路径同语义, 不同入口):
#   lsu_load_helper / lsu_store_helper 内 SV32 路径:
#     TLB miss (MMIO 不进 TLB, 永远 miss) → mmu_walker_helper_load/store
#     → mmu_walk 翻译 → IS_GPA_RAM(pa) false → mmio_read_helper / mmio_write_helper
#     (不 fill TLB; 跟 lsu_load_helper SV32 hit 路径 "直接 *hva" 不同)
#
# PT layout: 见 main.c setup_pt()
#   root_pt @ 0x80100000:
#     [0x200] → leaf_pt_code (代码段 vaddr 0x80000000)
#     [0x008] → leaf_pt_mmio (CLINT 段 vaddr 0x02000000)
#   leaf_pt_mmio @ 0x80102000 三 entry (msip / mtimecmp / mtime), 全 V|R|W|A|D 起步
#
# satp = 0x80080100 (Sv32 + PPN=0x80100)
#
# 控制流:
#   1. M setup: sp + mtvec; call setup_pt; csrw satp + sfence; mret 切 S
#   2. S phase: 5 段 MMIO 读写 (跟 a_02_2/01 同, 但经过 SV32 walker):
#      sw 0xDEADBEEF → vaddr 0x02004000 (mtimecmp 低半)
#      sw 0x12345678 → vaddr 0x02004004 (mtimecmp 高半)
#      lw 0x02004000 → a0
#      lw 0x02004004 → a1
#      sw 1          → vaddr 0x02000000 (msip)
#      lw 0x02000000 → a2
#      sw 0xCAFEBABE → vaddr 0x0200BFF8 (mtime 低半)
#      sw 0xFEEDFACE → vaddr 0x0200BFFC (mtime 高半)
#      lw 0x0200BFF8 → a3
#      lw 0x0200BFFC → a4
#      ecall (cause=9 from S)
#   3. M handler: csrr mcause → x15; halt

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
    addi x6, x6, 0x100              # satp = 0x80080100 (Sv32 + PPN=0x80100)
    csrw satp, x6
    sfence.vma

    la   x7, s_target
    csrw mepc, x7

    addi x4, x0, 1
    slli x4, x4, 11                 # mstatus.MPP = S
    csrw mstatus, x4

    mret                            # M→S

s_target:
    # --- mtimecmp[0] 写 + 读 ---
    lui  x4, 0x02004                # vaddr = 0x02004000
    li   x5, 0xDEADBEEF
    sw   x5, 0(x4)
    li   x5, 0x12345678
    sw   x5, 4(x4)
    lw   x10, 0(x4)
    lw   x11, 4(x4)

    # --- msip[0] 写 + 读 ---
    lui  x4, 0x02000                # vaddr = 0x02000000
    li   x5, 1
    sw   x5, 0(x4)
    lw   x12, 0(x4)

    # --- mtime 写 + 读 ---
    # mtime 在 0x0200BFF8 (跨 page 内偏移 0xFF8); leaf_pt_mmio[11] 覆盖 0x0200B000 page
    lui  x4, 0x0200C
    addi x4, x4, -8                 # vaddr = 0x0200BFF8
    li   x5, 0xCAFEBABE
    sw   x5, 0(x4)
    li   x5, 0xFEEDFACE
    sw   x5, 4(x4)
    lw   x13, 0(x4)
    lw   x14, 4(x4)

    ecall                           # cause=9 from S

m_handler:
    csrr x15, mcause                # x15 = 9

halt:
    csrw mtvec, x0
    ecall                           # cause=11 from M → triple fault

# ===== 期望结果 =====
# 期望 main.c dump:
#   a0 (x10) = 0xDEADBEEF
#   a1 (x11) = 0x12345678
#   a2 (x12) = 0x00000001
#   a3 (x13) = 0xCAFEBABE
#   a4 (x14) = 0xFEEDFACE
#   x15      = 9          (mcause from handler entry; ecall_from_S)
#   in_trap=3 mcause=11 priv=3  (handler 末 ecall_from_M 第 2 次保留)
#
# 关键验证点:
#   - 5 个 a 寄存器对 → SV32 walker_helper_load/store 真走 MMIO 分支 + mmio_*_helper
#   - 跟 a_02_2/01 (BARE) dump 完全一致 → 证 lsu_load_helper / lsu_store_helper inline
#     顶层对两条 priv 路径都收敛到同一 mmio_*_helper
