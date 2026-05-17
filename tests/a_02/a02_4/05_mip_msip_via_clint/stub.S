# a02_4 / 05_mip_msip_via_clint
# mip.MSIP 合成路径验证 — 写 CLINT.msip[0] MMIO → mip.MSIP 合成读看见; csrw mip
# 不能清 MSIP (RO in mip 视角); 只能 MMIO 写 clint.msip[0] = 0 清.
#
# BARE M-mode.
# CLINT msip[0] @ 0x02000000 (config.h CLINT_BASE + CLINT_MSIP_OFFSET).
#
# 期望最终寄存器:
#   a0 (x10) = 0x00000008   写 msip[0]=1 → csrr mip MSIP (bit 3) = 1
#   a1 (x11) = 0x00000008   csrw mip 0 不影响 MSIP (RO); 仍 set
#   a2 (x12) = 0x00000000   写 msip[0]=0 → csrr mip MSIP=0
#
# halt: ecall.

.section .text
.option norvc
.globl _start
_start:
    # --- 1. 写 CLINT.msip[0] = 1 ---
    li      t0, 0x02000000                        # CLINT msip[0] 地址
    li      t1, 1
    sw      t1, 0(t0)                             # clint.msip[0] = 1 (atomic store)

    # csrr mip → bit 3 (MSIP) = 1 (合成路径生效)
    csrr    a0, mip                               # a0 = 8 (1 << IRQ_M_SOFT)

    # --- 2. csrw mip 试清 MSIP (RO 应忽略) ---
    csrw    mip, x0                               # 写 0 进 _mip_sw 软件位 (本来就 0)
    csrr    a1, mip                               # a1 = 8 (MSIP 仍 set, RO)

    # --- 3. 写 CLINT.msip[0] = 0 清 ---
    sw      x0, 0(t0)                             # clint.msip[0] = 0
    csrr    a2, mip                               # a2 = 0

    ecall
