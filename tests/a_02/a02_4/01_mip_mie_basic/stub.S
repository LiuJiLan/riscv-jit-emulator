# a02_4 / 01_mip_mie_basic
# mie / mip / sip 基本读写 + mie WARL 截断 验证 (T2: csr 入口 + 字段就位).
#
# BARE M-mode 启动 (priv=M, satp=0); 不切 S, 不依赖 mret / medeleg.
# T2 阶段 IRQ 永远不来; mip 异步源 (CLINT.msip=0 / mtime=0 / mtimecmp=UINT64_MAX
# clint_init 初值) + _mip_sw=0 → 合成 mip 应为 0.
#
# 期望最终寄存器:
#   a0 (x10) = 0x00000288   csrw mie set (M_TIMER=128 | M_SOFT=8 | S_EXT=512), 直读 mie
#   a1 (x11) = 0x00000000   csrr mip 合成 (无源 + 无 inject)
#   a2 (x12) = 0x00000000   csrr sip 合成 mask view (空)
#   a3 (x13) = 0x00000AAA   csrw mie set 0xFFFFFFFF → MIE_VALID_MASK 截 (bit 1|3|5|7|9|11)
#
# halt 路径: ecall (M-mode, mtvec=0 未设 → pc=0 inst access fault → triple trap halt).

.section .text
.option norvc
.globl _start
_start:
    # --- 1. csrw mie 设三位 (IRQ_M_TIMER=7 | IRQ_M_SOFT=3 | IRQ_S_EXT=9) ---
    li      t0, (1 << 7) | (1 << 3) | (1 << 9)   # = 0x288
    csrw    mie, t0
    csrr    a0, mie                              # a0 = 0x288

    # --- 2. csrr mip — T2 阶段无 IRQ 源 + _mip_sw=0 → 0 ---
    csrr    a1, mip                              # a1 = 0

    # --- 3. csrr sip — 同形态合成 + mask, 0 ---
    csrr    a2, sip                              # a2 = 0

    # --- 4. csrw mie 全 1 验证 WARL 截 (MIE_VALID_MASK = bit 1|3|5|7|9|11) ---
    li      t0, 0xFFFFFFFF
    csrw    mie, t0
    csrr    a3, mie                              # a3 = 0xAAA

    ecall                                         # halt
