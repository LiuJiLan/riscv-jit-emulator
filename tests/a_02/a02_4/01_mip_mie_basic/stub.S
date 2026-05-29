# a02_4 / 01_mip_mie_basic
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# mie / mip / sip 基本读写 + mie WARL 截断 验证 (csr 入口 + 字段就位).
# (注: 改 test_dev 停机, 旧叙事见 REVIEW B.3)
#
# ===== 思路 =====
# BARE M-mode 启动 (priv=M, satp=0); 不切 S, 不依赖 mret / medeleg.
# 本 fixture IRQ 永远不来 — mip 异步源 (CLINT.msip=0 / mtimecmp=UINT64_MAX
# clint_init 初值 → MTIP=0 即使 mtime 由 timer 累加也极远不到 UINT64_MAX) +
# _mip_sw=0 → 合成 mip 应为 0.

.section .text
.option norvc
.globl _start
_start:
    # --- 1. csrw mie 设三位 (IRQ_M_TIMER=7 | IRQ_M_SOFT=3 | IRQ_S_EXT=9) ---
    li      t0, (1 << 7) | (1 << 3) | (1 << 9)   # = 0x288
    csrw    mie, t0
    csrr    a0, mie                              # a0 = 0x288

    # --- 2. csrr mip — 无 IRQ 源 + _mip_sw=0 → 0 ---
    csrr    a1, mip                              # a1 = 0

    # --- 3. csrr sip — 同形态合成 + mask, 0 ---
    csrr    a2, sip                              # a2 = 0

    # --- 4. csrw mie 全 1 验证 WARL 截 (MIE_VALID_MASK = bit 1|3|5|7|9|11) ---
    li      t0, 0xFFFFFFFF
    csrw    mie, t0
    csrr    a3, mie                              # a3 = 0xAAA

    # ---- 停机: test_dev FINISHER PASS ----
    li      x30, 0x100000                         # test_dev base
    li      x31, 0x5555                           # FINISHER PASS magic
    sw      x31, 0(x30)                           # 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望最终寄存器:
#   a0 (x10) = 0x00000288   csrw mie set (M_TIMER=128 | M_SOFT=8 | S_EXT=512), 直读 mie
#   a1 (x11) = 0x00000000   csrr mip 合成 (无源 + 无 inject)
#   a2 (x12) = 0x00000000   csrr sip 合成 mask view (空)
#   a3 (x13) = 0x00000AAA   csrw mie set 0xFFFFFFFF → MIE_VALID_MASK 截 (bit 1|3|5|7|9|11)
#   x30 = 0x100000 / x31 = 0x5555  (停机 scratch)
#   停机 = test_dev FINISHER PASS → exit 0
