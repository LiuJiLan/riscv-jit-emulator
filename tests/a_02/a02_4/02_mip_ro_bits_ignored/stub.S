# a02_4 / 02_mip_ro_bits_ignored
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# csrw mip 写 RO 位 (MSIP/MTIP/MEIP) 验证 — RO 位被 MIP_SW_WRITABLE_MASK 截掉,
# 不影响 _mip_sw 任何 bit; csrr mip 合成时 RO 位仍来自异步源 (clint_init 后初值 0).
# (注: 改 test_dev 停机, 旧叙事见 REVIEW B.3)
#
# ===== 思路 =====
# BARE M-mode; 不切 S.

.section .text
.option norvc
.globl _start
_start:
    # csrw mip 写 RO 位 (IRQ_M_SOFT=3 | IRQ_M_TIMER=7 | IRQ_M_EXT=11)
    li      t0, (1 << 3) | (1 << 7) | (1 << 11)   # = 0x888 (MSIP|MTIP|MEIP)
    csrw    mip, t0

    # csrr mip 合成: _mip_sw=0 (csrw 写 RO 位被 MIP_SW_WRITABLE_MASK 截; _mip_sw
    # 未被影响) + 异步源 (CLINT.msip=0 / mtime<mtimecmp=UINT64_MAX) → 0
    csrr    a0, mip                                # a0 = 0

    # ---- 停机: test_dev FINISHER PASS ----
    li      x30, 0x100000                          # test_dev base
    li      x31, 0x5555                            # FINISHER PASS magic
    sw      x31, 0(x30)                            # 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望最终寄存器:
#   a0 (x10) = 0x00000000   csrw mip set MSIP|MTIP|MEIP → 全 RO 忽略, csrr mip = 0
#   x30 = 0x100000 / x31 = 0x5555  (停机 scratch)
#   停机 = test_dev FINISHER PASS → exit 0
