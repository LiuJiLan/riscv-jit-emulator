# a02_4 / 03_mip_sw_writable
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# csrw mip 写软件位 (SSIP/STIP/SEIP_sw) — MIP_SW_WRITABLE_MASK 三位生效, csrr mip
# 合成读看见软件 inject 位 set (异步源仍 0).
# (注: 改 test_dev 停机, 旧叙事见 REVIEW B.3)
#
# ===== 思路 =====
# BARE M-mode.

.section .text
.option norvc
.globl _start
_start:
    # csrw mip 写软件位 (IRQ_S_SOFT=1 | IRQ_S_TIMER=5 | IRQ_S_EXT=9)
    li      t0, (1 << 1) | (1 << 5) | (1 << 9)   # = 0x222 (SSIP|STIP|SEIP_sw)
    csrw    mip, t0

    # csrr mip 合成: _mip_sw 三位都 set + 异步源 0 → 0x222
    csrr    a0, mip                              # a0 = 0x222

    # ---- 停机: test_dev FINISHER PASS ----
    li      x30, 0x100000                        # test_dev base
    li      x31, 0x5555                          # FINISHER PASS magic
    sw      x31, 0(x30)                           # 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望最终寄存器:
#   a0 (x10) = 0x00000222   csrw mip set (S_SOFT=2 | S_TIMER=32 | S_EXT=512) 全部生效
#   x30 = 0x100000 / x31 = 0x5555  (停机 scratch)
#   停机 = test_dev FINISHER PASS → exit 0
