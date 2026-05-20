# a02_4 / 04_sip_view_consistent
#
# ===== 测试目的 =====
# sip 入口验证 — mip / sip 看同物理 _mip_sw 字段; sip 内 SSIP RW, STIP / SEIP RO
# (RV spec §5.1.4 + §3.1.9, SIP_WRITABLE_MASK = 1<<IRQ_S_SOFT).
#
# ===== 思路 =====
# 全程 BARE M-mode (M priv 数字 >= S, 可直接访问 sip 入口 — sip CSR 编号 priv
# 段 = S 但 priv 数字大可访问). 不走 mret 切 S, 简化 fixture (跟 a_01 fixture
# 多次切 priv 比, 本 fixture 焦点在跨入口 view consistency, 不在 priv 切换).

.section .text
.option norvc
.globl _start
_start:
    # --- 1. csrw sip 设 SSIP (RW in sip) ---
    li      t0, (1 << 1)                          # IRQ_S_SOFT bit
    csrw    sip, t0
    csrr    a0, sip                               # a0 = 2 (SSIP)

    # --- 2. csrr mip 跨入口 view ---
    csrr    a1, mip                               # a1 = 2 (同物理 _mip_sw bit 1)

    # --- 3. csrs sip 试 set STIP (RO in sip; SIP_WRITABLE_MASK 不含 STIP) ---
    li      t0, (1 << 5)                          # IRQ_S_TIMER bit
    csrrs   zero, sip, t0                         # OR (1<<5) 进 sip, 但 STIP 被截
    csrr    a2, sip                               # a2 = 2 (SSIP 不变, STIP 不 set)

    # --- 4. csrs sip 试 set SEIP (RO in sip) ---
    li      t0, (1 << 9)                          # IRQ_S_EXT bit
    csrrs   zero, sip, t0
    csrr    a3, sip                               # a3 = 2 (SEIP 不 set)

    # --- 5. csrc sip 清 SSIP (RW) ---
    li      t0, (1 << 1)
    csrrc   zero, sip, t0
    csrr    a4, sip                               # a4 = 0

    ecall

# ===== 期望结果 =====
# 期望最终寄存器:
#   a0 (x10) = 0x00000002   csrw sip set SSIP → _mip_sw bit 1 = 1; csrr sip = 2
#   a1 (x11) = 0x00000002   csrr mip 跨入口看见 _mip_sw bit 1 (同物理字段)
#   a2 (x12) = 0x00000002   csrs sip 试 set STIP (RO in sip), 不生效, sip 仍 2
#   a3 (x13) = 0x00000002   csrs sip 试 set SEIP (RO in sip), 不生效, sip 仍 2
#   a4 (x14) = 0x00000000   csrc sip 清 SSIP (RW), sip 变 0
#
# halt: ecall.
