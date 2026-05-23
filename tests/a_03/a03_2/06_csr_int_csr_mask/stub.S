# a03_2 / 06_csr_int_csr_mask
#
# ===== 测试目的 =====
# 中断相关 CSR (mie / mip / sip / sie) 的 WARL mask 行为. 全单元测试, 不开 MIE
# 不触发中断 — 纯 csrr/csrw + GPR dump 验.
#
# mask 模型 (csr.c 实装):
#   mie WARL    — csr_mie_write: _mie = v & MIE_VALID_MASK
#                 (MIE_VALID_MASK = (1<<1)|(1<<3)|(1<<5)|(1<<7)|(1<<9)|(1<<11) = 0xAAA)
#   sie 是 mie 的 mask view (SIE_MASK = SIP_MASK = (1<<1)|(1<<5)|(1<<9) = 0x222):
#     read sie  = _mie & SIE_MASK
#     write sie = _mie = (_mie & ~SIE_MASK) | (v & SIE_MASK)   (只动 SIE_MASK 子集)
#   mip 写 mask — csr_mip_write: _mip_sw = (_mip_sw & ~MIP_SW_WRITABLE_MASK)
#                                        | (v & MIP_SW_WRITABLE_MASK)
#                 (MIP_SW_WRITABLE_MASK = SIP_MASK = 0x222; M_xxx RO 视角忽略)
#   mip 合成读  — csr_mip_read: _mip_sw | clint pending | plic pending
#   sip mask view: read sip = csr_mip_read(hart) & SIP_MASK
#                  write sip = csr_sip_write 走 _mip_sw 的 SIE_MASK 子集 (跟 mip 写
#                              MIP_SW_WRITABLE_MASK 实际一致)
#
# fixture 不动 CLINT/PLIC, 所以 csr_mip_read 合成只 = _mip_sw 部分.
#
# ===== 期望结果 =====
#   s1 = 0xAAA   (csrw mie 0xFFFFFFFF → 读回 = MIE_VALID_MASK)
#   s2 = 0x888   (csrw sie 0 → mie 只清 SIE_MASK 子集, M 位保留: 0xAAA & ~0x222 = 0x888)
#   s3 = 0xAAA   (csrw sie 0x222 → mie 恢复 = 0x888 | 0x222 = 0xAAA)
#   s4 = 0x222   (csrw mie 0xAAA / csrr sie → mie & SIE_MASK = 0x222)
#   s5 = 0x222   (csrw mip 0xFFFFFFFF → _mip_sw 只接受 SIP_MASK = 0x222; csrr mip)
#   s6 = 0x000   (csrw mip 0 → _mip_sw 清; csrr mip)
#   s7 = 0x222   (csrw mip 0x222 → 设 SSIP/STIP/SEIP_sw; csrr mip)
#   s8 = 0x222   (csrr sip 同上 = mip & SIP_MASK)

.section .text
.option norvc
.globl _start

_start:
    # ---- mie WARL: 写全 1, 读回应剩 MIE_VALID_MASK = 0xAAA ----
    li      t0, -1                          # 0xFFFFFFFF
    csrw    mie, t0
    csrr    s1, mie                         # 期望 0xAAA

    # ---- sie write 只动 SIE_MASK 子集: 清 sie → mie M 位保留 ----
    csrw    sie, x0
    csrr    s2, mie                         # 期望 0x888

    # ---- sie write 设 SIE_MASK: mie 恢复 ----
    li      t0, 0x222
    csrw    sie, t0
    csrr    s3, mie                         # 期望 0xAAA

    # ---- sie read = mie & SIE_MASK ----
    csrr    s4, sie                         # 期望 0x222

    # ---- mip write 全 1: _mip_sw 只接受 SIP_MASK (M_xxx RO 忽略) ----
    li      t0, -1
    csrw    mip, t0
    csrr    s5, mip                         # 期望 0x222 (无 clint/plic 合成)

    # ---- mip write 0: 清 _mip_sw ----
    csrw    mip, x0
    csrr    s6, mip                         # 期望 0

    # ---- mip write 0x222: 设 SW 子集 ----
    li      t0, 0x222
    csrw    mip, t0
    csrr    s7, mip                         # 期望 0x222

    # ---- sip read = mip & SIP_MASK ----
    csrr    s8, sip                         # 期望 0x222

    # ---- halt: 不开 MIE (mip 设着也不 fire), 清 mtvec + ecall → triple fault ----
    csrw    mtvec, x0
    ecall
