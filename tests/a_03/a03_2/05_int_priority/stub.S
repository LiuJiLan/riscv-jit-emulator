# a03_2 / 05_int_priority
#
# ===== 测试目的 =====
# 完整 6-source 中断优先级 (RV Priv Spec §3.1.9 + U74-MC §8.9):
#   M_EXT > M_SOFT > M_TIMER > S_EXT > S_SOFT > S_TIMER
# 同时 set 6 个 mip bit (MEIP + MSIP + MTIP + SEIP_sw + SSIP + STIP), mideleg=0
# (全 M deliver), 每次 fire 后 handler 清当前最高源 + 记 mcause + mret, 验下次 fire
# 是次高优先级.
#
# 类 a_02/a02_5/06_int_priority (它只测 M_SOFT > M_TIMER 2-case), 此处完整 6-case
# + 加 MEIP/SEIP (PLIC + 软件 SEIP_sw 合成) 验顺序。
#
# 6 源 set 方式:
#   - MEIP via PLIC: priority[3]=1, threshold=0, enable bit3, csrw 0x800 = 0x80000003
#   - MSIP via CLINT: msip[0] = 1
#   - MTIP via CLINT: mtimecmp[0] = 5 (timer thread 第 1 wake 即推过)
#   - SEIP_sw / SSIP / STIP via csrw mip (走 csr_mip_write 的 MIP_SW_WRITABLE_MASK
#     = SIP_MASK 子集; bit 1=SSIP, bit 5=STIP, bit 9=SEIP_sw)
#
# 6 源清法 (handler 内按 mcause dispatch):
#   - clear MEIP: PLIC claim+complete + csrw 0x800 = 3
#   - clear MSIP: msip[0] = 0
#   - clear MTIP: mtimecmp[0] = UINT64_MAX
#   - clear SEIP: csrw mip, (SSIP|STIP) = 0x22 (保留 SSIP STIP, 清 SEIP_sw)
#   - clear SSIP: csrw mip, STIP = 0x20 (清 SSIP)
#   - clear STIP: csrw mip, 0 (清 STIP)
#
# ===== 期望结果 =====
#   s4 = 0x8000000B (cause 11 = M_EXT) — iter 1
#   s5 = 0x80000003 (cause 3  = M_SOFT) — iter 2
#   s6 = 0x80000007 (cause 7  = M_TIMER) — iter 3
#   s7 = 0x80000009 (cause 9  = S_EXT) — iter 4
#   s8 = 0x80000001 (cause 1  = S_SOFT) — iter 5
#   s9 = 0x80000005 (cause 5  = S_TIMER) — iter 6
#   s3 = 6 (handler 进入次数)
#   s0 = 1 (退出 wait_loop)

.equ CSR_TOHOST, 0x800
.section .text
.option norvc
.globl _start

_start:
    la      t0, m_handler
    csrw    mtvec, t0

    # mie = MIE_VALID_MASK = (1<<1)|(1<<3)|(1<<5)|(1<<7)|(1<<9)|(1<<11) = 0xAAA
    li      t0, 0xAAA;  csrw mie, t0

    # mideleg = 0 (不 delegate, 6 源全走 M handler)
    csrw    mideleg, x0

    # ---- CLINT setup: mtimecmp=5, msip=1 ----
    li      t0, 0x02004000;  li t1, 5;  sw t1, 0(t0);  sw x0, 4(t0)
    li      t0, 0x02000000;  li t1, 1;  sw t1, 0(t0)

    # ---- PLIC setup: priority[3]=1, threshold[ctx0]=0, enable bit3 ----
    li      t0, 0x0C000000;  li t1, 1;  sw t1, 12(t0)
    li      t0, 0x0C200000;  sw x0, 0(t0)
    li      t0, 0x0C002000;  li t1, 0x8;  sw t1, 0(t0)

    # ---- init counters / GPR slots (必须在 fire 前) ----
    li      s0, 0       # wait_loop 退出 flag
    li      s3, 0       # handler 进入计数
    li      s4, 0
    li      s5, 0
    li      s6, 0
    li      s7, 0
    li      s8, 0
    li      s9, 0

    # ---- set MEIP via CSR_TOHOST (PLIC src=3 device_set) ----
    li      t0, 0x80000003;  csrw CSR_TOHOST, t0

    # ---- set SEIP_sw / SSIP / STIP 共 0x222 via csrw mip ----
    li      t0, 0x222;  csrw mip, t0

    # ---- 开 MIE → 下一 block trap_check fire 最高 (= M_EXT) ----
    csrrsi  x0, mstatus, 0x8

wait_loop:
    beqz    s0, wait_loop

    csrw    mtvec, x0
    ecall                                  # halt

# ============================================================================
# m_handler — dispatch 6 cases, 每次清当前 cause 对应源 + 记 cause + mret
# ============================================================================
m_handler:
    csrr    t0, mcause                     # t0 = cause | 0x80000000
    addi    s3, s3, 1                      # 进入次数 ++

    # ---- 按 s3 分发 mcause 到 s4..s9 ----
    li      t1, 1;  beq s3, t1, st_iter1
    li      t1, 2;  beq s3, t1, st_iter2
    li      t1, 3;  beq s3, t1, st_iter3
    li      t1, 4;  beq s3, t1, st_iter4
    li      t1, 5;  beq s3, t1, st_iter5
    li      t1, 6;  beq s3, t1, st_iter6
    j       handler_exit                   # >6 防御

st_iter1:  mv s4, t0;  j dispatch_clear
st_iter2:  mv s5, t0;  j dispatch_clear
st_iter3:  mv s6, t0;  j dispatch_clear
st_iter4:  mv s7, t0;  j dispatch_clear
st_iter5:  mv s8, t0;  j dispatch_clear
st_iter6:  mv s9, t0;  j dispatch_clear

dispatch_clear:
    # cause_low = mcause & 0x7FFFFFFF
    li      t1, 0x7FFFFFFF
    and     t1, t0, t1

    li      t2, 11;  beq t1, t2, clear_meip
    li      t2, 3;   beq t1, t2, clear_msip
    li      t2, 7;   beq t1, t2, clear_mtip
    li      t2, 9;   beq t1, t2, clear_seip
    li      t2, 1;   beq t1, t2, clear_ssip
    li      t2, 5;   beq t1, t2, clear_stip
    j       handler_exit

clear_meip:
    li      t2, 0x0C200004
    lw      t3, 0(t2)
    sw      t3, 0(t2)
    li      t2, 3;  csrw CSR_TOHOST, t2
    j       check_done

clear_msip:
    li      t2, 0x02000000;  sw x0, 0(t2)
    j       check_done

clear_mtip:
    li      t2, 0x02004000;  li t3, -1
    sw      t3, 0(t2);  sw t3, 4(t2)
    j       check_done

clear_seip:
    # 保留 SSIP/STIP, 清 SEIP_sw
    li      t2, 0x22;  csrw mip, t2
    j       check_done

clear_ssip:
    # 保留 STIP, 清 SSIP. SEIP_sw 已清.
    li      t2, 0x20;  csrw mip, t2
    j       check_done

clear_stip:
    csrw    mip, x0
    j       check_done

check_done:
    li      t2, 6
    beq     s3, t2, all_done
    mret

all_done:
    li      s0, 1
    mret

handler_exit:
    li      s0, 1
    mret
