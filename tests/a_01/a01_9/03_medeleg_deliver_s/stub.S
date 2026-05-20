# a_01_9 / 03_medeleg_deliver_s
#
# ===== 测试目的 =====
# 验 medeleg delegate ecall_from_S 给 S handler 真生效 (deliver_priv 按 medeleg.bit
# 派发, 不是 hard-coded PRIV_M)。
#
# ===== 思路 =====
# 模板: .S + .c (跟 a_01_8 fixture 02/03 同; 需要 setup_pt 建 SV32 PT)。
#
# PT layout: root_pt[0x200] = 0x200000CF (4MB superpage leaf identity, 跟 fixture 02 同)。
#
# 控制流:
#   1. M setup:
#      sp + mtvec → m_handler + stvec → s_handler + medeleg = (1<<9) (delegate ecall_from_S)
#      call setup_pt + csrw satp + sfence + mret 切 S
#   2. S phase: addi x11, x0, 1 (S phase 标记) + ecall (cause=9 ecall_from_S)
#   3. trap_set_state 内: hart->priv=S + cause=9 + medeleg.bit(9)=1 → deliver_priv=S
#      → 写 [PRIV_S] 槽 (scause=9, sepc=s_target ecall PC) + 写 SPP/SPIE/SIE → 跳 stvec=s_handler
#   4. S handler:
#      csrr x10, scause (期望 9; 验 medeleg deliver S 真写 [PRIV_S] 槽)
#      csrr x12, sepc   (期望 s_target ecall PC)
#      csrw stvec, x0   (清 stvec 让下次 deliver S 失败)
#      ecall (cause=9; medeleg.bit(9)=1 仍 deliver S; in_trap 1→2; 跳 stvec=0)
#   5. fetch from 0 fail (mmu_translate_pc walker walk vpn1=0, root_pt[0]=0 V=0):
#      fault_cause = INST_PAGE_FAULT (12); trap_set_state(12, 0); in_trap 2→3 早 return,
#      不 deliver M, 不切 priv, 不写字段; dispatcher 看 in_trap=3 break
#   6. m_handler 走不到 (m_handler 段为空, 跟 halt label 同地址但永不执行)
#
# 关键验证点:
#   - x10 = 9 → medeleg deliver S 真生效 (deliver_priv 按 medeleg.bit 派发, 不是
#     hard-coded PRIV_M; 后者会让 mcause=9, scause=0)
#   - priv = 1 (PRIV_S) → 真切到 S 后保持
#   - mcause = 0 → deliver M 路径从未走过 (hard-coded PRIV_M 形态下 mcause=9)
#
# 反例 (deliver_priv 仍 hard-coded PRIV_M, 用 medeleg 派发未实装时):
#   - x10 = 0 (scause 没写)
#   - mcause = 9 (ecall_from_S 直接 deliver M)
#   - priv = 3 (M; 没切 S)
#   这种情况能立即识别 medeleg 真接的回归 bug。

.equ CSR_TOHOST, 0x800
.equ CSR_PRIVRD, 0xCC0

.section .text
.option norvc
.globl _start
_start:
    # ---- 设 sp ----
    lui  sp, 0x80300

    # ---- 设 M handler ----
    la   x4, m_handler
    csrw mtvec, x4

    # ---- 设 S handler ----
    la   x4, s_handler
    csrw 0x105, x4                  # stvec

    # ---- 设 medeleg = (1<<9) — delegate ecall_from_S 给 S ----
    addi x4, x0, 1
    slli x4, x4, 9                  # x4 = 0x200 = 1 << 9
    csrw 0x302, x4                  # medeleg

    # ---- C 函数建 PT (跟 fixture 02 同 — 4MB superpage identity) ----
    call setup_pt

    # ---- 设 satp: MODE=Sv32 + PPN=0x80100 ----
    lui  x6, 0x80080
    addi x6, x6, 0x100              # satp = 0x80080100
    csrw satp, x6
    sfence.vma

    # ---- 设 mret 目标 = s_target identity vaddr ----
    la   x7, s_target
    csrw mepc, x7

    # ---- 设 mstatus.MPP = PRIV_S ----
    addi x4, x0, 1
    slli x4, x4, 11                 # x4 = 0x800
    csrw mstatus, x4

    mret                            # M→S

s_target:
    # ---- S phase ----
    addi x11, x0, 1                 # S phase 标记
    ecall                           # cause=9 ecall_from_S; 期望 medeleg deliver S handler

s_handler:
    # ---- S handler (medeleg deliver S 真生效时跑这里) ----
    csrr x10, 0x142                 # scause; 期望 9
    csrr x12, 0x141                 # sepc; 期望 s_target ecall PC
    csrw 0x105, x0                  # stvec = 0; 让下次 deliver S 失败
    ecall                           # cause=9 → deliver S → 跳 stvec=0 → fetch fail →
                                    # cause=1 INST_ACCESS_FAULT → priv=S, medeleg.bit(1)=0
                                    # → deliver M → 跳 mtvec=m_handler

m_handler:
halt:
    csrw mtvec, x0
    ecall                           # cause=11 from M, mtvec=0 fetch fail → triple fault

# ===== 期望结果 =====
# 期望 main dump:
#   x10 = 9              (scause; deliver S 写 [PRIV_S] 槽 — 核心验证点)
#   x11 = 1              (S phase 标记)
#   x12 = 0x8000005C     (s_target ecall PC; sepc 第 1 次 deliver 写)
#   mcause = 0           (deliver M 路径从未走过, mcause 保持初值 0)
#   mtval  = 0           (同上)
#   mepc   = 0           (同上)
#   mtvec  = 0x80000070  (_start 设过, 后续没改; m_handler 地址 = halt label)
#   priv = 1 (PRIV_S)    (第 2 次 deliver S 后 priv=S; 第 3 次早 return 不变)
#   in_trap = 3
