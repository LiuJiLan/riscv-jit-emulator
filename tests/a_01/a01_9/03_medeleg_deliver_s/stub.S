# a_01_9 / 03_medeleg_deliver_s
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
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
#      fault_cause = INST 类; medeleg.bit 不含 → deliver M → 跳 mtvec=m_handler
#   6. m_handler/halt: test_dev FINISHER PASS 干净停机 (priv=M)
#
# 关键验证点:
#   - x10 = 9 → medeleg deliver S 真生效 (deliver_priv 按 medeleg.bit 派发, 不是
#     hard-coded PRIV_M; 后者会让第 1 次 ecall_from_S 直接 deliver M, scause 不写, x10=0)
#   - x11 = 1 → 真切到 S 后 S phase 跑过
#
# 反例 (deliver_priv 仍 hard-coded PRIV_M, 用 medeleg 派发未实装时):
#   - x10 = 0 (scause 没写; 第 1 次 ecall_from_S 直接 deliver M, 没走 [PRIV_S] 槽)
#   这种情况能立即识别 medeleg 真接的回归 bug (核心区分点是 x10: 真生效=9 / 未实装=0)。

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
    li   x30, 0x100000              # test_dev base
    li   x31, 0x5555                # FINISHER PASS magic
    sw   x31, 0(x30)                # 干净停机: test_dev 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望 main dump:
#   x10 = 9              (scause; deliver S 写 [PRIV_S] 槽 — 核心验证点)
#   x11 = 1              (S phase 标记)
#   x12 = s_target ecall PC (sepc 第 1 次 deliver 写; 绝对值随 codegen 漂, 不钉死)
#   x30 = 0x100000       (停机用 scratch: test_dev base)
#   x31 = 0x5555         (停机用 scratch: FINISHER PASS magic)
#   priv = 3 (M)         (s_handler 末 ecall → deliver S → stvec=0 fetch fail → INST 类
#                          deliver M → m_handler 干净停机, 故 halt 时 priv=M)
#   停机 = test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
# (注: 绝对 pc / total_count 随 test_dev 收尾漂, 见 REVIEW B.3 段; 不在断言内。)
