# a02_5 / 07_mtvec_vectored_m_timer
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# mtvec mode=1 vectored, M timer interrupt fire → pc = base + 4 * IRQ_M_TIMER (= base + 28)。
# 跟 a02_5/01 区别: mtvec mode=1 (vectored) vs mode=0 (direct); 验 trap_set_interrupt_state
# vectored 分支 (base + 4*cause_low) 而不是 direct (base).
#
# ===== 思路 =====
#   1. mtvec = vec_table | 1 (mode 1)
#   2. mtimecmp = 50 (mtime 推进过 50 后 MTIP fire)
#   3. mie.MTIE + mstatus.MIE
#   4. timer fire → trap_set_interrupt_state 看 xtvec.mode=1 → pc = base + 4*7 = slot 7
#   5. slot 7 内 `j timer_handler` → 跳真 handler
#   6. timer_handler 验 mcause = 0x80000007, 清 MTIP, mret
#   7. main 看 s0=1 → halt path

.section .text
.option norvc
.globl _start
_start:
    # mtvec = vec_table | 1 (mode 1 vectored); csr_mtvec_write WARL 接受 mode=0/1
    la      t0, vec_table
    ori     t0, t0, 1
    csrw    mtvec, t0

    # mie.MTIE = 0x80
    li      t0, 0x80
    csrw    mie, t0

    # mtimecmp[0] = 50 (跟 fixture 01 同; setup 完后 mtime 推进过 50 才 fire)
    li      t0, 0x02004000
    li      t1, 50
    sw      t1, 0(t0)
    sw      x0, 4(t0)

    # init s0/a5 在 csrrsi 之前 (fixture 06 模板; 防 fire 提前进 init 段死循)
    li      s0, 0
    li      a5, 0

    csrw    0x310, x0               # mstatush 清 MDT (A.7 prelude; 否则下条 MIE 写被强制 0)
    csrrsi  x0, mstatus, 0x8     # mstatus.MIE = 1

wait_loop:
    addi    a5, a5, 1
    beqz    s0, wait_loop

    mv      a0, s1               # a0 = mcause from timer_handler

    # ---- halt: test_dev FINISHER PASS 干净停机 (scratch x30/x31) ----
    li      x30, 0x100000        # test_dev base
    li      x31, 0x5555          # FINISHER PASS magic
    sw      x31, 0(x30)          # 置 exit_code=0 + SRS=0, dispatcher 退主循环
1:  j       1b                      # 防 fall-through: SRS=0 下个 block 边界退

# vec_table 4 字节对齐 (mtvec.BASE 要求); 12 slot 覆盖 IRQ_*_EXT 最大位号 11
.align 2
vec_table:
    j       halt                 # slot 0  (sync exception 在 vectored 下不该走这里; 防御)
    j       halt                 # slot 1  (IRQ_S_SOFT)
    j       halt                 # slot 2
    j       halt                 # slot 3  (IRQ_M_SOFT)
    j       halt                 # slot 4
    j       halt                 # slot 5  (IRQ_S_TIMER)
    j       halt                 # slot 6
    j       timer_handler        # slot 7  (IRQ_M_TIMER) — 验证点: pc = base + 28
    j       halt                 # slot 8
    j       halt                 # slot 9  (IRQ_S_EXT)
    j       halt                 # slot 10
    j       halt                 # slot 11 (IRQ_M_EXT)

timer_handler:
    csrr    s1, mcause           # 期望 0x80000007 (CAUSE_INTERRUPT_BIT | IRQ_M_TIMER)

    li      t0, 0x02004000
    li      t1, -1
    sw      t1, 0(t0)
    sw      t1, 4(t0)

    li      s0, 1
    mret

halt:
    # 走到这里 = vectored 跳错 slot (不该发生; bug 兜底)。
    # 兜底也走 test_dev FINISHER PASS, 但跑错 slot 会留 s0=0/a0=0 暴露 bug。
    li      x30, 0x100000
    li      x31, 0x5555
    sw      x31, 0(x30)

# ===== 期望结果 =====
# 期望最终寄存器:
#   a0 = 0x80000007   (mcause from M_TIMER; vectored 跳 slot 7 → timer_handler 真跑过)
#   s0 = 1
#   s1 = 0x80000007
#   a5 > 0            (wait_loop 至少跑一轮)
#   x30 = 0x100000 / x31 = 0x5555 (停机用 scratch: test_dev base / FINISHER PASS magic)
#   停机 = test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
#
# 关键验证:
#   - timer_handler 真跑 (s0=1 + s1=0x80000007) → vectored mode 跳到 base + 4*7 而不是 base
#   - 若走了 base (mode 0 direct), 进 slot 0 = `j halt` → s0/s1 都 0; a0 = 0
