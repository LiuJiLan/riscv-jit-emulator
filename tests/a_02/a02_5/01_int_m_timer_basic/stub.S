# a02_5 / 01_int_m_timer_basic
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
# RUN-TSAN-SKIP
#
# ===== TSan SKIP 原因 =====
# Tsan run timer cond_wait + atomic_fetch_add 推进 mtime 显著慢于 Debug — Debug
# 下 ~22ms 跑完, Tsan 24s timeout 仍卡在 wait_loop. 单 hart fixture 无 cross-hart
# race verify 目的, timer thread 跟主 hart 同步走 atomic_load_explicit acquire,
# 已有 a03_7/04_wfi_mtime_write_wake 等 fixture 通过 Tsan, race-correctness 已被
# 别处 cover. 本 fixture 是 M-mode timer fire 路径验, Debug+ASan 验功能正确性即可.
#
# ===== 测试目的 =====
# M-mode timer interrupt 基本路径 — 端到端验证 (假设 mtime 在主动步进)。
#
# ===== 思路 =====
#   1. mtime 推进 (实现方式取决于当前阶段, 见 platform/clint.h 顶段; 行为受步进
#      实现影响, 但 mip.MTIP 合成读 + trap 路径语义不变)
#   2. trap_check_interrupt 看到 mip.MTIP (clint_timer_pending) & mie.MTIE & mstatus.MIE → fire
#   3. trap_set_interrupt_state: deliver M (mideleg.MTIE=0), xcause = CAUSE_INTERRUPT_BIT | IRQ_M_TIMER
#      = 0x80000007; jump-to xtvec base (mode=0 direct)
#   4. m_handler: 验 mcause, 清 MTIP via 写 mtimecmp = UINT64_MAX, mret
#   5. main 看 s0=1 → 跳出 wait_loop → halt path

.section .text
.option norvc
.globl _start
_start:
    # ---- M-mode 起步 ----
    # 设 mtvec = m_handler (mode=0 direct; D9 解绑后 mode=1 vectored 也支持, 但这条
    # fixture 用 direct, vectored 形态由 fixture 之外的内核单测覆盖)
    la      t0, m_handler
    csrw    mtvec, t0

    # mie.MTIE = (1<<7) = 0x80
    li      t0, 0x80
    csrw    mie, t0

    # mtimecmp[0] = 50 (小值, mtime 推进几轮后超过, MTIP fire)
    li      t0, 0x02004000          # CLINT mtimecmp[0] 低 32
    li      t1, 50
    sw      t1, 0(t0)
    sw      x0, 4(t0)               # 高 32 = 0

    # mstatus.MIE = 1 (bit 3); csrrsi 立即数 0x8 = (1<<3)
    csrw    0x310, x0               # mstatush 清 MDT (A.7 prelude; 否则下条 MIE 写被强制 0)
    csrrsi  x0, mstatus, 0x8

    # 等待 timer interrupt fire 的 busy loop (handler 将设 s0=1 退出)
    li      s0, 0
    li      a5, 0
wait_loop:
    addi    a5, a5, 1               # 累加器 (验 wait_loop 真跑了几轮)
    beqz    s0, wait_loop

    # ---- handler 返回后 ----
    mv      a0, s1                  # a0 = handler 保存的 mcause (期望 0x80000007)

    # ---- halt: test_dev FINISHER PASS 干净停机 (scratch x30/x31) ----
    li      x30, 0x100000           # test_dev base
    li      x31, 0x5555             # FINISHER PASS magic
    sw      x31, 0(x30)             # 置 exit_code=0 + SRS=0, dispatcher 退主循环
1:  j       1b                      # 防 fall-through: SRS=0 下个 block 边界退

m_handler:
    # 保存 mcause 待 main 验证 (期望 0x80000007 = CAUSE_INTERRUPT_BIT | IRQ_M_TIMER)
    csrr    s1, mcause

    # 清 MTIP via 写 mtimecmp = UINT64_MAX (跟 clint_init sentinel 一致)
    li      t0, 0x02004000
    li      t1, -1                  # 0xFFFFFFFF
    sw      t1, 0(t0)
    sw      t1, 4(t0)

    li      s0, 1                   # 完成标志
    mret

# ===== 期望结果 =====
# 期望最终寄存器:
#   a0 (x10) = 0x80000007   (handler s1 → main a0; CAUSE_INTERRUPT_BIT | IRQ_M_TIMER)
#   s0 (x8)  = 1            (handler 设置, 验证 handler 真跑)
#   s1 (x9)  = 0x80000007   (handler 保存 mcause)
#   a5 (x15) > 0            (wait_loop 至少跑一轮)
#   x30 = 0x100000 / x31 = 0x5555 (停机用 scratch: test_dev base / FINISHER PASS magic)
#   停机 = test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
# (注: 绝对 pc / total_count 随 test_dev 收尾漂, 见 REVIEW B.3 段; 不在断言内。)
