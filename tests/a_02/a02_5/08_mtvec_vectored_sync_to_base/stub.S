# a02_5 / 08_mtvec_vectored_sync_to_base
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# mtvec mode=1 vectored 时 sync exception 仍走 base (RV Priv Spec §3.1.7):
#   "When MODE=Vectored, all synchronous exceptions ... cause the pc to be set to the
#    address in the BASE field, whereas interrupts cause the pc to be set to the
#    address in the BASE field plus four times the interrupt cause number."
#
# ===== 思路 =====
#   1. mtvec = vec_table | 1 (mode 1)
#   2. ecall (cause = 11 ECALL_FROM_M, sync exception)
#   3. trap_set_exception_state 内 jump-to = xtvec & ~0x3 = base (mask 掉 mode bit) → slot 0
#   4. slot 0 = `j sync_handler` (vectored 下 sync 落点)
#   5. sync_handler 验 mcause = 11, 设 s0=1, j halt
#   6. halt: mv a0,s1 (导出 mcause) → test_dev 干净停机
#
# 跟 fixture 07 区别: fixture 07 验 async interrupt 跳 base + 4*cause; 本 fixture 验 sync
# exception 跳 base (mode bit 在 set_exception_state 内自己 mask 不算 cause).
#
# 实现验证: src/core/trap.c::trap_set_exception_state 末段 `hart->regs[0] = xtvec[deliver_priv]
# & ~0x3u` 是核心 (D5 修 jump-to xtvec mask bug + D9 csr_*tvec_write WARL 解绑后必要的 mask).

.section .text
.option norvc
.globl _start
_start:
    # 清 mstatush.MDT (Smdbltrp reset=1; 否则下方首个 ecall 进 M trap 直接 MDT abort,
    # sync_handler 跑不到) —— 本 fixture 的 trap 进 M (medeleg=0), 故必须 prelude 清 MDT
    csrw    0x310, x0

    # mtvec = vec_table | 1 (mode 1 vectored)
    la      t0, vec_table
    ori     t0, t0, 1
    csrw    mtvec, t0

    # 不启用 interrupts (避免被 async fire 干扰; 本 fixture 只看 sync exception 路径)

    ecall                        # cause=11 sync. vectored mode 下 pc = base (slot 0)

.align 2
vec_table:
    j       sync_handler         # slot 0  (sync exception 落点 — 关键验证点)
    j       halt                 # slot 1
    j       halt                 # slot 2
    j       halt                 # slot 3
    j       halt                 # slot 4
    j       halt                 # slot 5
    j       halt                 # slot 6
    j       halt                 # slot 7  (M_TIMER cause; 若 sync 错落 base+4*11 不在表内但
                                  #          这里走也算 bug 兜底; 实际 cause=11 → slot 11)
    j       halt                 # slot 8
    j       halt                 # slot 9
    j       halt                 # slot 10
    j       halt                 # slot 11 (若 sync 错按 cause=11 当 async 跳 base+44 → 走这里;
                                  #          走到 = bug)

sync_handler:
    csrr    s1, mcause           # 期望 11 (ECALL_FROM_M; 无 CAUSE_INTERRUPT_BIT 高位)
    li      s0, 1
    j       halt

halt:
    mv      a0, s1               # a0 = mcause (sync_handler 跑过 → 11; 跑错 slot → 0)
    # ---- test_dev FINISHER PASS 干净停机 (scratch x30/x31) ----
    li      x30, 0x100000        # test_dev base
    li      x31, 0x5555          # FINISHER PASS magic
    sw      x31, 0(x30)          # 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望最终寄存器:
#   a0 = 11           (sync exception mcause; sync_handler 跑过证明 sync → base 不是 base + 44)
#   s0 = 1            (sync_handler 真跑过)
#   s1 = 11
#   x30 = 0x100000 / x31 = 0x5555 (停机用 scratch: test_dev base / FINISHER PASS magic)
#   停机 = test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
#
# 关键验证:
#   - sync exception (ecall) cause=11; 若 vectored 模式错把 sync 当 async 走 base + 4*11,
#     pc 落 slot 11 = `j halt`, sync_handler 不跑 → s0=0 / a0=0
#   - 正确行为: sync → base = slot 0 = sync_handler → s0=1 / a0=11
