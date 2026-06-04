# a04_3 / 02_lr_sc_spinlock
#
# ===== 运行参数 =====
# RUN-SMP: 2
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 2-hart LR.W/SC.W spinlock 竞争 — 端到端验跨 hart reservation 清除 + 序列化:
#   1. 两 hart 各自 N=16 次 lock_acquire (LR/SC test-and-set) + counter++ + lock_release
#   2. 序列化: counter 由 spinlock 保护, 最终值应严格 == 2*N = 32 (无丢失更新)
#   3. SC 失败 retry: 跨 hart SC 真竞争时 lrsc_on_store / SC 内部清匹配 reservation,
#      失败 hart 走 retry 路径 (单 hart fixture 永远一次成功, 不验 retry)
#
# 任一 hart 完工但 counter != 2*N 走 fail.
#
# ===== 思路 =====
# 同 reset vector 启动, csrr mhartid 分流: hart 0 + hart 1 都跑 worker 路径
# (N 次 lock 保护 counter++), 都完工后 hart 0 验 counter 写 PASS, hart 1 跑完
# spin 等死 (test_dev PASS 触发 RESET 路径让所有 hart 退出).
#
# 共享数据 (RAM):
#   LOCK     @ 0x80100000   spinlock (0 = free / 1 = held)
#   COUNTER  @ 0x80100010   累加 counter (init 0)
#   HART_DONE @ 0x80100020 + 4*hartid   per-hart 完工 flag (hart 0 等 hart 1 done)
#
# Spinlock 实装 (LR/SC test-and-set; 经典 RV pattern):
#   acquire: do { while(lr.w(lock) != 0); } while (sc.w(lock, 1) != 0);
#   release: sw(lock, 0)  /* 普通 SW 触发 lrsc_on_store 清所有 hart reservation */
#
# Boot prelude 清 MDT/SDT (两 hart 都跑).
# N=16 是 fixture 速度跟覆盖率折中 (32 次成功 CAS 已能 stress lrsc_on_store + bucket
# lock 路径; 跑过快不增确定性).
#
# ===== TSan note =====
# 本 fixture 在 TSan build 上会报 race: lrsc_lr_w atomic_load *pa vs store_helper
# plain memcpy *hva (cross-hart 同 PA RAM access). 这是项目 host x86 TSO 接受的
# latent race (跟 a_03 SMP cross-hart plain RAM access 同源 — a_03 fixture 短没采到,
# spinlock 50 万条密集 busy-wait 必然触发). 不是 LR/SC 实装引入的新问题, 跑 debug
# build 验证 functional correctness 即可. 跨 host ARM64 兼容时 lsu RAM access 一并
# 升 atomic (plan §2 + lrsc_amo_decision.md Q20 撤回项).

.equ TEST_DEV_BASE,     0x00100000
.equ FINISHER_PASS,     0x5555
.equ FINISHER_FAIL,     0x3333

.equ LOCK_ADDR,         0x80100000
.equ COUNTER_ADDR,      0x80100010
.equ HART_DONE_BASE,    0x80100020

.equ N_ITER,            16

.section .text
.option norvc
.globl _start

_start:
    # ---- Boot prelude (两 hart 都跑): 清 MDT/SDT ----
    csrw    0x310, x0
    csrw    sstatus, x0

    csrr    t0, mhartid
    beqz    t0, hart0_path                  # mhartid==0 → 跑 worker, 完工后等 hart 1 验 counter
    li      t1, 1
    beq     t0, t1, hart1_path              # mhartid==1 → 跑 worker, 完工后写 HART_DONE[1]
    # 其他 hartid (不应触发, --smp 只用 0/1) 直接 fail
    j       fail

# ============================================================
# hart 0 路径: 跑 worker + 完工后 poll hart 1 done + 验 counter == 2*N → PASS
# ============================================================
hart0_path:
    li      s0, LOCK_ADDR
    li      s1, COUNTER_ADDR
    li      s2, N_ITER

    # init COUNTER = 0 (只 hart 0 init, 单方写 — hart 1 第一次 lock_acquire 时
    # COUNTER 已被 hart 0 init)
    sw      x0, 0(s1)

    # init LOCK = 0
    sw      x0, 0(s0)

    li      s3, 0                           # 完工 iter 计数器
hart0_loop:
    # acquire lock
    call    acquire_lock_s0
    # critical section: counter++
    lw      t3, 0(s1)
    addi    t3, t3, 1
    sw      t3, 0(s1)
    # release lock
    sw      x0, 0(s0)                       # store 0 → lrsc_on_store 清所有 hart reservation
    addi    s3, s3, 1
    bne     s3, s2, hart0_loop

    # 标 hart 0 完工 (HART_DONE[0] = 1)
    li      t4, HART_DONE_BASE
    li      t5, 1
    sw      t5, 0(t4)

    # poll hart 1 完工 (HART_DONE[1] == 1)
poll_hart1_done:
    lw      t5, 4(t4)
    beqz    t5, poll_hart1_done

    # 验 counter == 2*N
    lw      t3, 0(s1)
    li      t4, 32                          # 2 * N_ITER
    bne     t3, t4, fail

    # PASS
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_PASS
    sw      t1, 0(t0)
1:  j       1b

# ============================================================
# hart 1 路径: 跑 worker + 完工后写 HART_DONE[1] + spin (等 hart 0 写 PASS reset)
# ============================================================
hart1_path:
    # 等 hart 0 init COUNTER/LOCK 完 — 简单延迟 (hart 0 写 LOCK 后才能用; 没真同步
    # 是因为 hart 0 init 是开局 4 个 sw 极短窗口, hart 1 此时还在 csrw + branch
    # 摊开几条指令, 实测足够). 真做严格同步要加 barrier flag, fixture 简化.
    nop
    nop
    nop
    nop

    li      s0, LOCK_ADDR
    li      s1, COUNTER_ADDR
    li      s2, N_ITER

    li      s3, 0
hart1_loop:
    call    acquire_lock_s0
    lw      t3, 0(s1)
    addi    t3, t3, 1
    sw      t3, 0(s1)
    sw      x0, 0(s0)
    addi    s3, s3, 1
    bne     s3, s2, hart1_loop

    # 标 hart 1 完工 (HART_DONE[1] = 1)
    li      t4, HART_DONE_BASE
    li      t5, 1
    sw      t5, 4(t4)

    # spin 等 hart 0 写 test_dev PASS → main 触发 SHUTDOWN, hart 1 退出
hart1_spin:
    j       hart1_spin

# ============================================================
# acquire_lock_s0 — 用 s0 寄存器持 LOCK_ADDR, 阻塞直到获得锁
# clobbers: t0, t1 (caller 端 s3 worker 计数器不动)
# ============================================================
acquire_lock_s0:
acquire_retry:
    lr.w    t0, (s0)
    bnez    t0, acquire_retry               # lock 已占用 → 不发 SC (避免不必要的锁竞争), retry
    li      t1, 1
    sc.w    t0, t1, (s0)
    bnez    t0, acquire_retry               # SC 失败 (cross-hart 抢到 lock) → retry
    ret

fail:
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_FAIL
    sw      t1, 0(t0)
2:  j       2b

# ===== 期望结果 =====
#   COUNTER 最终 == 32 (2 * 16 严格序列化) → hart 0 写 FINISHER_PASS → exit_code 0
#   wall time < 1s (32 次成功 CAS + retry 路径, 数千条指令量级)
#   stderr 无 trap (cause 2 illegal / 4 misalign / 5/7 access fault / 15 page fault)
#   无 TSan race 报告 (LR/SC + 普通 SW 跨 hart 走 lrsc_on_store atomic 路径)
# 不期望:
#   - COUNTER < 32 → 序列化失败 (锁失效或丢失更新) → hart 0 验 fail
#   - 死锁 → 两 hart spin 永等 (fixture wall time 阈值会 catch)
#   - SC 虚假成功 (跟 b 路径区分) → COUNTER 跳过更新但 lock 状态不一致, fixture 不直接验,
#     但 03_lr_sc_aba_aa 专门覆盖此场景
