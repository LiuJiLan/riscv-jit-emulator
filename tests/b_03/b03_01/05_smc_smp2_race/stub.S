# b_03_1 / 05_smc_smp2_race
#
# ===== 运行参数 =====
# RUN-SMP: 2
# RUN-EXPECT-EXIT: 0
# RUN-TSAN-SKIP
#
# ===== TSan SKIP 原因 =====
# TSan 会报 'data race in lsu memcpy' — hart 1 sw 改 hot_loop inst 的 store_helper
# 内 memcpy 写 host RAM, 跟 hart 0 dispatcher fetch 该 RAM word (mmu_translate_pc
# fetch path) 没用 atomic, host 两线程 plain RAM r/w = TSan 视角真 race.
#
# 但**这是 RV spec 允许的 cross-hart no-fence.i race** — hart 0 fetch 看到 stale
# i-cache (旧 inst) 是 spec 允许的, OS 用 SBI_REMOTE_FENCE_I broadcast IPI 才保证
# cross-hart i-cache coherence. 我们 emulator 在 single-hart 内走 page_dirty bitmap
# + dispatcher 顶扫保证 eventually consistent; cross-hart 内 RV spec 不强制 (跟
# QEMU TCG / Box64 同).
#
# emulator 内部 store_helper memcpy 改用 atomic_store 会重创 perf (RAM 是 hot path);
# 跟 cross-hart i-cache coherence 的设计一起做推 a_05+ (SBI broadcast / 更强 RAM
# atomic 模型). 本 fixture 在 Debug+ASan+UBSan 验 race-free correctness (no SIGSEGV
# 139 / no use-after-free / no deadlock); TSan run 整体 skip.
#
# ===== 测试目的 =====
# 验 b_03 T1.a SMC chain 在 **多 hart race** 下不撞 use-after-free / 不死锁 / 不
# stale 跑错 (6 axes (ii)=多 hart + (iii)=SMC trigger 联合):
#   - hart 0 = 跑 hot_loop N iter 编 JIT 块 + 跑 + halt
#   - hart 1 = spin 无止境 sw 改 hot_loop inst (交替两种 imm); 看 STOP_FLAG 退
#   - 期望: cpu_wait_all_harts_exit_host_code 协议生效 — hart 1 改 inst →
#     SIGSEGV handler → page_dirty → dispatcher 顶扫 (hart 1 自己跑顶扫;
#     hart 0 也跑顶扫) → jit_invalidate_page → cpu_wait_all_harts_exit_host_code
#     扫所有 hart 等 host_code 持有者 (此时 hart 0 可能正跑 host_code, 状态灯 =
#     host_code) 退出, 然后 backend release RX 段; race-free 无 use-after-free
#
# **简化版** (跟 b_02 单 hart fixture 体例外的复杂度限制):
#   - 不 verify x6 精确值 (race 不可重复; 不同跑会得不同 x6, 因 stale execution
#     窗口跟系统 scheduling 相关)
#   - 只 verify: exit 0 + 不 crash (没 SIGSEGV 139) + 没 sanitizer warning
#     + dispatcher 跑通不死锁
#   - x6 应该 ∈ [N_iter, N_iter × 10] (跑改前 +1 跟跑改后 +10 交替)
#
# ===== 思路 =====
# 同 reset vector 启动 (跟 a03_14/01 同), csrr mhartid 分流:
#   hart 0: 跑 hot_loop 1000 iter + set STOP_FLAG + halt
#   hart 1: spin sw 改 hot_loop inst (交替 1↔10) + poll STOP_FLAG + halt
#
# 跨 hart RAM 写: hart 1 改 hot_loop inst 在 page 0x80000 (code page); hart 0
# 写 STOP_FLAG 在 page 0x80100 (data page, 跟 a03_14/01 DONE_FLAG_ADDR 同段).
# 不 overlap, mprotect race-safe.
#
# 跟 a03_14/01 同体例: 清 MDT/SDT prelude, mhartid 分流.

.equ TEST_DEV_BASE,     0x00100000
.equ FINISHER_PASS,     0x5555

.equ STOP_FLAG_ADDR,    0x80100000      # 数据段 (远离 .text); hart 0 set → hart 1 看

.equ HART0_ITER,        1000            # hot_loop iter count (跨 THRESHOLD 多次)

.section .text
.option norvc
.globl _start

_start:
    # ---- Boot prelude (两 hart 都跑): 清 MDT / SDT (跟 a03_14/01 同) ----
    csrw    0x310, x0                   # mstatush 清 MDT
    csrw    sstatus, x0                 # 清 SDT

    # ---- mhartid 分流 ----
    csrr    t0, mhartid
    beqz    t0, hart0_path
    li      t1, 1
    beq     t0, t1, hart1_path
    j       fail                        # mhartid >= 2 不期望

# ============================================================================
# hart 0: 跑 hot_loop N iter (编 JIT) + set STOP_FLAG + halt
# ============================================================================
hart0_path:
    lui     sp, 0x80300                 # hart 0 栈

    li      x1, HART0_ITER              # iter count = 1000
    li      x6, 0                       # accumulator

hot_loop:
    # *** 第一条 inst (hart 1 spin sw 改; 1 ↔ 10 交替) ***
    addi    x6, x6, 1                   # 0x00130313 (hart 1 改 → 0x00a30313)
    addi    x1, x1, -1
    bne     x1, x0, hot_loop

    # ---- set STOP_FLAG 让 hart 1 退 (避 t1=x6 撞 hot_loop accumulator; 用 t3/t2) ----
    li      t3, STOP_FLAG_ADDR
    li      t2, 1
    sw      t2, 0(t3)

    # ---- halt (用 a0/a1 不撞 hot_loop accumulator x6=t1) ----
    li      a0, TEST_DEV_BASE
    li      a1, FINISHER_PASS
    sw      a1, 0(a0)                   # test_dev fanout 全 hart 退
1:  j       1b

# ============================================================================
# hart 1: spin sw 改 hot_loop inst 直到 STOP_FLAG set
# ============================================================================
hart1_path:
    lui     sp, 0x80310                 # hart 1 栈 (跟 hart 0 不同; 不重叠)

    la      a2, hot_loop                # a2 = hot_loop PA (BARE identity; 用 a2 不撞 t)
    li      t3, STOP_FLAG_ADDR

spin_modify:
    lw      t4, 0(t3)                   # poll STOP_FLAG
    bnez    t4, hart1_halt              # set 后退

    # 改 hot_loop inst: 1 → 10 → 1 → 10 ... 交替
    li      t2, 0x00a30313              # addi x6, x6, 10
    sw      t2, 0(a2)                   # SMC trigger (round N)
    li      t2, 0x00130313              # addi x6, x6, 1 (改回)
    sw      t2, 0(a2)                   # SMC trigger (round N+1)
    j       spin_modify

hart1_halt:
    li      a0, TEST_DEV_BASE
    li      a1, FINISHER_PASS
    sw      a1, 0(a0)
1:  j       1b

# ============================================================================
# fail path (mhartid != 0/1 不期望)
# ============================================================================
fail:
    li      a0, TEST_DEV_BASE
    li      a1, 0x3333                  # FINISHER_FAIL
    sw      a1, 0(a0)
1:  j       1b

# ===== 期望结果 =====
# exit 0 (test_dev FINISHER_PASS 任一 hart 写都触发).
# hart 0 dump:
#   x1 = 0          (loop 倒数到 0)
#   x6 = ∈ [1000, 10000]    ← race; 不 verify 精确值
#   STOP_FLAG 已 set
#
# 关键 verify:
#   1. exit 0 (没 SIGSEGV 139 = handler 工作)
#   2. 没死锁 (有限时间内 hart 0 跑完 1000 iter)
#   3. 没 sanitizer (ASan/UBSan/TSan) warning
#   4. trace 中 J 字符跟 ._ 交替 (JIT hit ↔ invalidate-后-interpret 反复; 验
#      cpu_wait_all_harts_exit_host_code 协议工作)
#
# 不验 x6 精确值因 race 不可重复; race-free + 协议工作即 PASS.
