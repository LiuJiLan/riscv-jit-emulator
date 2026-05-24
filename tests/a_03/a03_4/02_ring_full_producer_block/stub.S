# a03_4 / 02_ring_full_producer_block
#
# ===== 测试目的 =====
# T6.2 plic_refresh_queue 16 slot ring 满时 producer block 路径 (cond_timedwait
# not_full 100ms 心跳 + 真等 consumer signal not_full) 不死锁 + consumer 正确 drain:
#   主帧紧凑 SET source 1 × 200 次 (远超 16 ring slot, 强制 producer block);
#   spin 等 csrr time 涨 20ms 给 consumer 充足时间 drain;
#   lw PLIC pending bitmap → 期望 bit 1 set = 0x2 (consumer 真消费过 SET event;
#     sources[1].device_line=1, sources[1].claimed=0);
#   CLEAR source 1 (一次) + spin 5ms;
#   lw PLIC pending bitmap → 期望 0;
#   ecall halt; dump 验 s0/s1.
#
# T6.2 ring 满时 producer 在 plic_enqueue_refresh 内 cond_timedwait not_full 100ms;
# consumer 在 plic_pending_refresh_run 拿 mutex + 消费 1 event + 释放 mutex +
# pthread_cond_signal not_full → producer 立即唤醒重检 queue_full → 入队下一个.
# 实际 producer block 时间 ~10-50µs/次 (consumer 处理 event 速率), 不是 100ms.
#
# 主帧不死锁 (run-config timeout 10s 内退出) = producer block 路径 cond_timedwait
# 三返回处理正确 (rc=0 / ETIMEDOUT / 其他错). pending bitmap=0x2 = consumer 真消费
# 过 SET event 改 sources[1].device_line=1.
#
# 本 fixture 不进 handler (mie.MEIP=0). 多 source + 仲裁验证由 01 fixture 覆盖.
#
# ===== 思路 =====
# 200 次 SET 用计数 loop (addi a5,a5,-1; bnez a5, loop), 比展开 200 行 sw 紧凑.
# release 100M MIPS 下单 sw test_dev 走 bus + plic_enqueue_refresh + mutex_lock ~50ns
# 量级 (consumer 跟得上时); ring 满时 producer block ~10-50µs/次; 200 SET 总耗时
# ~10ms (consumer 主导). csrr time spin 20ms 给余量.
#
# 顺手验 csrr time CSR (a_03_session_007 一并加, time/timeh CSR view of clint.mtime).

.equ TEST_DEV_BASE,      0x00100000
.equ TEST_DEV_SET_OFF,   0x40
.equ TEST_DEV_CLEAR_OFF, 0x44

.section .text
.option norvc
.globl _start

_start:
    # ---- 配 PLIC: source 1 minimal (priority=1, threshold=0, enable bit 1) ----
    li      t0, 0x0C000000;  li t1, 1;     sw t1, 4(t0)      # priority[1] = 1
    li      t0, 0x0C200000;                 sw x0, 0(t0)      # threshold[ctx0] = 0
    li      t0, 0x0C002000;  li t1, 0x2;   sw t1, 0(t0)      # enable[ctx0][0] bit 1

    # ---- 主帧 SET 1 × 200 (计数 loop) ----
    li      t1, TEST_DEV_BASE
    li      t2, 1                          # source_id = 1
    li      a5, 200                        # 200 次
set_loop:
    sw      t2, TEST_DEV_SET_OFF(t1)
    addi    a5, a5, -1
    bnez    a5, set_loop

    # ---- csrr time spin 20ms (200000 tick @ 100ns/tick) 等 consumer drain ring ----
    csrr    t3, time                       # t3 = mtime_lo base
    li      t4, 200000                     # 20ms 阈值
spin1:
    csrr    t5, time
    sub     t6, t5, t3
    blt     t6, t4, spin1

    # ---- 读 PLIC pending bitmap (验 consumer 消费完: bit 1 = 1, 期望 0x2) ----
    li      t0, 0x0C001000
    lw      s0, 0(t0)                      # s0 = pending (期望 0x2)

    # ---- 主帧 CLEAR 1 (一次) ----
    sw      t2, TEST_DEV_CLEAR_OFF(t1)

    # ---- csrr time spin 5ms 等 consumer 消费 CLEAR ----
    csrr    t3, time
    li      t4, 50000
spin2:
    csrr    t5, time
    sub     t6, t5, t3
    blt     t6, t4, spin2

    # ---- 读 pending (验 0) ----
    li      t0, 0x0C001000
    lw      s1, 0(t0)                      # s1 = pending (期望 0)

    # ---- halt: ecall → triple fault ----
    ecall

# ===== 期望结果 =====
# 期望最终寄存器 (debug build cpu_dump):
#   s0 = 0x2          (PLIC pending bitmap after SET 1 × 200 + 20ms spin; bit 1 set)
#   s1 = 0            (PLIC pending bitmap after CLEAR + 5ms spin)
#
# 期望 trap dump (halt 路径 triple fault):
#   in_trap = 3
#   mcause  = 1       (CAUSE_INST_ACCESS_FAULT; ecall + mtvec=0 → 取指 fault)
#
# 不期望:
#   - 主帧死锁 (run-config timeout 10s 命中 = producer block 路径 cond_timedwait
#     协议 bug; 三返回 rc=0 / ETIMEDOUT / 其他错没正确重检 queue_full / 没正确唤醒)
#   - s0 ≠ 0x2 (consumer 没 drain 完 / device_line 状态错)
#   - 任何 [bus] CAUSE_*_ACCESS_FAULT
#   - 程序崩溃 (ASan / UBSan; mutex / cond 死锁路径报错)
