# a03_4 / 01_async_multi_source
#
# ===== 测试目的 =====
# T6.2 PLIC 异步消费可见性 + 多 source 仲裁正确性:
#   主帧紧凑 SET 1/2/3/4 (4 次连续 sw test_dev SET_OFF), ring 入队 4 event;
#   plic_pending_refresh_thread 异步消费 (consumer 在另一 OS thread); spin 等
#   csrr time 涨 5ms 让 consumer 消费完 4 event; 然后:
#     - lw PLIC pending bitmap @0x0C001000 → 期望 = 0x1E (bit 1-4 都 set)
#     - csrr mip → 期望 MEIP bit (0x800) set
#   再紧凑 CLEAR 1/2/3/4 + spin → 期望 pending=0 + MEIP=0
#   ecall halt; dump 验 s0/s1/s2/s3.
#
# T6.1 时 device_set_pending 是同步 (直接 wrlock 改 sources[].device_line), pending
# bitmap 立即可见; T6.2 后异步消费, pending bitmap 有 ms 量级延迟. 本 fixture 通过
# csrr time spin 5ms 间接验"consumer 跟得上 / 异步可见性最终一致"。顺手验 csrr time
# CSR (a_03_session_007 一并加, time/timeh CSR view of clint.mtime).
#
# 不进 m_handler (mie.MEIP=0), 只验 PLIC pending bitmap 跟 mip MEIP 合成视图.
# 进 handler 的路径已被 a_03/a03_2 全套覆盖, 本 fixture 专门验 T6.2 新加路径.
#
# ===== 思路 =====
# csrr time spin: csrr base / loop csrr cur / sub / blt; 50000 tick @ 100ns/tick = 5ms.
# 不依赖编译器优化 (.S 手写不优化), 不依赖 host CPU 速度 (跨机稳).

.equ TEST_DEV_BASE,      0x00100000
.equ TEST_DEV_SET_OFF,   0x40
.equ TEST_DEV_CLEAR_OFF, 0x44

.section .text
.option norvc
.globl _start

_start:
    # ---- 不设 mtvec / mie.MEIP / mstatus.MIE (本 fixture 不进 handler) ----

    # ---- 配 PLIC: priority[1..4] = 1 (都同 prio, 仲裁挑 src 升序) ----
    li      t0, 0x0C000000          # PLIC_BASE = priority base
    li      t1, 1
    sw      t1, 4(t0)               # priority[1] = 1
    sw      t1, 8(t0)               # priority[2] = 1
    sw      t1, 12(t0)              # priority[3] = 1
    sw      t1, 16(t0)              # priority[4] = 1

    # ---- 配 PLIC: threshold[ctx0] = 0 ----
    li      t0, 0x0C200000          # CTX 0 base
    sw      x0, 0(t0)

    # ---- 配 PLIC: enable[ctx0][word0] = (1<<1)|(1<<2)|(1<<3)|(1<<4) = 0x1E ----
    li      t0, 0x0C002000          # ENABLE ctx 0 word 0
    li      t1, 0x1E
    sw      t1, 0(t0)

    # ---- 主帧紧凑 SET 1/2/3/4 (4 次连续 sw, ring 入队 4 event) ----
    li      t1, TEST_DEV_BASE
    li      t0, 1;  sw t0, TEST_DEV_SET_OFF(t1)
    li      t0, 2;  sw t0, TEST_DEV_SET_OFF(t1)
    li      t0, 3;  sw t0, TEST_DEV_SET_OFF(t1)
    li      t0, 4;  sw t0, TEST_DEV_SET_OFF(t1)

    # ---- csrr time spin 5ms (50000 tick) 等 consumer 消费完 4 event ----
    csrr    t1, time                # t1 = mtime_lo base
    li      t2, 50000               # 5ms 阈值
spin1:
    csrr    t3, time
    sub     t4, t3, t1
    blt     t4, t2, spin1

    # ---- 读 PLIC pending bitmap (验 consumer 消费完: bit 1-4 都 set = 0x1E) ----
    li      t0, 0x0C001000          # PENDING base, word 0
    lw      s0, 0(t0)               # s0 = pending bitmap (期望 0x1E)

    # ---- csrr mip (验 MEIP bit 11 = 0x800 set) ----
    csrr    s1, mip                 # s1 = mip (期望 含 0x800; 其他 bit 不挑)

    # ---- 主帧紧凑 CLEAR 1/2/3/4 ----
    li      t1, TEST_DEV_BASE
    li      t0, 1;  sw t0, TEST_DEV_CLEAR_OFF(t1)
    li      t0, 2;  sw t0, TEST_DEV_CLEAR_OFF(t1)
    li      t0, 3;  sw t0, TEST_DEV_CLEAR_OFF(t1)
    li      t0, 4;  sw t0, TEST_DEV_CLEAR_OFF(t1)

    # ---- csrr time spin 5ms 等 consumer 消费 CLEAR ----
    csrr    t1, time
    li      t2, 50000
spin2:
    csrr    t3, time
    sub     t4, t3, t1
    blt     t4, t2, spin2

    # ---- 读 PLIC pending bitmap (验 = 0) ----
    li      t0, 0x0C001000
    lw      s2, 0(t0)               # s2 = pending (期望 0)

    # ---- csrr mip (验 MEIP bit clear) ----
    csrr    s3, mip                 # s3 = mip (期望 0x800 bit clear; 其他 bit 不挑)

    # ---- halt: ecall → triple fault (mtvec 默认 0, ecall 走 handler 又取指 fault) ----
    ecall

# ===== 期望结果 =====
# 期望最终寄存器 (debug build cpu_dump):
#   s0 = 0x1E                  (PLIC pending bitmap after SET 1/2/3/4 + 5ms spin)
#   s1 含 0x800 bit            (mip MEIP set; mip 其他 bit 可能有 MTIP/MSIP noise, mask 0x800)
#   s2 = 0                     (PLIC pending bitmap after CLEAR + 5ms spin)
#   s3 不含 0x800 bit          (mip MEIP clear; mask 0x800)
#
# 期望 trap dump (halt 路径 triple fault):
#   in_trap = 3
#   mcause  = 1                (CAUSE_INST_ACCESS_FAULT; ecall + mtvec=0 → 取指 fault)
#
# 不期望:
#   - cpu_dump 出现 mcause=5/7 (LOAD/STORE_ACCESS_FAULT; bus 失败路径
#     trap_raise_exception longjmp, 不打 fprintf — src/ 内无 [bus] tag)
#   - 程序崩溃 (ASan / UBSan)
#   - spin loop 死等 (mtime 不涨 = timer thread 没跑 = 退化)
#   - s0 ≠ 0x1E (consumer 没 drain 完 / 多 source recompute_all 没反映)
