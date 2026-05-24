# a03_4 / 03_pending_bitmap_mmio_view
#
# ===== 测试目的 =====
# PLIC MMIO pending bitmap 区 (off [0x1000, 0x2000)) 读路径完整验证. T6.2 post-EIP
# 改 atomic_load(plic_pending_bitmap_cache[word_idx], acquire) 直返 — 过去 a_03
# fixture 全套**无人专门测此路径** (a_03/a03_1/01 只验地址解码不崩; a_03/a03_2 全套
# 都测中断 fire; a_03/a03_4 现有 01+02 测 pending 是 verify consumer drain 的 side
# effect), 现 atomic cache 加入顺手补一个专门验.
#
# 覆盖测试点 (跟 plic.c plic_pending_bitmap_cache 调用点矩阵对齐):
#   1. 空 baseline: SET 前 lw 3 word 都 0
#   2. 跨 word 读 (96 src / 3 word): SET src 1/5/35/70 跨 3 word; spin 等 consumer
#      drain; lw word 0/1/2 验各 word 独立解码 + atomic cache 各 word 独立刷新
#   3. csrr mip 顺手验 MEIP set (跨 word source 都参与 ctx_eip 仲裁)
#   4. claim → pending bit 清: lw claim 仲裁选最小 src=1 (priority 都 1, 升序);
#      lw word 0 验 bit 1 清 (sources[1].claimed 后 pending = device_line && !claimed
#      = 1 && 0 = 0); plic_read claim wrlock 内调 plic_recompute_pending_bitmap_locked
#   5. complete → pending bit 重新出现: sw complete src=1; lw word 0 验 bit 1
#      重新可见 (claimed 清, device_line 仍 set); plic_write complete wrlock 内调
#      plic_recompute_pending_bitmap_locked
#   6. CLEAR + spin → 全清: CLEAR 1/5/35/70; spin 等 consumer; lw word 0 验 0
#
# 不进 m_handler (mie.MEIP=0); 只测 pending bitmap 读路径 + cache 跟 device_line /
# claimed 联动正确. 中断 fire 路径由 a_03/a03_2 + a_03/a03_4/01/02 覆盖.
#
# ===== 思路 =====
# 跨 word source 选: src 1+5 (word 0), src 35 (word 1, bit 3), src 70 (word 2, bit 6).
# priority 都 = 1 → claim 仲裁升序选 src 1.
# csrr time spin 5ms 等 consumer drain (跟 01/02 同体例).
# priority offset 4/20/140/280 都在 sw 12-bit signed imm 范围 (±2047), 单条 sw 不拆.

.equ TEST_DEV_BASE,      0x00100000
.equ TEST_DEV_SET_OFF,   0x40
.equ TEST_DEV_CLEAR_OFF, 0x44

.section .text
.option norvc
.globl _start

_start:
    # ---- 不设 mtvec / mie.MEIP / mstatus.MIE (本 fixture 不进 handler) ----

    # ---- 配 PLIC priority[1] = priority[5] = priority[35] = priority[70] = 1 ----
    li      t0, 0x0C000000          # PLIC_BASE = priority base
    li      t1, 1
    sw      t1, 4(t0)               # priority[1]  (offset 1*4)
    sw      t1, 20(t0)              # priority[5]  (offset 5*4)
    sw      t1, 140(t0)             # priority[35] (offset 35*4)
    sw      t1, 280(t0)             # priority[70] (offset 70*4)

    # ---- threshold[ctx0] = 0 ----
    li      t0, 0x0C200000
    sw      x0, 0(t0)

    # ---- enable[ctx0][word0..2] = 0x22 / 0x8 / 0x40 ----
    # word 0: bit 1 (src 1) + bit 5 (src 5) = 0x22
    # word 1: bit 3 (src 35 = 1*32 + 3) = 0x08
    # word 2: bit 6 (src 70 = 2*32 + 6) = 0x40
    li      t0, 0x0C002000          # ENABLE ctx 0 word 0
    li      t1, 0x22; sw t1, 0(t0)
    li      t1, 0x08; sw t1, 4(t0)
    li      t1, 0x40; sw t1, 8(t0)

    # ========================================================================
    # 测试点 1: 空 baseline — SET 前 lw 3 word 都 0
    # ========================================================================
    li      t0, 0x0C001000          # PENDING base
    lw      s0, 0(t0)               # word 0 (期望 0)
    lw      s1, 4(t0)               # word 1 (期望 0)
    lw      s2, 8(t0)               # word 2 (期望 0)

    # ========================================================================
    # 测试点 2: SET 1/5/35/70 跨 word + spin + 读 3 word 验跨 word 解码
    # ========================================================================
    li      t1, TEST_DEV_BASE
    li      t0, 1;   sw t0, TEST_DEV_SET_OFF(t1)
    li      t0, 5;   sw t0, TEST_DEV_SET_OFF(t1)
    li      t0, 35;  sw t0, TEST_DEV_SET_OFF(t1)
    li      t0, 70;  sw t0, TEST_DEV_SET_OFF(t1)

    # csrr time spin 5ms 等 consumer drain 4 event
    csrr    t1, time
    li      t2, 50000
spin1:
    csrr    t3, time
    sub     t4, t3, t1
    blt     t4, t2, spin1

    li      t0, 0x0C001000
    lw      s3, 0(t0)               # word 0 (期望 0x22 = bit 1 + bit 5)
    lw      s4, 4(t0)               # word 1 (期望 0x08 = bit 3, src 35)
    lw      s5, 8(t0)               # word 2 (期望 0x40 = bit 6, src 70)

    # ========================================================================
    # 测试点 3: csrr mip 顺手验 MEIP set (跨 word source 都参与仲裁 → ctx_eip = 1)
    # ========================================================================
    csrr    s6, mip                 # 期望含 0x800 (MEIP bit 11)

    # ========================================================================
    # 测试点 4: lw claim → 仲裁选最小 src (priority 都 1, src 升序选 1)
    #          + 读 word 0 验 bit 1 清 (sources[1].claimed 后 pending = 0)
    # ========================================================================
    li      t0, 0x0C200004          # ctx 0 claim/complete reg
    lw      s7, 0(t0)               # 期望 = 1 (src 1, 最小)

    li      t0, 0x0C001000
    lw      s8, 0(t0)               # word 0 (期望 0x20 = bit 1 清; bit 5 仍 set)

    # ========================================================================
    # 测试点 5: complete src=1 + 读 word 0 验 bit 1 重新出现 (claimed 清, device_line 仍 set)
    # ========================================================================
    li      t0, 0x0C200004
    li      t1, 1;  sw t1, 0(t0)    # complete src 1 (清 sources[1].claimed)

    li      t0, 0x0C001000
    lw      s9, 0(t0)               # word 0 (期望 0x22 = bit 1 重新可见)

    # ========================================================================
    # 测试点 6: CLEAR 1/5/35/70 + spin → 验全清
    # ========================================================================
    li      t1, TEST_DEV_BASE
    li      t0, 1;   sw t0, TEST_DEV_CLEAR_OFF(t1)
    li      t0, 5;   sw t0, TEST_DEV_CLEAR_OFF(t1)
    li      t0, 35;  sw t0, TEST_DEV_CLEAR_OFF(t1)
    li      t0, 70;  sw t0, TEST_DEV_CLEAR_OFF(t1)

    csrr    t1, time
    li      t2, 50000
spin2:
    csrr    t3, time
    sub     t4, t3, t1
    blt     t4, t2, spin2

    li      t0, 0x0C001000
    lw      s10, 0(t0)              # word 0 (期望 0; word 1/2 同, 不重复 dump 省寄存器)

    # ---- halt: ecall → triple fault (mtvec=0) ----
    ecall

# ===== 期望结果 =====
# 期望最终寄存器 (debug build cpu_dump):
#   s0 = 0      (baseline word 0)
#   s1 = 0      (baseline word 1)
#   s2 = 0      (baseline word 2)
#   s3 = 0x22   (SET 后 word 0 = bit 1 + bit 5)
#   s4 = 0x08   (SET 后 word 1 = bit 3, src 35 跨 word 解码验)
#   s5 = 0x40   (SET 后 word 2 = bit 6, src 70 跨 word 解码验)
#   s6 含 0x800 (mip MEIP set, ctx_eip 反映跨 word source 仲裁)
#   s7 = 1      (claim 返 src 1, 仲裁升序选最小)
#   s8 = 0x20   (claim 后 word 0, bit 1 清 因 claimed; bit 5 仍 set)
#   s9 = 0x22   (complete 后 word 0, bit 1 重新可见; cache 跟 claimed 清联动)
#   s10 = 0     (CLEAR 后 word 0, 全清; consumer 异步 drain 跟 cache 联动)
#
# 期望 trap dump (halt 路径 triple fault):
#   in_trap = 3
#   mcause  = 1                (CAUSE_INST_ACCESS_FAULT; ecall + mtvec=0)
#
# 不期望:
#   - cpu_dump 出现 mcause=5/7 (LOAD/STORE_ACCESS_FAULT; bus 失败路径
#     trap_raise_exception longjmp, 不打 fprintf — src/ 内无 [bus] tag)
#   - 程序崩溃 (ASan / UBSan)
#   - spin loop 死等 (mtime 不涨 = timer thread 没跑 = 退化)
#   - s3 ≠ 0x22 / s4 ≠ 0x08 / s5 ≠ 0x40 (跨 word 解码错 / cache 各 word 没正确刷新)
#   - s8 ≠ 0x20 (plic_read claim 路径没调 plic_recompute_pending_bitmap_locked)
#   - s9 ≠ 0x22 (plic_write complete 路径没调 plic_recompute_pending_bitmap_locked)
