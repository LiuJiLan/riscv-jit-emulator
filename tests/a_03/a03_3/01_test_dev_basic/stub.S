# a03_3 / 01_test_dev_basic
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# test_dev MMIO fanout 端到端 (T3 step A 验收 fixture):
#   1. 配 PLIC: priority[3]=1, threshold[ctx0]=0, enable[ctx0][word0] bit3=1
#   2. 经 test_dev MMIO 写: sw TEST_DEV_SET_OFF(t1), value=3 → device_set_pending(3)
#   3. PLIC 仲裁: ctx0 (= hart0 M ctx) 有可送 source → is_plic_meip_pending(0)=1
#                  → csr_mip_read 合成 MEIP (bit 11) → trap_check_interrupt fire
#   4. m_handler:
#        - 验 mcause = 0x8000000B (INT_BIT | IRQ_M_EXT=11)
#        - lw claim @ PLIC_BASE+0x200004 → 期望 = 3
#        - sw TEST_DEV_CLEAR_OFF(t1), 3 → device_clear_pending(3)
#        - sw complete @ PLIC_BASE+0x200004, 3 → PLIC 清 sources[3].claimed
#        - mret
#   5. 主流程退 → test_dev sifive FINISHER PASS 干净停机 (exit_code=0)
#
# 跟 a_03_2/01 的本质同体例 (step B 切后 a_03_2/01 也用 test_dev MMIO); 此 fixture 是
# step A 新建的 test_dev 验收入口, 跟 a_03_2/01 跑通效果等价, 留 a_03 收尾 milestone
# test review 时 dedupe。
#
# ===== 思路 =====
# RV I-type lw/sw 立即数 12-bit signed (-2048..+2047), test_dev SET/CLEAR off = 0x40/0x44
# 在 imm 范围内 → 一条 li t1, TEST_DEV_BASE + sw t0, 0x40(t1) 够 (跟 PLIC 大 offset 不同,
# 不用拆 lui+addi)。
#
# sw 不是 hard block boundary (跟 csrw 区分; a_03_2/01 用 csrw 必须把 li s0/a5 挪到
# trigger 前), 这里 init 跟 trigger 顺序不严格, 但保留 "init 在 trigger 前" 风格更稳。

.equ TEST_DEV_BASE,      0x00100000
.equ TEST_DEV_SET_OFF,   0x40
.equ TEST_DEV_CLEAR_OFF, 0x44

.section .text
.option norvc
.globl _start

_start:
    # ---- M init: mtvec / mie.MEIP / mstatus.MIE ----
    la      t0, m_handler
    csrw    mtvec, t0

    # mie.MEIP = (1<<11) = 0x800
    li      t0, 0x800
    csrw    mie, t0

    # mstatus.MIE = 1 (bit 3)
    csrw    0x310, x0               # mstatush 清 MDT (A.7 prelude; 否则下条 MIE 写被强制 0)
    csrrsi  x0, mstatus, 0x8

    # ---- 配 PLIC: priority[3]=1 ----
    li      t0, 0x0C000000          # PLIC_BASE
    li      t1, 1
    sw      t1, 12(t0)              # priority[3] = 1 (off = 4*3 = 12)

    # ---- 配 PLIC: threshold[ctx0]=0 ----
    li      t0, 0x0C200000          # PLIC_BASE + CONTEXT_OFF (ctx 0)
    sw      x0, 0(t0)               # threshold = 0

    # ---- 配 PLIC: enable[ctx0][word0] |= (1<<3) ----
    li      t0, 0x0C002000          # PLIC_BASE + ENABLE_OFF (ctx 0)
    li      t1, 0x8                 # bit 3 = source 3
    sw      t1, 0(t0)

    # ---- s0/a5 初始化 (sw 不是 hard boundary, 顺序不严格, init-before-trigger 风格更稳) ----
    li      s0, 0
    li      a5, 0

    # ---- 触发 device_set_pending(3) via test_dev MMIO ----
    li      t1, TEST_DEV_BASE
    li      t0, 3
    sw      t0, TEST_DEV_SET_OFF(t1)

wait_loop:
    addi    a5, a5, 1
    beqz    s0, wait_loop

    # 保存 handler 给的结果到 a0 (cause), a1 (claim_src)
    mv      a0, s1
    mv      a1, s2

    # halt: test_dev sifive FINISHER PASS → 干净停机, exit_code=0
    # 注: 替代旧 triple-fault 停机 (Smdbltrp MDT abort 后不再可用)
    li      t2, 0x00100000          # TEST_DEV_BASE
    li      t3, 0x5555              # FINISHER_PASS
    sw      t3, 0(t2)               # sifive FINISHER off 0
1:  j       1b                      # 防 fall-through: SRS=0 下个 block 边界退

m_handler:
    # 验 mcause = INT_BIT | IRQ_M_EXT (11) = 0x8000000B
    csrr    s1, mcause

    # 读 PLIC claim @ PLIC_BASE + CONTEXT_OFF + 4 (ctx 0)
    li      t0, 0x0C200004          # claim register (ctx 0)
    lw      s2, 0(t0)               # 期望 s2 = 3

    # 触发 device_clear_pending(3) via test_dev MMIO
    li      t1, TEST_DEV_BASE
    li      t2, 3
    sw      t2, TEST_DEV_CLEAR_OFF(t1)

    # 写 PLIC complete = 3 (同地址 0x0C200004, write 是 complete)
    sw      s2, 0(t0)

    # 通知主流程退出
    li      s0, 1
    mret

# ===== 期望结果 =====
# 期望最终寄存器:
#   a0 = 0x8000000B   (INT_BIT | IRQ_M_EXT = 0x80000000 | 11)
#   a1 = 3            (PLIC claim 返的 source_id)
#   s0 = 1
#   s1 = 0x8000000B
#   s2 = 3
#   a5 > 0
#
# 期望停机 (test_dev sifive FINISHER PASS):
#   shell 退出码 $? = 0
#   [dispatcher] halted: ... (SRS=0 退主循环)
#
# 不期望:
#   - 任何 [bus] CAUSE_*_ACCESS_FAULT (test_dev / plic 不该产 fault)
#   - 程序崩溃 (ASan / UBSan 报错)
#   - wait_loop 永不退 (说明 test_dev fanout / PLIC 仲裁 / mip 合成 / trap_check_interrupt 哪环断了)
