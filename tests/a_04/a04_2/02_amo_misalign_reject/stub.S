# a04_2 / 02_amo_misalign_reject
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# T2 AMO_MISALIGN_CHECK 反向验证: AMOADD.W 地址非 4-byte aligned (gva & 3 != 0) 触发
# cause 6 (CAUSE_STORE_ADDR_MISALIGNED; RV spec §4.6 store/AMO 同 cause). interpreter
# case 入口 AMO_MISALIGN_CHECK 宏 trap_raise_exception, helper 不会到达.
#
# 配 01_amo_basic 一起组成 AMO misalign 正反双覆盖:
#   01: 4-byte aligned AMO 不 trap
#   02: 非 4-byte aligned AMO trap cause 6 (= 本 fixture)
#
# 验:
#   1. AMOADD.W 在 0x80100001 (= aligned + 1) 触发 trap
#   2. mcause = 6
#   3. mtval = 0x80100001 (RV spec misalign tval = gva 不齐地址)
#   4. handler 验完写 test_dev FINISHER_PASS → exit 0
# 不期望:
#   - AMO 不 trap (= AMO_MISALIGN_CHECK 漏 / 失效)
#   - mcause != 6 (走错 trap 路径)
#   - mtval != 0x80100001
#
# ===== 思路 =====
# 单 hart, M 模式. Boot prelude 清 MDT/SDT.
# 装 misalign_handler 到 mtvec direct.
# amoadd.w 用未对齐 gva 触发 trap → handler 验 mcause + mtval → test_dev PASS.

.equ TEST_DEV_BASE,     0x00100000
.equ FINISHER_PASS,     0x5555
.equ FINISHER_FAIL,     0x3333

.equ MISALIGN_ADDR,     0x80100001      # = 0x80100000 + 1 (非 4-byte aligned)
.equ CAUSE_STORE_MISALIGN, 6

.section .text
.option norvc
.globl _start

_start:
    csrw    0x310, x0                   # 清 MDT
    csrw    sstatus, x0                 # 清 SDT

    la      t0, misalign_handler
    csrw    mtvec, t0

    # 触发: AMOADD.W rd, rs2, (rs1) 其中 rs1 = 0x80100001 (非 4-byte aligned)
    li      s0, MISALIGN_ADDR
    li      t2, 7                       # rs2 value (不重要; 不到 apply)
    amoadd.w t3, t2, (s0)               # → AMO_MISALIGN_CHECK trap cause 6

    # fall-through: 不该到这
    j       fail

misalign_handler:
    csrr    t0, mcause
    li      t1, CAUSE_STORE_MISALIGN
    bne     t0, t1, fail

    csrr    t0, mtval
    li      t1, MISALIGN_ADDR
    bne     t0, t1, fail

    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_PASS
    sw      t1, 0(t0)
1:  j       1b

fail:
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_FAIL
    sw      t1, 0(t0)
2:  j       2b

# ===== 期望结果 =====
#   handler 进入: mcause = 6, mtval = 0x80100001
#   exit_code = 0 (FINISHER_PASS)
#   stderr 有一条 cause=6 trap 报
#   wall time < 50ms
#   无 ASan / TSan 报错
# 不期望:
#   - exit_code != 0 (AMO 没 trap → fall-through fail / 或 handler 内 mcause/mtval 错)
#   - sanitizer 报错
