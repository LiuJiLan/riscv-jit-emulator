# a04_1 / 02_fence_reserved_funct3_reject
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# T1 fence/fence.i 实装 reject 配对验证: 0x0F opcode 下只有 funct3=000 (FENCE) 跟
# funct3=001 (FENCE.I) 是合法分支, 其它 funct3 (010/011/100/101/110/111) 仍是
# OP_UNSUPPORTED → trap cause 2 illegal instruction.
#
# 配 01_fence_basic 一起组成 decode 0x0F case 的正反双覆盖:
#   01: funct3=000/001 happy path 不再 illegal
#   02: funct3=010 reserved 仍 illegal (= 本 fixture)
# 配置后任何 "funct3 分流写漏" / "误把保留 funct3 收成 FENCE" 的回归 01 跟 02 二选一
# 会爆.
#
# 验:
#   1. 0x0000200F (opcode 0x0F + funct3=010, rest=0) 触发 illegal trap
#   2. mcause = 2 (ILLEGAL_INSTRUCTION)
#   3. mtval = 0x0000200F (raw 指令; RV spec §3.1.16 illegal trap mtval = 触发指令本身)
#   4. handler 验完写 test_dev FINISHER_PASS → exit 0
# 不期望:
#   - 触发指令不 trap (= decode 误收 funct3=010 为 FENCE) → 走到 fall-through fail
#   - mcause != 2 (走错 trap 路径)
#   - mtval != 0x0000200F (raw_inst 字段没正确填)
#
# ===== 思路 =====
# 单 hart, M 模式跑全程 (illegal trap 默认不 delegate, trap to M).
# Boot prelude: 清 MDT/SDT 让 trap 不撞 Smdbltrp.
# 装 illegal_handler 到 mtvec direct mode (bits[1:0]=0).
# .word 0x0000200F 触发 → trap → handler 验 mcause + mtval → test_dev PASS.

.equ TEST_DEV_BASE,     0x00100000
.equ FINISHER_PASS,     0x5555
.equ FINISHER_FAIL,     0x3333

.equ ILLEGAL_RAW,       0x0000200F      # opcode 0x0F + funct3=010 + rest=0
.equ CAUSE_ILLEGAL,     2

.section .text
.option norvc
.globl _start

_start:
    # ---- Boot prelude: 清 MDT / SDT ----
    csrw    0x310, x0                   # mstatush 清 MDT
    csrw    sstatus, x0                 # 清 SDT

    # ---- 装 illegal_handler 到 mtvec (direct mode) ----
    la      t0, illegal_handler
    csrw    mtvec, t0

    # ---- 触发 reserved funct3 → expect illegal trap ----
    .word   ILLEGAL_RAW                 # 0x0000200F

    # fall-through: 不该到这, trap 应该跳走
    j       fail

illegal_handler:
    # 验 mcause == 2 (ILLEGAL_INSTRUCTION)
    csrr    t0, mcause
    li      t1, CAUSE_ILLEGAL
    bne     t0, t1, fail

    # 验 mtval == 0x0000200F (RV spec illegal mtval = raw 指令; interpreter
    # OP_UNSUPPORTED case 走 trap_raise_exception(.., d.raw_inst))
    csrr    t0, mtval
    li      t1, ILLEGAL_RAW
    bne     t0, t1, fail

    # ---- PASS ----
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_PASS
    sw      t1, 0(t0)
1:  j       1b                          # 防 fall-through (test_dev sw 触发退主循环)

fail:
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_FAIL
    sw      t1, 0(t0)
2:  j       2b

# ===== 期望结果 =====
#   handler 进入: mcause = 2, mtval = 0x0000200F
#   exit_code = 0 (FINISHER_PASS)
#   stderr 有一条 cause=2 trap 报 (不该报"unexpected illegal" 之类 panic)
#   main wall time < 50ms
#   无 ASan / TSan 报错
# 不期望:
#   - exit_code != 0 (走 fall-through fail 路径, 即触发指令被误识别为合法 FENCE)
#   - handler 内 mcause != 2 或 mtval != 0x0000200F (trap_raise 协议错)
#   - 任何 sanitizer 报错
