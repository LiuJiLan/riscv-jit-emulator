# a04_2 / 01_amo_basic
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# T2 AMO 9 op (Zaamo) 实装基本验证. 每个 op 用已知 (mem, value) → 比对 (rd=old, mem_after).
# 9 op 各 1 次, 任一 op 异常 (rd != old / mem != expected) 走 fail.
#
# 验:
#   1. 9 AMO 编码全部 decode 成功 (T2 前 0x2F → OP_UNSUPPORTED → cause 2 illegal)
#   2. 9 AMO 语义正确:
#      AMOADD.W   = mem += value
#      AMOSWAP.W  = mem = value
#      AMOXOR.W   = mem ^= value
#      AMOOR.W    = mem |= value
#      AMOAND.W   = mem &= value
#      AMOMIN.W   = mem = signed_min(mem, value)
#      AMOMAX.W   = mem = signed_max(mem, value)
#      AMOMINU.W  = mem = unsigned_min(mem, value)
#      AMOMAXU.W  = mem = unsigned_max(mem, value)
#   3. rd 都是 old (RV spec sext32; RV32 下跟原 32-bit 值等价)
#   4. AMO 4-byte aligned (SCRATCH 起 0x80100000, 后续每段 +16 对齐)
#
# ===== 思路 =====
# 单 hart, M 模式 (REGIME_BARE; bare 不查 PTE, AMO 走 BARE + IS_GPA_RAM 命中 RAM 路径).
# Boot prelude 清 MDT/SDT 防 Smdbltrp.
# 每个 op:
#   1. li mem_init → sw 初值
#   2. li value (rs2 寄存器)
#   3. amoXXX.w rd, rs2, (rs1)
#   4. li expected_old → bne rd 不等就 fail
#   5. lw mem_after → bne 不等就 fail
# 9 op 全过 → test_dev PASS.
#
# SCRATCH 选 0x80100000 (远离 .text 0x80000000; 跟 a_03 fixture 体例).
# 测试值刻意选可识别的位模式 (不是 0/1) 防漏 sign-extend / 比较错误.

.equ TEST_DEV_BASE,     0x00100000
.equ FINISHER_PASS,     0x5555
.equ FINISHER_FAIL,     0x3333

.equ SCRATCH,           0x80100000

.section .text
.option norvc
.globl _start

_start:
    # ---- Boot prelude ----
    csrw    0x310, x0                   # mstatush 清 MDT
    csrw    sstatus, x0                 # 清 SDT

    li      s0, SCRATCH                 # 工作地址 (固定)

    # ============================================================
    # AMOADD.W: mem=100, value=7  → old=100, new=107
    # ============================================================
    li      t1, 100
    sw      t1, 0(s0)
    li      t2, 7
    amoadd.w t3, t2, (s0)
    li      t4, 100
    bne     t3, t4, fail                # rd 应 = old
    lw      t5, 0(s0)
    li      t4, 107
    bne     t5, t4, fail                # mem_after = 100+7

    # ============================================================
    # AMOSWAP.W: mem=0xDEADBEEF, value=0xCAFEBABE → old=0xDEADBEEF, new=0xCAFEBABE
    # ============================================================
    li      t1, 0xDEADBEEF
    sw      t1, 0(s0)
    li      t2, 0xCAFEBABE
    amoswap.w t3, t2, (s0)
    li      t4, 0xDEADBEEF
    bne     t3, t4, fail
    lw      t5, 0(s0)
    li      t4, 0xCAFEBABE
    bne     t5, t4, fail

    # ============================================================
    # AMOXOR.W: mem=0xFF00FF00, value=0x0FF00FF0 → old=mem, new = mem^value = 0xF0F0F0F0
    # ============================================================
    li      t1, 0xFF00FF00
    sw      t1, 0(s0)
    li      t2, 0x0FF00FF0
    amoxor.w t3, t2, (s0)
    li      t4, 0xFF00FF00
    bne     t3, t4, fail
    lw      t5, 0(s0)
    li      t4, 0xF0F0F0F0
    bne     t5, t4, fail

    # ============================================================
    # AMOOR.W:  mem=0xF000F000, value=0x0F0F0F0F → old=mem, new = 0xFF0FFF0F
    # ============================================================
    li      t1, 0xF000F000
    sw      t1, 0(s0)
    li      t2, 0x0F0F0F0F
    amoor.w  t3, t2, (s0)
    li      t4, 0xF000F000
    bne     t3, t4, fail
    lw      t5, 0(s0)
    li      t4, 0xFF0FFF0F
    bne     t5, t4, fail

    # ============================================================
    # AMOAND.W: mem=0xFFFFFFFF, value=0x0F0F0F0F → old=mem, new = 0x0F0F0F0F
    # ============================================================
    li      t1, 0xFFFFFFFF
    sw      t1, 0(s0)
    li      t2, 0x0F0F0F0F
    amoand.w t3, t2, (s0)
    li      t4, 0xFFFFFFFF
    bne     t3, t4, fail
    lw      t5, 0(s0)
    li      t4, 0x0F0F0F0F
    bne     t5, t4, fail

    # ============================================================
    # AMOMIN.W (signed): mem=10, value=-5 → old=10, new=-5 (signed_min)
    # ============================================================
    li      t1, 10
    sw      t1, 0(s0)
    li      t2, -5
    amomin.w t3, t2, (s0)
    li      t4, 10
    bne     t3, t4, fail
    lw      t5, 0(s0)
    li      t4, -5
    bne     t5, t4, fail

    # ============================================================
    # AMOMAX.W (signed): mem=-10, value=5 → old=-10, new=5 (signed_max)
    # ============================================================
    li      t1, -10
    sw      t1, 0(s0)
    li      t2, 5
    amomax.w t3, t2, (s0)
    li      t4, -10
    bne     t3, t4, fail
    lw      t5, 0(s0)
    li      t4, 5
    bne     t5, t4, fail

    # ============================================================
    # AMOMINU.W (unsigned): mem=0xFFFFFFFF, value=1 → old=0xFFFFFFFF, new=1
    #   (unsigned_min(0xFFFFFFFF, 1) = 1; signed 视角是 (-1, 1) → -1 < 1; 区分点)
    # ============================================================
    li      t1, 0xFFFFFFFF
    sw      t1, 0(s0)
    li      t2, 1
    amominu.w t3, t2, (s0)
    li      t4, 0xFFFFFFFF
    bne     t3, t4, fail
    lw      t5, 0(s0)
    li      t4, 1
    bne     t5, t4, fail

    # ============================================================
    # AMOMAXU.W (unsigned): mem=1, value=0xFFFFFFFF → old=1, new=0xFFFFFFFF
    #   (unsigned_max(1, 0xFFFFFFFF) = 0xFFFFFFFF; signed 视角是 (1, -1) → 1 > -1; 区分点)
    # ============================================================
    li      t1, 1
    sw      t1, 0(s0)
    li      t2, 0xFFFFFFFF
    amomaxu.w t3, t2, (s0)
    li      t4, 1
    bne     t3, t4, fail
    lw      t5, 0(s0)
    li      t4, 0xFFFFFFFF
    bne     t5, t4, fail

    # ---- PASS ----
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
#   9 段 AMO 全部通过 → exit_code = 0 (FINISHER_PASS)
#   stderr 无 trap cause 2 (illegal) / cause 6 (misalign) / cause 7 (access fault) / cause 15 (page fault)
#   wall time < 100ms (9 AMO + 9 verify ≈ 不到 100 条指令)
#   无 ASan / TSan 报错
# 不期望:
#   - 某 op decode 漏 → cause 2 illegal
#   - 某 op apply 错 (rd 错 / mem_after 错) → fail handler → exit_code != 0
#   - signed vs unsigned 算错 (MIN/MAX/MINU/MAXU 区分点设计就为捕捉这类错)
