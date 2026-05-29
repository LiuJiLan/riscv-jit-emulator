# a03_1 / 01_plic_mmio_skeleton
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# PLIC MMIO 骨架 — 对 priority / pending / enable / context 四区各读 / 写一次,
# 验证 src/platform/plic.c 的 plic_read / plic_write 地址解码 + fprintf 输出对。
#
# T1 阶段 PLIC 不真接通 (字段被 fprintf 旁路), 这条 fixture **不验中断 fire**,
# 只验"MMIO 解码 stderr 上看得到 [plic] <region> <args> <R/W> 行 + 不崩 + 不 trap"。
# (注: 上述 T1 骨架期叙事保留作 trail。PLIC 完整 atomic 实装后那批逐访问 [plic] fprintf
#  已删 (plic.c 现无 [plic] trace), 本 fixture 现以"四区 R/W 全不报 access fault + exit 0"
#  验地址解码正确; 不再数 stderr trace 行。根因 REVIEW (B) stub→完整实装。)
#
# ===== 思路 =====
# RV I-type lw/sw 立即数 12-bit signed (-2048..+2047), 不能直接用 0x1000 / 0x2000
# 作 offset; 每个区先 li 算 base 到寄存器, 再用小立即数访问。

.section .text
.option norvc
.globl _start
_start:
    # ---- (1) priority src=1 R + W (PRIORITY base = PLIC_BASE) ----
    li      t0, 0x0C000000          # PLIC_BASE = priority base
    lw      t1, 4(t0)               # read priority[1]
    li      t2, 7
    sw      t2, 4(t0)               # write priority[1] = 7

    # ---- (2) pending word 0 R (PENDING base = PLIC_BASE + 0x1000) ----
    li      t0, 0x0C001000          # PENDING base
    lw      t1, 0(t0)               # read pending word 0 (src 0..31)

    # ---- (3) enable ctx 0 word 0 R + W (ENABLE ctx0 base = PLIC_BASE + 0x2000) ----
    li      t0, 0x0C002000          # ENABLE ctx 0 base
    lw      t1, 0(t0)               # read enable[ctx 0][word 0]
    li      t2, 0x2                 # 使能 source 1 (bit 1)
    sw      t2, 0(t0)               # write

    # ---- (4) enable ctx 1 word 0 R + W (ENABLE ctx1 base = PLIC_BASE + 0x2080) ----
    li      t0, 0x0C002080          # ENABLE ctx 1 base
    lw      t1, 0(t0)               # read enable[ctx 1][word 0]
    sw      t2, 0(t0)               # write same value

    # ---- (5) context ctx 0 全套 (CTX0 base = PLIC_BASE + 0x200000) ----
    # threshold RW: per-ctx 阈值; claim/complete 共享 off=+4 (R=claim, W=complete).
    li      t0, 0x0C200000          # CTX 0 base
    sw      x0, 0(t0)               # threshold = 0 W
    lw      t1, 0(t0)               # threshold R (验 threshold R 解码)
    lw      a6, 4(t0)               # claim R (T1 阶段返常 0)
    li      t2, 1
    sw      t2, 4(t0)               # complete W (假装 complete source 1)

    # ---- (6) context ctx 1 全套 (CTX1 base = PLIC_BASE + 0x201000) ----
    # 验 ctx_id 不同槽 (ctx=1) 解码也对.
    li      t0, 0x0C201000          # CTX 1 base
    sw      x0, 0(t0)               # threshold = 0 W
    lw      t1, 0(t0)               # threshold R
    lw      a6, 4(t0)               # claim R
    li      t2, 1
    sw      t2, 4(t0)               # complete W

    # ---- halt: test_dev FINISHER PASS 干净停机 (scratch x30/x31) ----
    li      x30, 0x100000           # test_dev base
    li      x31, 0x5555             # FINISHER PASS magic
    sw      x31, 0(x30)             # 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 四区 (priority / pending / enable / context) × 2 ctx 的 R/W 全部成功解码、无一报
# CAUSE_*_ACCESS_FAULT、不崩 → 程序跑到 halt 干净退 (exit 0)。即 plic_read / plic_write
# 地址解码对所有这些 off 都命中正确分支 (而非 fault/silent-drop 错分支)。
# (注: 旧期望是"数 stderr 上 14 行 [plic] <region> R/W trace" —— 那批逐访问 trace fprintf
#  在 PLIC 完整实装时已删, 现 stderr 无 [plic] 行; 改以 exit 0 + 无 access fault 验解码。
#  下方原 14 行 trace 清单按 trail 不再贴 (见 git 历史 / 本 fixture 汇编访问顺序自明)。)
#
# 停机:
#   test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
#   x30 = 0x100000 / x31 = 0x5555 (停机 scratch: test_dev base / FINISHER PASS magic)
# (注: 原停机靠 ecall + mtvec=0 triple fault; a_03 Smdbltrp 后 triple fault 变 MDT abort,
#  改 test_dev 干净停机。绝对 pc / total_count 见 REVIEW B.3 段, 不在断言内。)
#
# 不期望:
#   - 任何 [plic] 报 CAUSE_*_ACCESS_FAULT (size / 对齐 / 越界都不应该撞)
#   - 程序崩溃 (ASan / UBSan 报错)
