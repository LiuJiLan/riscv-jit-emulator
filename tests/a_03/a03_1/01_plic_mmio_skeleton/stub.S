# a03_1 / 01_plic_mmio_skeleton
#
# ===== 测试目的 =====
# PLIC MMIO 骨架 — 对 priority / pending / enable / context 四区各读 / 写一次,
# 验证 src/platform/plic.c 的 plic_read / plic_write 地址解码 + fprintf 输出对。
#
# T1 阶段 PLIC 不真接通 (字段被 fprintf 旁路), 这条 fixture **不验中断 fire**,
# 只验"MMIO 解码 stderr 上看得到 [plic] <region> <args> <R/W> 行 + 不崩 + 不 trap"。
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

    # ---- halt: ecall → triple fault (mtvec=0 默认, M-mode ecall 跳 0 取指 fault) ----
    ecall

# ===== 期望结果 =====
# stderr 看到以下 [plic] 行 (顺序跟汇编访问顺序一致, 14 行):
#   [plic] priority src=1 R
#   [plic] priority src=1 W
#   [plic] pending word=0 src=[0..31] R
#   [plic] enable ctx=0 src=[0..31] R
#   [plic] enable ctx=0 src=[0..31] W
#   [plic] enable ctx=1 src=[0..31] R
#   [plic] enable ctx=1 src=[0..31] W
#   [plic] ctx=0 threshold W
#   [plic] ctx=0 threshold R
#   [plic] ctx=0 claim R
#   [plic] ctx=0 complete W
#   [plic] ctx=1 threshold W
#   [plic] ctx=1 threshold R
#   [plic] ctx=1 claim R
#   [plic] ctx=1 complete W
#
# 期望 trap dump:
#   in_trap = 3                 (ecall → mtvec=0 → triple fault, 跟 a_02 fixture 同 halt)
#   mcause  = 1                 (CAUSE_INST_ACCESS_FAULT)
#
# 不期望:
#   - 任何 [plic] 报 CAUSE_*_ACCESS_FAULT (size / 对齐 / 越界都不应该撞)
#   - 程序崩溃 (ASan / UBSan 报错)
