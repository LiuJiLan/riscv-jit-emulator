# a02_2 / 01_mmio_basic
#
# ===== 测试目的 =====
# CLINT MMIO 读写基础验证 (T1: bus + CLINT 路径走通; 不验 timer / soft interrupt)。
#
# ===== 思路 =====
# 走 BARE 模式 (priv=M, satp=0); CLINT 在 0x02000000..0x02010000, 不在 RAM 区,
# load/store BARE 路径 IS_GPA_RAM 不命中 → bus_dispatch → 命中 CLINT → device fn
# (clint_read/clint_write) → 写入 / 返回值。
#
# 寄存器布局 (config.h CLINT_*):
#   msip[0]     @ 0x02000000   4 byte  (仅低 1 位有效)
#   mtimecmp[0] @ 0x02004000   8 byte  (低半 0x02004000 / 高半 0x02004004)
#   mtime       @ 0x0200BFF8   8 byte  (低半 0x0200BFF8 / 高半 0x0200BFFC)

.section .text
.option norvc
.globl _start
_start:
    # --- mtimecmp[0] 写 + 读 ---
    li   t0, 0x02004000          # mtimecmp[0] 低半地址
    li   t1, 0xDEADBEEF
    sw   t1, 0(t0)               # mtimecmp[0] 低半 = 0xDEADBEEF
    li   t1, 0x12345678
    sw   t1, 4(t0)               # mtimecmp[0] 高半 = 0x12345678
    lw   a0, 0(t0)               # a0 = 0xDEADBEEF
    lw   a1, 4(t0)               # a1 = 0x12345678

    # --- msip[0] 写 + 读 ---
    li   t0, 0x02000000          # msip[0] 地址
    li   t1, 1
    sw   t1, 0(t0)               # msip[0] = 1 (clint 内 mask 低 1 位)
    lw   a2, 0(t0)               # a2 = 1

    # --- mtime 写 + 读 ---
    li   t0, 0x0200BFF8          # mtime 低半地址
    li   t1, 0xCAFEBABE
    sw   t1, 0(t0)               # mtime 低半 = 0xCAFEBABE
    li   t1, 0xFEEDFACE
    sw   t1, 4(t0)               # mtime 高半 = 0xFEEDFACE
    lw   a3, 0(t0)               # a3 = 0xCAFEBABE
    lw   a4, 4(t0)               # a4 = 0xFEEDFACE

    ecall                        # halt (M-mode ecall → cause 11 → triple fault)

# ===== 期望结果 =====
# 期望最终寄存器:
#   a0 (x10) = 0xDEADBEEF   mtimecmp[0] 低半
#   a1 (x11) = 0x12345678   mtimecmp[0] 高半
#   a2 (x12) = 0x00000001   msip[0] (写 1 读 1)
#   a3 (x13) = 0xCAFEBABE   mtime 低半
#   a4 (x14) = 0xFEEDFACE   mtime 高半
#
# halt 路径: ecall (cause 11 from M; mtvec=0 未设 → 跳 pc=0 不可执行 → 二次 inst
# access fault → 三次 → in_trap=3 triple fault halt)。
