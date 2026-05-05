# a_01_6 / 03_store_oor_reject
# 验证 store_helper 的 RAM 边界检查: sw 到 RAM 之外 (0x40000000) → trap_raise(7 store access fault)。
# stderr 还会有一行 "[lsu] store: PA 0x40000000 not in RAM (MMIO bus_dispatch not implemented in
# a_01_6)" 由 store_helper 的 fprintf 输出 (a_01_6 占位提示, 未来 a_01_7+ 接 bus_dispatch 时替换)。
# 11 条指令 (la 算 2 条) = 44 字节 (32-bit only, .option norvc)。
#
# 触发条件 (lsu.c store_helper Step 2 BARE 路径):
#   (uint32_t)(pa - GUEST_RAM_START) >= GUEST_RAM_SIZE
# pa=0x40000000, GUEST_RAM_START=0x80000000, GUEST_RAM_SIZE=128MB:
#   (0x40000000 - 0x80000000) cast u32 = 0xC0000000 = 3 GiB > 128 MiB → access fault cause=7。
#
# RAM 区是 [0x80000000, 0x88000000); 0x40000000 显然不在 (在 RAM 起点之下)。
# misalign 不会撞 (0x40000000 是 4-byte 对齐)。
#
# 期望:
#   x1 = 7          (mcause = store/AMO access fault, RV spec §3.1.16)
#   x2 = 0x40000000 (mtval = 触发 store 的 GVA)
#   x3 = 0x80000014 (mepc = sw 指令的 PC)
#   pc = 0
#   in_trap = 3
#   mcause = 11     (字段保留第 2 次 trap = handler 末 ecall 覆盖)
#   mtval = 0
#   mepc = 0x80000028 (handler 末 ecall PC)
#   mtvec = 0
#   total_count = 7 (跟 02_load_misalign_reject 同形态)
#
# 字节地址 (la 是 pseudo, 展开 auipc+addi 占 8 字节):
#   _start: 0x00 la(8) → 0x00-0x07; 0x08 csrw mtvec; 0x0c lui; 0x10 addi; 0x14 sw (trap here)
#   handler 起点 = 0x18:
#       0x18 csrr x1 mcause; 0x1c csrr x2 mtval; 0x20 csrr x3 mepc; 0x24 csrw mtvec,x0; 0x28 ecall
#
# 控制流追 (带 0x80000000 RAM 起点偏移):
#   block 1 (RAM+0x00): la(2)+csrw mtvec(boundary)       count=3,  total=3
#                        mtvec = 0x80000018
#   block 2 (RAM+0x0c): lui+addi+sw → store_helper trap   count++ 不到, total=3
#                        in_trap 0→1, _cause=7, _tval=0x40000000, _epc=RAM+0x14, regs[0]=mtvec=RAM+0x18
#   block 3 (RAM+0x18): csrr x1, mcause(boundary)         count=1,  total=4   x1=7
#   block 4 (RAM+0x1c): csrr x2, mtval(boundary)          count=1,  total=5   x2=0x40000000
#   block 5 (RAM+0x20): csrr x3, mepc(boundary)           count=1,  total=6   x3=0x80000014
#   block 6 (RAM+0x24): csrw mtvec, x0(boundary)          count=1,  total=7   mtvec=0
#   block 7 (RAM+0x28): ecall → trap longjmp              count++ 不到, total=7
#                        in_trap 1→2, _cause=11, _epc=RAM+0x28
#   block 8 (0x0):      mmu fail → trap                   in_trap 2→3 候选 A
#   while 退出, total_count=7

.section .text
.option norvc
.globl _start
_start:
    la   x4,  handler       # 0x00-0x07: auipc + addi (la pseudo)
    csrw mtvec, x4          # 0x08:      mtvec = handler

    # ---- 触发 store oor (PA 0x40000000 不在 RAM) ----
    lui  x6,  0x40000       # 0x0c:      x6 = 0x40000000 (不在 RAM, 4-byte 对齐)
    addi x10, x0, 0x42      # 0x10:      x10 = 0x42 (要 store 的值)
    sw   x10, 0(x6)         # 0x14:      sw 0x40000000 → store_helper access fault cause=7
    # ↑ trap_raise_exception _Noreturn longjmp; 不到此处

handler:
    csrr x1,  mcause        # 0x18:  x1 = mcause = 7 (store/AMO access fault)
    csrr x2,  mtval         # 0x1c:  x2 = mtval = 0x40000000
    csrr x3,  mepc          # 0x20:  x3 = mepc = 0x80000014 (sw 的 PC)

    # ---- 停机 ----
    csrw mtvec, x0          # 0x24:  mtvec = 0
    ecall                   # 0x28:  trap → 跳 0 → mmu fail x2 → triple fault 退出
