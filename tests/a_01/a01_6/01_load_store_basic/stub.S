# a_01_6 / 01_load_store_basic
# 验证 a_01_6 load/store 基本读写 + sext/zext 区别:
#   - sw + lw 32-bit 往返 (验 store_helper 写 + load_helper 读 + 4-byte 对齐 OK)
#   - sb 0xFF + lb (sext) → -1 = 0xFFFFFFFF
#   - sb 同字节 + lbu (zext) → 0xFF
# 10 条指令 = 40 字节 (32-bit only, .option norvc)。
#
# 数据区: 用 RAM 内 [0x80001000, 0x80001008) 的 8 字节做读写 (RAM 是 mmap 上的, 任意地址都
# 可写; .text 段只到 0x80000028 左右, 离 0x80001000 还远不会冲突)。
#
# 期望 (main.c 末尾 fprintf):
#   x1 = 0x42       (= 66 in %u; 32-bit lw 读回)
#   x2 = 0xFFFFFFFF (= 4294967295 in %u; lb sign-ext -1)
#   x3 = 0xFF       (= 255 in %u; lbu zero-ext)
#   pc = 0          (triple fault 后停在 0)
#   in_trap = 3
#   mcause = 1, mtval = 0, mepc = 0, mtvec = 0
#     (字段保留第 2 次 trap = ecall 后 mmu fail at 0; cause=1 access fault on fetch)
#   total_count = 9
#     (block 1: lui+addi+sw+lw+addi+sb+lb+lbu+csrw mtvec(boundary) = 9 条;
#      block 2: ecall 触发 trap longjmp, count++ 不到, total 不变;
#      block 3-4: mmu fail x2 → in_trap 3 → 退出)
#
# 控制流追:
#   block 1 (0x80000000): 8 条算术/load/store + csrw mtvec(boundary)   count=9, total=9
#                          x1=0x42, x2=0xFFFFFFFF, x3=0xFF; mtvec=0
#   block 2 (0x80000024): ecall → trap longjmp                          total 不变 = 9
#                          in_trap 0→1, _cause=11, _epc=0x80000024, regs[0]=mtvec=0
#   block 3 (0x0):        mmu fail → trap_set_state(1, 0)               in_trap 1→2
#                          _cause=1, _tval=0, _epc=0; regs[0]=mtvec=0
#   block 4 (0x0):        mmu fail → trap (候选 A 早 return)             in_trap 2→3
#   while 退出, total_count=9

.section .text
.option norvc
.globl _start
_start:
    lui  x6,  0x80001       # 0x80000000: x6 = 0x80001000 (data base, RAM 内)

    # ---- 测试 1: sw + lw 32-bit 往返 ----
    addi x10, x0, 0x42      # 0x80000004: x10 = 0x42 (源)
    sw   x10, 0(x6)         # 0x80000008: mem[0x80001000] = 0x00000042 (4 字节)
    lw   x1,  0(x6)         # 0x8000000c: x1 = 0x42

    # ---- 测试 2: sb + lb sign-ext ----
    addi x11, x0, -1        # 0x80000010: x11 = 0xFFFFFFFF (低 8 位 = 0xFF)
    sb   x11, 4(x6)         # 0x80000014: mem[0x80001004] = 0xFF (1 字节)
    lb   x2,  4(x6)         # 0x80000018: x2 = sext(0xFF) = -1 = 0xFFFFFFFF

    # ---- 测试 3: lbu zero-ext (同字节, 验 helper 不做 sext) ----
    lbu  x3,  4(x6)         # 0x8000001c: x3 = zext(0xFF) = 0xFF

    # ---- 停机 (项目内部 triple fault 协议, 跟 a_01_5_c 05_triple_fault_halt 一致) ----
    csrw mtvec, x0          # 0x80000020: mtvec = 0 (准备 triple fault)
    ecall                   # 0x80000024: trap → 跳 0 → mmu fail x2 → triple fault 退出
