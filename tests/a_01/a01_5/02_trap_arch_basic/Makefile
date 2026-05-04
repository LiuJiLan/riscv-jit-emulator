# a_01_5_c / 02_trap_arch_basic
# 产出 out.bin: 36 字节, 起步 3 条 (la+csrw 设 mtvec) + 主程序 4 条 + handler 2 条;
# loader 加载到 GUEST_RAM_START。
#
# 验证目标 (a_01_5_c 阶段):
#   - csr.c 真接读写: csrw mtvec 设 handler 地址 (a_01_5_b 由 main.c hard-code, a_01_5_c
#     起 fixture 自包含, main.c 不再设 xtvec)
#   - trap_set_state 写 xcause/xtval/xepc 字段对
#   - regs[0] = xtvec[PRIV_M] 跳 trap handler 真生效 (跳到 0x8000001c 跑 addi x3)
#   - 候选 A 早 return: 第 3 次 trap 字段保留第 2 次状态
#   - dispatcher while(in_trap < 3) 退出条件 + sigsetjmp 一次性落点 + trap_raise_exception
#     _Noreturn longjmp 路径
#
# 期望:
#   x1 = 1           (主程序第一条 addi 跑过)
#   x2 = 0           (.word 0 触发 trap 截断主程序, addi x2 不应跑)
#   x3 = 7           (handler 内 addi x3 跑过)
#   pc = 0x80000020  (block 4 内 addi x3 跑完 pc += 4 = 0x20, ecall 触发第 3 次 trap 候选 A
#                      早 return 不动 pc; pc 停在 ecall PC)
#   trap dump: in_trap=3 mcause=11 mtval=0 mepc=0x80000020 mtvec=0x8000001c
#              (字段保留第 2 次 trap = handler ecall 的; mtvec 由 fixture csrw 写)
#
# dispatcher 行为:
#   block 1 (0x80000000): la(2) + csrw(boundary)
#                          mtvec = 0x8000001c; count_out=3, total_count += 3 = 3
#   block 2 (0x8000000c): addi x1 + .word 0 → trap_raise(2,0) longjmp
#                          (in_trap 0→1, _cause=2/_tval=0/_epc=0x10, regs[0]=mtvec=0x1c)
#                          longjmp 跳过 dispatcher 的 total_count += 行, total_count 不变
#   block 3 (0x8000001c handler): addi x3 + ecall → trap_raise(11,0) longjmp
#                          (in_trap 1→2, _cause=11/_tval=0/_epc=0x20, regs[0]=mtvec=0x1c)
#                          total_count 不变
#   block 4 (0x8000001c handler): addi x3 + ecall → trap_raise(11,0) longjmp
#                          (in_trap 2→3, 候选 A 早 return; 字段保留第 2 次)
#                          total_count 不变
#   while (3<3) false → 退出, total_count = 3
#
# 注: a_01_5_c 起 trap_raise_exception 是 _Noreturn longjmp, 跳过 dispatcher 的
# total_count += local_count 行; 所以 trap 触发的 block 内"trap 之前已跑的指令" 不计入
# total_count。这是与 a_01_5_b 的行为差异 (a_01_5_b 时 trap_raise return + caller
# goto out + fetch loop 末尾 *count_out=count + dispatcher total_count += 都正常执行)。
#
# 工具链 / 模式: 套 a01_4 + 01_csr_basic 同款。

TOOL    := riscv64-unknown-elf
CC      := $(TOOL)-gcc
OBJCOPY := $(TOOL)-objcopy
CFLAGS  := -nostdlib -nostartfiles -march=rv32imac -mabi=ilp32 -mcmodel=medany

.PHONY: all clean
all: out.bin

out.elf: stub.S link.ld
	$(CC) $(CFLAGS) -T link.ld stub.S -o out.elf

out.bin: out.elf
	$(OBJCOPY) -O binary out.elf out.bin

clean:
	rm -f out.elf out.bin
