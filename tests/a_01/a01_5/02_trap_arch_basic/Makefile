# a_01_5_b / 02_trap_arch_basic
# 产出 out.bin: 24 字节, 主程序 4 条 + handler 2 条, loader 加载到 GUEST_RAM_START。
#
# 验证目标 (a_01_5_b 阶段):
#   - trap_set_state 写 xcause/xtval/xepc 字段对 (cause=2 illegal, tval=raw_inst, epc=触发 PC)
#   - regs[0] = xtvec[PRIV_M] 跳 trap handler 真生效 (block 跳到 0x80000010 跑 addi x3)
#   - mmu_translate_pc 路径不命中 (xtvec=0x80000010 在 RAM 内, fetch 不 fail)
#   - 候选 A 早 return: 第 3 次 trap 字段保留第 2 次状态 (mtval=0x73 ecall raw, 不是再次覆盖)
#   - dispatcher while(in_trap < 3) 退出条件: 主程序 .word 0 + handler 2 次 ecall = 3 次 trap
#
# 期望:
#   x1 = 1   (主程序第一条 addi 跑过)
#   x2 = 0   (主程序第二条 addi 不应跑 — .word 0 触发 trap 截断主程序)
#   x3 = 7   (handler 内 addi x3, x0, 7 跑过, 第二/第三轮 handler 重复设同值)
#   pc = 0x80000014   (block 3 内 addi 跑完后 regs[0] += 4 = 0x14; ecall 触发第 3 次 trap;
#                       候选 A 早 return 不动 regs[0]; 所以 regs[0] 停在第 3 次 trap 的触发 PC)
#   trap dump: in_trap=3 mcause=2 mtval=0x73 mepc=0x80000014 mtvec=0x80000010
#                       (字段保留第 2 次 deliver 的状态: 第 3 次早 return 不写字段)
#
# dispatcher 行为:
#   block 1 (0x80000000): addi x1 + .word 0 → trap (in_trap 0→1)
#                          deliver: regs[0] = xtvec = 0x80000010; count_out=1
#   block 2 (0x80000010): addi x3 (regs[0] += 4 → 0x14) + ecall → trap (in_trap 1→2)
#                          deliver: 字段更新 (cause=2 tval=0x73 epc=0x14); regs[0] = xtvec
#                          = 0x80000010; count_out=1
#   block 3 (0x80000010): addi x3 (regs[0] += 4 → 0x14) + ecall → trap (in_trap 2→3)
#                          候选 A 早 return: 不写字段, 不动 regs[0]; regs[0] 停在 0x14
#                          count_out=1
#   while (3 < 3) false → 退出, total_count = 3, regs[0] = 0x80000014
#
# 工具链 / 模式: 套 a01_4 + 01_csr_basic 同款。
# build flow: stub.S → out.elf → out.bin (objcopy 提 raw bytes)。

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
