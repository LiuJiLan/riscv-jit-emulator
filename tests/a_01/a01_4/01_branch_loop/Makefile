# a_01_4 / 01_branch_loop
# 产出 out.bin: 24 字节, 5 条 RV32I 算术/分支 + 1 条 ecall, loader 加载到 GUEST_RAM_START。
# 验证方式: 跑 ./jit-emu out.bin, dispatcher while(1) 多块循环, branch 命中 boundary 后
#           退出 fetch loop 进下一轮; 末尾 ecall 触发 OP_UNSUPPORTED, count=0, dispatcher
#           break; main.c 末尾 fprintf 寄存器, 期望 x2 = 5 (循环累加 5 次), x1 = 0 (倒数到 0)。
# 工具链 / 模式: 套 a01_3 / 01_arith_basic 同款。

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
