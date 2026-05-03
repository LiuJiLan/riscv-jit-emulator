# a01_3 / 01_arith_basic
# 产出 out.bin: 16 字节, 4 条 RV32I 算术 + 1 条 ecall, loader 加载到 GUEST_RAM_START。
# 验证方式: 跑 ./jit-emu out.bin, dispatcher 内 interpret_one_block 跑完 3 条算术后
#           ecall 触发 OP_UNSUPPORTED, break 回 dispatcher; main.c 末尾 fprintf 寄存器,
#           期望 x3 = 50 (= 42 + 8), pc = 0x8000000c (ecall 地址)。
# 工具链 / 模式: 套 a01_1 / 01_bin_basic 同款, 完全一致。

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
