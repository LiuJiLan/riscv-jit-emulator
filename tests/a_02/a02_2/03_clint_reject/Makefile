# a02_2 / 03_clint_reject
# 产出 out.bin: CLINT 内部 reject fixture (4 write branch + 1 read sample)。
# 跑法: ./jit-emu out.bin; main dump 段验证 a0..a5 + s0 + s1 寄存器值。
# 详 stub.S 顶部注释。

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
