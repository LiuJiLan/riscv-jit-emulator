# a02_2 / 01_mmio_basic
# 产出 out.bin: CLINT MMIO 读写基础验证 fixture (T1 阶段)。
# 跑法: ./jit-emu out.bin; main dump 段验证 a0 / a1 / a2 / a3 / a4 寄存器值。
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
