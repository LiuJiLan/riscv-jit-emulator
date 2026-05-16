# a02_2 / 02_bus_unmapped_reject
# 产出 out.bin: bus_dispatch_* 未命中 reject fixture (T1 阶段; bus 自产 access fault)。
# 跑法: ./jit-emu out.bin; main dump 段验证 a0 / a1 / a2 / s0 寄存器值。
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
