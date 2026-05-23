# a03_1 / 01_plic_mmio_skeleton
# 产出 out.bin: PLIC MMIO 骨架 — 对 priority / pending / enable / context 四区
# 各访问一次, 验证 plic_read/write 地址解码 fprintf 输出对。

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
