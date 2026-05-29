# a02_4 / 01_mip_mie_basic
# 产出 out.bin: mie / mip / sip 基本读写 + WARL 截断 验证 fixture.
# 跑法: ./jit-emu out.bin; main dump 段验证 a0 / a1 / a2 / a3 寄存器值.
# 详 stub.S 顶部注释.

TOOL    := riscv64-unknown-elf
CC      := $(TOOL)-gcc
OBJCOPY := $(TOOL)-objcopy
CFLAGS  := -nostdlib -nostartfiles -march=rv32imac_zicsr_zifencei -mabi=ilp32 -mcmodel=medany

.PHONY: all clean
all: out.bin

out.elf: stub.S link.ld
	$(CC) $(CFLAGS) -T link.ld stub.S -o out.elf

out.bin: out.elf
	$(OBJCOPY) -O binary out.elf out.bin

clean:
	rm -f out.elf out.bin
