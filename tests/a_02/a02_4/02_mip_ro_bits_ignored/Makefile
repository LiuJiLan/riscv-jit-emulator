# a02_4 / 02_mip_ro_bits_ignored
# 产出 out.bin: csrw mip 写 RO 位 (MSIP/MTIP/MEIP) 验证忽略 fixture.
# 跑法: ./jit-emu out.bin; main dump 段验证 a0 寄存器值.

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
