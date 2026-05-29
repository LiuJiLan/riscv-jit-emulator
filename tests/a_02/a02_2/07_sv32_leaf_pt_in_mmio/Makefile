# a_02_2 / 07_sv32_leaf_pt_in_mmio
TOOL    := riscv64-unknown-elf
CC      := $(TOOL)-gcc
OBJCOPY := $(TOOL)-objcopy
CFLAGS  := -nostdlib -nostartfiles -ffreestanding -march=rv32im_zicsr_zifencei -mabi=ilp32 -mcmodel=medany

.PHONY: all clean
all: out.bin

out.elf: stub.S main.c link.ld
	$(CC) $(CFLAGS) -T link.ld stub.S main.c -o out.elf

out.bin: out.elf
	$(OBJCOPY) -O binary out.elf out.bin

clean:
	rm -f out.elf out.bin
