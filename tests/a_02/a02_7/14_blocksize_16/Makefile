# a02_7 / 14_blocksize_16
# 产出 out.bin: bare 纯算术紧循环, block = 16 指令; block 大小 sweep (12-16) 之一。

TOOL    := riscv64-unknown-elf
CC      := $(TOOL)-gcc
OBJCOPY := $(TOOL)-objcopy
CFLAGS  := -nostdlib -nostartfiles -march=rv32im_zicsr_zifencei -mabi=ilp32 -mcmodel=medany

.PHONY: all clean
all: out.bin

out.elf: stub.S link.ld
	$(CC) $(CFLAGS) -T link.ld stub.S -o out.elf

out.bin: out.elf
	$(OBJCOPY) -O binary out.elf out.bin

clean:
	rm -f out.elf out.bin
