# a03_3 / 06_sifive_test_fail
# 产出 out.bin: sw (42<<16)|0x3333 → test_dev sifive_test FINISHER FAIL magic →
# main return 42. shell 验证: ./jit-emu out.bin; echo $? 应为 42.

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
