# a04_1 / 02_fence_reserved_funct3_reject
# 产出 out.bin: 0x0F + funct3=010 (reserved) 触发 illegal trap, handler 验
# mcause=2 + mtval=raw, test_dev PASS. 配 01_fence_basic 正反双覆盖 decode 0x0F case.

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
