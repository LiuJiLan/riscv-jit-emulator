# a03_4 / 02_ring_full_producer_block
# 产出 out.bin: T6.2 plic_refresh_queue ring 满时 producer block 路径 (cond_timedwait
# not_full) 不死锁 + consumer 正确 drain. 单 source 连续 SET 200 次远超 16 ring slot.

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
