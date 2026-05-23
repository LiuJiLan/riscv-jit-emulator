# a03_3 / 01_test_dev_basic
# 产出 out.bin: test_dev MMIO fanout 端到端 — 经 sw TEST_DEV_SET 触发
# device_set_pending(3), 验 PLIC 仲裁 + MEIP 合成 + claim/complete 全流程。

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
