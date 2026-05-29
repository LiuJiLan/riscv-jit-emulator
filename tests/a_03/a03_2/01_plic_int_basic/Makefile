# a03_2 / 01_plic_int_basic
# 产出 out.bin: PLIC 完整状态机端到端 — enable + priority + threshold 配好,
# 经 test_dev MMIO 写触发 device_set_pending(3), 验 MEIP 合成 + claim/complete 全流程。

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
