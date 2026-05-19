# a02_6 / 01_clint_mtime_async
# 产出 out.bin: 验 timer 辅助线程异步累加 mtime — 读两次 CLINT mtime MMIO,
# poll 等到值变, 验 delta > 0 (mtime 真跟 host wall clock 推进, 不靠 dispatcher 主帧)。

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
