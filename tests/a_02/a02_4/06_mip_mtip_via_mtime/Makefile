# a02_4 / 06_mip_mtip_via_mtime
# 产出 out.bin: mip.MTIP 合成路径验证 — (mtime ≥ mtimecmp[hartid]) 派生; clint_init
# mtimecmp 初值 UINT64_MAX 修复 spurious MTIP; 边界 `>=` 测试.

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
