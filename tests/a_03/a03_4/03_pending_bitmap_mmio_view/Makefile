# a03_4 / 03_pending_bitmap_mmio_view
# 产出 out.bin: PLIC MMIO pending bitmap 读路径完整验证 (T6.2 post-EIP atomic
# cache 后 — 过去 a_03 fixture 全套无人专门测此路径, 现 cache 加入顺手补).
# 覆盖: 空 baseline + 跨 word (96 src / 3 word) + claim 后清 + complete 后重出现 + CLEAR.

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
