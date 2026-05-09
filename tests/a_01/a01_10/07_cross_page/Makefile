# a_01_10 / 07_cross_page
# 见 stub.S 顶部 doc。BARE M-mode 跨页保护 fixture (interpreter.c page_mask check 触发).
# 模板套 a_01_10/03_rvc_arith (-march=rv32imac, BARE 纯 .S, 无 main.c).

TOOL    := riscv64-unknown-elf
CC      := $(TOOL)-gcc
OBJCOPY := $(TOOL)-objcopy
CFLAGS  := -nostdlib -nostartfiles -ffreestanding -march=rv32imac -mabi=ilp32 -mcmodel=medany

.PHONY: all clean
all: out.bin

out.elf: stub.S link.ld
	$(CC) $(CFLAGS) -T link.ld stub.S -o out.elf

out.bin: out.elf
	$(OBJCOPY) -O binary out.elf out.bin

clean:
	rm -f out.elf out.bin
