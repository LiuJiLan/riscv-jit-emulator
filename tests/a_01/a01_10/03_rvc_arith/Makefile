# a_01_10 / 03_rvc_arith
# 见 stub.S 顶部 doc。a_01_10 (7) step 1 RVC 算术 fixture (~16 case).
# 模板套 a_01_3/02_arith_compressed (-march=rv32imac, c.* mnemonic 强制 RVC).
# 纯 .S 模板, 无 main.c (BARE M-mode, 无 SV32 / 无 PT).

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
