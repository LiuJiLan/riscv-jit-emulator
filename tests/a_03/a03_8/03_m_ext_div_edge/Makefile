# a03_8 / 03_m_ext_div_edge
# 验 RV32M div/rem spec edge cases (RV Unprivileged § 7.2):
#   - DIV/DIVU/REM/REMU by-0 各定值不 trap
#   - DIV/REM INT_MIN ÷ -1 overflow 各定值不 trap

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
