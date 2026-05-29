# a_01_9 / 01_m_ro_identity_csrs
# 见 stub.S 顶部 doc。
# 纯 .S 模板 (跟 fixture 02/03 同; a_01_8 04-09 的 .S+.c 模板留给真要 PT 建立的 fixture)。

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
