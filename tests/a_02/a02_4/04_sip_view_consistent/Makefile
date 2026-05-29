# a02_4 / 04_sip_view_consistent
# 产出 out.bin: sip 入口验证 — 跨 mip/sip 入口看同物理字段; sip 内 STIP/SEIP RO,
# SSIP RW (RV spec §5.1.4 + §3.1.9).

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
