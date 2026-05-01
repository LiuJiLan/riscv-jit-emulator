# 03_elf_multi_seg
# 产出 out.elf: 两个 PT_LOAD, 各 8 字节, 落在不同地址。
# 验证: gpa_to_hva_offset + 0x80000000  应见 EF BE AD DE  EF BE AD DE
#       gpa_to_hva_offset + 0x80100000  应见 BE BA FE CA  BE BA FE CA

TOOL   := riscv64-unknown-elf
CC     := $(TOOL)-gcc
CFLAGS := -nostdlib -nostartfiles -march=rv32imac -mabi=ilp32 -mcmodel=medany

.PHONY: all clean
all: out.elf

out.elf: stub.S link.ld
	$(CC) $(CFLAGS) -T link.ld stub.S -o out.elf

clean:
	rm -f out.elf
