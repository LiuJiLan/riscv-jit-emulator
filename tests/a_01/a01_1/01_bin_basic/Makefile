# 01_bin_basic
# 产出 out.bin: 16 字节, 应被 loader 加载到 GUEST_RAM_START 处。
# 验证方式: CLion 调试到 ram_init/loader 之后, 检查 gpa_to_hva_offset + 0x80000000 处
#           应见 EF BE AD DE  EF BE AD DE  78 56 34 12  78 56 34 12 (LE)。

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
