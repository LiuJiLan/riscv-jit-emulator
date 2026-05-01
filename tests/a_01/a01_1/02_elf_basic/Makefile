# 02_elf_basic
# 产出 out.elf: ELF32 / EM_RISCV / ET_EXEC / 单 PT_LOAD @ 0x80000000。
# 是 happy path: loader 应返 0, 16 字节落在 GUEST_RAM_START 处。

TOOL   := riscv64-unknown-elf
CC     := $(TOOL)-gcc
CFLAGS := -nostdlib -nostartfiles -march=rv32imac -mabi=ilp32 -mcmodel=medany

.PHONY: all clean
all: out.elf

out.elf: stub.S link.ld
	$(CC) $(CFLAGS) -T link.ld stub.S -o out.elf

clean:
	rm -f out.elf
