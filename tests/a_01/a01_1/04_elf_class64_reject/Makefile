# 04_elf_class64_reject
# 产出 out.elf: ELFCLASS64 / EM_RISCV / ET_EXEC。
# 期望: loader 在 class 检查处 fprintf "ELFCLASS64 not supported in current build (RV32 only)"
#       并 return -1。

TOOL   := riscv64-unknown-elf
CC     := $(TOOL)-gcc
# 注意是 rv64 那套, 这是 reject 测试的关键差异
CFLAGS := -nostdlib -nostartfiles -march=rv64imac -mabi=lp64 -mcmodel=medany

.PHONY: all clean
all: out.elf

out.elf: stub.S link.ld
	$(CC) $(CFLAGS) -T link.ld stub.S -o out.elf

clean:
	rm -f out.elf
