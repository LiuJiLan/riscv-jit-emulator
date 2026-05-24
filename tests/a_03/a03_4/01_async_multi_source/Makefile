# a03_4 / 01_async_multi_source
# 产出 out.bin: T6.2 PLIC 异步消费可见性 + 多 source 仲裁正确性 (不进 handler,
# 只验 PLIC pending bitmap + csrr mip 合成视图; 中断 fire 路径由 a_03_2 全套覆盖).

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
