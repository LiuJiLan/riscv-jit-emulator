# a03_3 / 03_uart_hello
# 产出 out.bin: hello world 纯 TX 轮询 — 不接 PLIC / 不接 trap, 单线 lb LSR 检 THRE +
# sb THR 字符循环; 验 UART TX fast path + stdout 输出. milestone 顺手副产物.

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
