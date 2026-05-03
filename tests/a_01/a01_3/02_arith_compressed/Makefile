# a01_3 / 02_arith_compressed
# 产出 out.bin: 10 字节, 3 条 RVC 算术 + 1 条 32-bit ecall。
# 验证 decode 走 RVC 路径 + fetch loop +=2 推进 + interpreter 真执行 RVC 算术。
# 期望: x1=11 / x2=5 / pc=0x80000006 / dispatcher count=3。
# 与 01_arith_basic 不同之处: 不加 .option norvc, stub.S 用 c.* 显式 mnemonic 强制 RVC 编码。
# 工具链 / 模板: 与 01_arith_basic 完全一致 (套 a01_1 fixture 模板)。

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
