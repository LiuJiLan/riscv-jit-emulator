# a_01_10 / 02_sum_store_fast_path
# 见 stub.S 顶部 doc。跟 fixture 01 (load 路径 SUM 验证) 互补 — 02 验 store 路径。
# .S+.c 模板, 编译 stub.S + main.c 一起进 out.elf。

TOOL    := riscv64-unknown-elf
CC      := $(TOOL)-gcc
OBJCOPY := $(TOOL)-objcopy
CFLAGS  := -nostdlib -nostartfiles -ffreestanding -march=rv32im_zicsr_zifencei -mabi=ilp32 -mcmodel=medany
# 注: -march=rv32im (去 a + c) — 同 a_01_8 / a_01_10 fixture 系列, RVC 完整后 cleanup。

.PHONY: all clean
all: out.bin

out.elf: stub.S main.c link.ld
	$(CC) $(CFLAGS) -T link.ld stub.S main.c -o out.elf

out.bin: out.elf
	$(OBJCOPY) -O binary out.elf out.bin

clean:
	rm -f out.elf out.bin
