# a_01_8 / 05_sv32_hw_managed_ad — 跟 04 同模板
# 见 stub.S 顶部 doc。

TOOL    := riscv64-unknown-elf
CC      := $(TOOL)-gcc
OBJCOPY := $(TOOL)-objcopy
CFLAGS  := -nostdlib -nostartfiles -ffreestanding -march=rv32im_zicsr_zifencei -mabi=ilp32 -mcmodel=medany
# -march=rv32im 同 04: 项目 RVC 完整实现是 a01_9 (7) 待办, 当前 decode 只识别
# C.LI / C.ADDI 两个 RVC; 全程禁 c 让 gcc 只发 32-bit 指令。

.PHONY: all clean
all: out.bin

out.elf: stub.S main.c link.ld
	$(CC) $(CFLAGS) -T link.ld stub.S main.c -o out.elf

out.bin: out.elf
	$(OBJCOPY) -O binary out.elf out.bin

clean:
	rm -f out.elf out.bin
