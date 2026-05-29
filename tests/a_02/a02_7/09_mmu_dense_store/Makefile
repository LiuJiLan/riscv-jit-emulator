# a02_7 / 09_mmu_dense_store
# 产出 out.bin: Sv32 (S-mode) 下纯 store 紧循环 (每轮 8 sw, 单热页 TLB 命中), 测
# SV32 store 命中路径吞吐。.S+.c 模板 (跟 a02_2/04 同): stub.S + main.c 进 out.elf。

TOOL    := riscv64-unknown-elf
CC      := $(TOOL)-gcc
OBJCOPY := $(TOOL)-objcopy
CFLAGS  := -nostdlib -nostartfiles -ffreestanding -march=rv32im_zicsr_zifencei -mabi=ilp32 -mcmodel=medany

.PHONY: all clean
all: out.bin

out.elf: stub.S main.c link.ld
	$(CC) $(CFLAGS) -T link.ld stub.S main.c -o out.elf

out.bin: out.elf
	$(OBJCOPY) -O binary out.elf out.bin

clean:
	rm -f out.elf out.bin
