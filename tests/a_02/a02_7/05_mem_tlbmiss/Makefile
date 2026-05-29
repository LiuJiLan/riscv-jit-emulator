# a02_7 / 05_mem_tlbmiss
# 产出 out.bin: Sv32 (S-mode) 下步进跨页扫 256 个数据页, 每次 load 故意 TLB miss,
# 测 mmu_walk 重走开销。.S+.c 模板 (跟 a02_2/04 同): stub.S + main.c 一起进 out.elf。

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
