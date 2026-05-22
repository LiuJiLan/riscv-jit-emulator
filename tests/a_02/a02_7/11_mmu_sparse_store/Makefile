# a02_7 / 11_mmu_sparse_store
# 产出 out.bin: Sv32 (S-mode) 下纯 store 紧循环 (每轮 8 sw 撞 8 个同 index 的页 →
# 条条 TLB miss), 测 SV32 store miss 重走路径吞吐。.S+.c 模板 (跟 a02_2/04 同)。

TOOL    := riscv64-unknown-elf
CC      := $(TOOL)-gcc
OBJCOPY := $(TOOL)-objcopy
CFLAGS  := -nostdlib -nostartfiles -ffreestanding -march=rv32im -mabi=ilp32 -mcmodel=medany

.PHONY: all clean
all: out.bin

out.elf: stub.S main.c link.ld
	$(CC) $(CFLAGS) -T link.ld stub.S main.c -o out.elf

out.bin: out.elf
	$(OBJCOPY) -O binary out.elf out.bin

clean:
	rm -f out.elf out.bin
