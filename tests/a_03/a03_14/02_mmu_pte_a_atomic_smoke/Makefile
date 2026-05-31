# a03_14 / 02_mmu_pte_a_atomic_smoke
# 产出 out.bin: 冒烟验 mmu_walk PTE.A atomic OR set + 跨 hart 可见性 (session 024
# 末落地 atomic_fetch_or 替换 plain memcpy)。
# stub.S + main.c (setup_pt 在 C 里写, 跟 a02_7/08 体例同)。

TOOL    := riscv64-unknown-elf
CC      := $(TOOL)-gcc
OBJCOPY := $(TOOL)-objcopy
CFLAGS  := -nostdlib -nostartfiles -march=rv32imac_zicsr_zifencei -mabi=ilp32 -mcmodel=medany

.PHONY: all clean
all: out.bin

out.elf: stub.S main.c link.ld
	$(CC) $(CFLAGS) -T link.ld stub.S main.c -o out.elf

out.bin: out.elf
	$(OBJCOPY) -O binary out.elf out.bin

clean:
	rm -f out.elf out.bin
