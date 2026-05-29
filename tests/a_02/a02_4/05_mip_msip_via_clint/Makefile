# a02_4 / 05_mip_msip_via_clint
# 产出 out.bin: mip.MSIP 合成路径验证 — 写 CLINT.msip[0] = 1 → csrr mip 看见 MSIP=1
# (类 (5) 异步源 OR 合成读). csrw mip 不能清 MSIP (RO in mip).

TOOL    := riscv64-unknown-elf
CC      := $(TOOL)-gcc
OBJCOPY := $(TOOL)-objcopy
CFLAGS  := -nostdlib -nostartfiles -march=rv32imac_zicsr_zifencei -mabi=ilp32 -mcmodel=medany

.PHONY: all clean
all: out.bin

out.elf: stub.S link.ld
	$(CC) $(CFLAGS) -T link.ld stub.S -o out.elf

out.bin: out.elf
	$(OBJCOPY) -O binary out.elf out.bin

clean:
	rm -f out.elf out.bin
