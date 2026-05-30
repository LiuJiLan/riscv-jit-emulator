# a03_14 / 01_clint_ipi
# 产出 out.bin: 2-hart CLINT msip IPI 端到端 — hart 0 写 msip[1] 触发 hart 1
# MSIP, hart 1 trap handler 收 + 清 msip[1] + 写 done flag; 双 hart 都 PASS
# (test_dev sifive FINISHER 幂等)。项目第一个跨 hart fixture。

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
