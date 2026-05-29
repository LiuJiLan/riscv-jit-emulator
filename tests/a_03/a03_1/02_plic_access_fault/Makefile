# a03_1 / 02_plic_access_fault
# 产出 out.bin: PLIC 空洞 / 越界访问 reject 验证 (PLIC 自产 access fault)。
# 跑法: ./jit-emu out.bin; main dump 验 s0 / s2 / s3 寄存器值 (5 case 计数 + cause 分流)。
# 详 stub.S 顶部注释。

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
