# a02_3 / 01_u_priv_reject
# 产出 out.bin: U-mode 调 SYSTEM priv-required 指令 (MRET/SRET/SFENCE.VMA) reject
# fixture (a_02 session_005 P5; PRIV_CHECK_OR_TRAP 触发 cause 2 illegal + tval=raw_inst)。
# 跑法: ./jit-emu out.bin; main dump 段验证 a0..a5 / s0 寄存器值。
# 详 stub.S 顶部注释。

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
