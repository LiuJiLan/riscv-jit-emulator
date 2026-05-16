# a02_3 / 02_s_mret_reject
# 产出 out.bin: S-mode 调 MRET reject fixture (a_02 session_005 P5; S < M 触发
# PRIV_CHECK_OR_TRAP(PRIV_M) cause 2 illegal + tval=raw_inst)。
# 跑法: ./jit-emu out.bin; main dump 段验证 a0 / a1 / s0 寄存器值。
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
