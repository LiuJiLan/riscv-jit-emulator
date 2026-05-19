# a02_6 / 02_clint_timer_auto_fire
# 产出 out.bin: 验 mtimecmp → MTIP → trap deliver 端到端真异步 fire (不依赖
# dispatcher 主帧指令数推进; 跟 a02_5/01 同形态, 但 mtimecmp 设跨多个 timer wake)。

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
