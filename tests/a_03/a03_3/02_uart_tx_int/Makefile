# a03_3 / 02_uart_tx_int
# 产出 out.bin: UART TX 中断端到端 — 经 IER.ETBEI=1 触发 device_set_pending(10),
# 验 PLIC 仲裁 + MEIP 合成 + handler 读 IIR ack THR empty + complete 全流程;
# 顺手输出 "OK\n" 到 stdout (TX 路径走通验证).

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
