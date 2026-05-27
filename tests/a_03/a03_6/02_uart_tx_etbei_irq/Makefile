# a03_6 / 02_uart_tx_etbei_irq
# 产出 out.bin: UART TX async drain ETBEI 中断验收 — hart 写 32 字节 + 开 IER.ETBEI,
# drain thread drain 完 tx_count→0 后清 acked + uart_compute_device_line 触发 PLIC
# set_pending(10), handler 读 IIR 验 THR_EMP code (= 0xC2). 验 TX 中断真实联动 +
# drain 完 acked 重置路径.

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
