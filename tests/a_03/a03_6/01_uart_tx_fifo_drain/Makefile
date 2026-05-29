# a03_6 / 01_uart_tx_fifo_drain
# 产出 out.bin: UART TX FIFO async drain 验收 — hart 写 130 字节 (超 UART_FIFO_SIZE
# 128 上限, 后 2 字节 silent drop), spin csrr time 50 ms 让 drain thread drain 完,
# 读 LSR 期望 THRE=1 + TEMT=1 (FIFO 空); 验 LSR queue-based 反映 + drain 真正生效.

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
