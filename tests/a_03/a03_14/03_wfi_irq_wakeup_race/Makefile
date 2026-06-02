# a03_14 / 03_wfi_irq_wakeup_race
# 产出 out.bin + disk.img: 反复 read virtio-blk sector 0 N=5000 走 IRQ+wfi
# 路径, 验 sticky kick race fix 闭合 (session 026/027).

TOOL    := riscv64-unknown-elf
CC      := $(TOOL)-gcc
OBJCOPY := $(TOOL)-objcopy
CFLAGS  := -nostdlib -nostartfiles -march=rv32imac_zicsr_zifencei -mabi=ilp32 -mcmodel=medany

.PHONY: all clean
all: out.bin disk.img

out.elf: stub.S link.ld
	$(CC) $(CFLAGS) -T link.ld stub.S -o out.elf

out.bin: out.elf
	$(OBJCOPY) -O binary out.elf out.bin

# disk.img: 1 sector (512 B) 全零起点; fixture 只 read sector 0, 不验内容,
# 任意内容都 OK. 用 dd 起最小镜像.
disk.img:
	dd if=/dev/zero of=disk.img bs=512 count=1 status=none

clean:
	rm -f out.elf out.bin disk.img
