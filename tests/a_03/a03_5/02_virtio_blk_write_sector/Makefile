# a03_5 / 02_virtio_blk_write_sector
# 产出 out.bin + disk.img: virtio-blk 写后读自闭环 — 先 write sector 0 byte
# pattern, 再 read 同 sector 验落盘. disk.img 全零起点 (不依赖 host 端预填).

TOOL    := riscv64-unknown-elf
CC      := $(TOOL)-gcc
OBJCOPY := $(TOOL)-objcopy
CFLAGS  := -nostdlib -nostartfiles -march=rv32imac -mabi=ilp32 -mcmodel=medany

.PHONY: all clean
all: out.bin disk.img

out.elf: stub.S link.ld
	$(CC) $(CFLAGS) -T link.ld stub.S -o out.elf

out.bin: out.elf
	$(OBJCOPY) -O binary out.elf out.bin

# disk.img: 4 KB image (8 sector) 全零起点; stub.S 自己写 pattern + 读回比对.
disk.img:
	dd if=/dev/zero of=disk.img bs=512 count=8 status=none

clean:
	rm -f out.elf out.bin disk.img
