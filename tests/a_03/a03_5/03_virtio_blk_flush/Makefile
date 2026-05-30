# a03_5 / 03_virtio_blk_flush
# 产出 out.bin + disk.img: virtio-blk VIRTIO_BLK_F_FLUSH / T_FLUSH 闭环。
# disk.img = 4 KB 空白可写盘 (round 1 OUT 写 sector 0, round 2 FLUSH fsync)。

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

# disk.img: 4 KB (8 sector) 空白可写盘
disk.img:
	dd if=/dev/zero of=disk.img bs=512 count=8 status=none

clean:
	rm -f out.elf out.bin disk.img
