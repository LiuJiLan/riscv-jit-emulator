# a03_5 / 01_virtio_blk_read_sector
# 产出 out.bin + disk.img: virtio-blk legacy v1.0 读 sector 0 端到端 — guest
# 端摆 vring + desc 链 + sw QueueNotify → worker drain pread → 写 used + 触发
# IRQ → handler claim/ack/complete; 主帧验 data buf 头 4B == 0xdeadbeef.

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

# disk.img: 4 KB image (8 sector), sector 0 前 4 byte 填 0xdeadbeef
# (LE byte 顺 'ef be ad de'); 跟 stub.S `lw a7, 0(DATA_BUF)` 期望 0xdeadbeef
# (RV32 LE) 配对. 用 POSIX octal escape (\NNN) — dash printf 不解析 \xHH.
disk.img:
	dd if=/dev/zero of=disk.img bs=512 count=8 status=none
	printf '\357\276\255\336' | dd of=disk.img bs=1 count=4 conv=notrunc status=none

clean:
	rm -f out.elf out.bin disk.img
