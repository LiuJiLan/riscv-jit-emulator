# a04_3 / 05_lr_sc_device_dma
# T4 端到端 — virtio_blk DMA 写 RAM 后 lrsc_on_device_write 清 reservation,
# guest 后续 SC.W 必失败 (RV spec §14.2.1 line 108-109).

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

# disk.img: 4 KB (8 sector), sector 0 前 4 byte = 0xDEADBEEF (LE 'ef be ad de').
# guest LR.W @ DATA_BUF 在 pread 完后读到 0xDEADBEEF; SC.W 想写 0x99999999, 必
# 失败 (lrsc_on_device_write 已清 reservation); SC 后验 mem 仍是 0xDEADBEEF 而非
# 0x99999999. POSIX octal escape (\NNN) — dash printf 不解析 \xHH.
disk.img:
	dd if=/dev/zero of=disk.img bs=512 count=8 status=none
	printf '\357\276\255\336' | dd of=disk.img bs=1 count=4 conv=notrunc status=none

clean:
	rm -f out.elf out.bin disk.img
