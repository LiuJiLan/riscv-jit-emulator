# 05_bin_oversize_reject
# 产出 out.bin: 200MB 全 0 文件, 超出 GUEST_RAM_SIZE (128MB)。
# 无 .S / .ld —— 不需要编译, dd 直接生成。
# 期望: loader 在 size 检查处 fprintf "file too big: bytes=..., available=..." 并 return -1。

.PHONY: all clean
all: out.bin

out.bin:
	dd if=/dev/zero of=out.bin bs=1M count=200 status=none

clean:
	rm -f out.bin
