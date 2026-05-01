# 06_not_elf
# 产出 out.elf: 实际是文本文件, 但后缀 .elf 强制 main.c 走 guest_load_elf 路径。
# 期望: loader 读完 52 字节 Elf32_Ehdr 后做 magic 比对, 不匹配 ELFMAG,
#       fprintf "not an ELF file (bad magic)" 并 return -1。
#
# 文件必须 >= 52 字节 (sizeof Elf32_Ehdr), 否则 read_full_at 会先 EOF, 走不到 magic 检查
# (那种太短的情况由 07_tiny_file_reject 覆盖)。
# 下面这段刚好 57 字节, 安全过 52 阈值。

.PHONY: all clean
all: out.elf

out.elf:
	printf 'This is plain text. Loader should reject by magic check.\n' > out.elf

clean:
	rm -f out.elf
