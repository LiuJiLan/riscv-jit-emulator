# 07_tiny_file_reject
# 产出 out.elf: 0 字节空文件。后缀 .elf 强制走 guest_load_elf 路径。
# 期望: loader 第一次 pread Elf32_Ehdr 立刻 EOF (offset=0, remaining=52),
#       fprintf "unexpected EOF reading Elf32_Ehdr at offset 0, 52 bytes left" 并 return -1。
#
# 与 06_not_elf 的区别:
#   06 是 47 字节文本(EOF 在中途, 还差 5 字节)
#   07 是 0 字节(EOF 在起点, 一字节都没读到)
# 两者走同一段 read_full_at EOF 处理代码, 但触发条件不同, 算补完性测试。

.PHONY: all clean
all: out.elf

out.elf:
	printf '' > out.elf

clean:
	rm -f out.elf
