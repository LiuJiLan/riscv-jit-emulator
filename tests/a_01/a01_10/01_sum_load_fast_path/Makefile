# a_01_10 / 01_sum_load_fast_path
# 见 stub.S 顶部 doc。
# .S+.c 模板 (跟 a_01_8 fixture 04 同形态): 编译 stub.S + main.c 一起进 out.elf。

TOOL    := riscv64-unknown-elf
CC      := $(TOOL)-gcc
OBJCOPY := $(TOOL)-objcopy
CFLAGS  := -nostdlib -nostartfiles -ffreestanding -march=rv32im -mabi=ilp32 -mcmodel=medany
# 注: -march=rv32im (去 a + c) — 项目 RVC 完整实现是 a01_9 (7) 待办, 当前 decode 只识别
# C.LI / C.ADDI 两个 RVC, main.c 内 setup_pt() gcc 默认会发 c.addi / c.sw / c.lui 等
# 不支持的 RVC; 全程禁 c 让 gcc 只发 32-bit 指令。a 扩展也去掉 (fixture 不用 LR/SC)。

.PHONY: all clean
all: out.bin

out.elf: stub.S main.c link.ld
	$(CC) $(CFLAGS) -T link.ld stub.S main.c -o out.elf

out.bin: out.elf
	$(OBJCOPY) -O binary out.elf out.bin

clean:
	rm -f out.elf out.bin
