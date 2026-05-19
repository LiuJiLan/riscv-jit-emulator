# a02_6 / 03_perf_estimate
# 产出 out.bin: 长跑工作循环 (100K 轮 add/sw/lw/branch 混合), 读 mtime 前后做
# delta 算 effective MIPS — debug build 解释器粗略性能估算 (release + JIT 接入
# 后跑同 fixture 做对比, 性能数据进论文)。

TOOL    := riscv64-unknown-elf
CC      := $(TOOL)-gcc
OBJCOPY := $(TOOL)-objcopy
CFLAGS  := -nostdlib -nostartfiles -march=rv32imac -mabi=ilp32 -mcmodel=medany

.PHONY: all clean
all: out.bin

out.elf: stub.S link.ld
	$(CC) $(CFLAGS) -T link.ld stub.S -o out.elf

out.bin: out.elf
	$(OBJCOPY) -O binary out.elf out.bin

clean:
	rm -f out.elf out.bin
