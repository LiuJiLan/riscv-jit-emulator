# b_02_5 / 01_counter_threshold
# 产出 out.bin: 24 字节, 5 条 RV32I + ecall, loader 加载到 GUEST_RAM_START.
# 验证方式: 跑 ./riscv_jit_emulator --bios out.bin, trace 前段 ~100 iter '._' 无 'J',
#           后段 ~100 iter '._J' 验证 b_02 T5 heat counter 真消费 + COMPILE_THRESHOLD=100
#           触发 jit_compile_block 路径走通. 期望 x2=200, total_count=606.
# 工具链 / 模式: 套 a01_4 / 01_branch_loop 同款.

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
