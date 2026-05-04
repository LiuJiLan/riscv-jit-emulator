# a_01_4 / 02_jal_jalr_call
# 产出 out.bin: 24 字节, 5 条 RV32I + 1 条 ecall, loader 加载到 GUEST_RAM_START。
# 验证方式: 跑 ./jit-emu out.bin, 测函数调用模式 (jal call + jalr return) 端到端正确。
#           main.c 末尾 fprintf 寄存器, 期望 x1 = 0x8000000c (jal 写的返回地址),
#           x2 = 12 (= 7 + 5, sub 内 add 算的结果), x3 = 5 (未动)。
# 工具链 / 模式: 套 a01_3 / 01_arith_basic + a01_4 / 01_branch_loop 同款。

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
