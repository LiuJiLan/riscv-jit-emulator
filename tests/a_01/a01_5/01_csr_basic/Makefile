# a_01_5_a / 01_csr_basic
# 产出 out.bin: 32 字节, 1 addi + 6 csr 变体 + 1 ecall, loader 加载到 GUEST_RAM_START。
#
# 验证目标 (a_01_5_a 阶段):
#   - decode 路径: 6 个 csr op_kind 都路由对 (Step 1 + decode_test 已验)
#   - csr_op 大 helper 分发对 (stderr 出 6 read 占位 + 5 write 占位; csrrsi zimm=0 不写)
#   - 硬边界路径: 每条 csr 独立 block, dispatcher 多次重派发 (total_count=7)
#   - x10 = 0x42 是唯一非零寄存器 (csr 写源值 / read 占位都返 0, x1..x6 全 0)
#
# 期望 stderr (顺序):
#   [csr_a stub] read mtvec -> 0
#   [csr_a stub] write mtvec = 0x00000042       ; csrrw read_old=0 → write x10=0x42
#   [csr_a stub] read mstatus -> 0
#   [csr_a stub] write mstatus = 0x00000042     ; csrrs read_old=0, set 0x42 → 0x42
#   [csr_a stub] read mcause -> 0
#   [csr_a stub] write mcause = 0x00000000      ; csrrc read_old=0, clear 0x42 → 0
#   [csr_a stub] read mepc -> 0
#   [csr_a stub] write mepc = 0x00000007        ; csrrwi zimm=7 → write 7
#   [csr_a stub] read mtval -> 0                ; csrrsi zimm=0 不写 (RV §2.1.2)
#   [csr_a stub] read mtvec -> 0
#   [csr_a stub] write mtvec = 0x00000000       ; csrrci read_old=0, clear 0x1F → 0
#   [dispatcher] halted: total_count=7 pc=0x8000001c
#   [main] result: pc=0x8000001c x1=0 x2=0 x3=0
#
# 工具链 / 模式: 套 a01_4 同款。-march=rv32imac (不加 _zicsr 子集后缀) 在当前 binutils
# 版本下默认含 csr 指令 (zicsr 在较老 binutils 里隐含进 base, 不能用 z 前缀显式声明 —
# 加了会报 "unsupported ISA subset 'z'")。新版 binutils 支持 _zicsr 显式但与旧版不兼容,
# 这里保留与 a01_4 一致。
# build flow: stub.S → out.elf → out.bin (objcopy 提 raw bytes); 与 a01_3 / a01_4 一致。

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
