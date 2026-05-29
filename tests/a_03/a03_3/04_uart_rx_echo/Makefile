# a03_3 / 04_uart_rx_echo
# 产出 out.bin: UART RX echo pipe 注入 — 经 IER.ERBFI=1 接 RX 中断, handler pop RBR +
# 写回 THR (echo); 验 RX 链路 (reader thread → FIFO → device_set_pending → PLIC → handler).
#
# 跑法:
#   1. 自动化 (我代跑): printf 'abc' | /tmp/jit-emu-rel out.bin
#      (timeout 包装防 hang; pipe 注入 3 字符后 EOF, reader thread 退; hart spin 等
#      s0=3 跳出. 期望 stdout: "abc")
#   2. 手测 (user): CLion Run Config "Redirect input from" 指 input.txt, 或 Console
#      直接键盘输入 "abc"+Enter (cooked 模式行缓冲, Enter 后 'a''b''c''\n' 4 字符全送;
#      多收一个 \n 不影响 — fixture 只等 s0=3 跳出, 跑完前 \n 也被 handler echo 出去)
#   3. debug trace: 跑 Debug 配置看 DEBUG_TRACE 打点 (RX push / device_line / claim 链路)

TOOL    := riscv64-unknown-elf
CC      := $(TOOL)-gcc
OBJCOPY := $(TOOL)-objcopy
CFLAGS  := -nostdlib -nostartfiles -march=rv32imac_zicsr_zifencei -mabi=ilp32 -mcmodel=medany

.PHONY: all clean
all: out.bin input.txt

out.elf: stub.S link.ld
	$(CC) $(CFLAGS) -T link.ld stub.S -o out.elf

out.bin: out.elf
	$(OBJCOPY) -O binary out.elf out.bin

# input.txt: 喂 fixture stdin 的 3 字符 "abc" (无换行 — handler echo 满 3 个
# s0=3 跳出正常停机, 不靠 timeout)。RUN-STDIN tag 指向它 (stub.S 运行参数段)。
input.txt:
	printf 'abc' > input.txt

clean:
	rm -f out.elf out.bin input.txt
