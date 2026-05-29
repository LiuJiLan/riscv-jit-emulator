# a03_3 / 04_uart_rx_echo
#
# ===== 运行参数 =====
# RUN-STDIN: input.txt
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# UART RX 中断 PLIC 联调 — 验 reader thread → RX FIFO → device_set_pending(10) →
# PLIC 仲裁 → MEIP 合成 → handler pop RBR + echo THR + complete 全链路:
#   1. 主帧 init PLIC source 10 + UART LCR=0x03 + IER.ERBFI=1
#   2. 主帧 spin wait_loop (等 s0=3 跳出)
#   3. 外部 pipe 注入字符 ("abc"); reader thread poll → read 字节 → push RX FIFO →
#      uart_compute_device_line → (ERBFI && rx_cnt>0) = true → device_set_pending(10)
#   4. dispatcher trap_check_interrupt → MEIP fire → m_handler
#   5. m_handler:
#        - 验 mcause = 0x8000000B
#        - lw claim → 10
#        - lb RBR (pop 一字节; 若 FIFO 还有字节, device_line 仍 1)
#        - sb 同字节 THR (echo 到 stdout)
#        - sw complete = 10
#        - s0++; mret
#   6. FIFO 空 → device_line=0 → 主帧 s0=3 跳出 → test_dev FINISHER 干净停机
#
# ===== 思路 =====
# RX 中断"读 IIR 不清"语义: pop RBR (lb 走 size=1 path) 是唯一清 RX 中断的方式;
# handler 不需要读 IIR. handler 走 sb THR echo 时, IER.ETBEI=0 不会触发 TX 中断.
#
# 主帧 spin wait_loop 无 timeout — pipe EOF 后 reader 退出, hart spin 检 s0 < 3 会
# hang; 因此外部跑必加 timeout (Makefile 注释段写跑法).
#
# li t0, 3 + bne s0, t0 — bne 是 hard block boundary, fetch 段结束让 dispatcher 重判
# 中断; 跟 a_03_2 fixture beq trigger 同套路.

.equ PLIC_PRIO_BASE,    0x0C000000
.equ PLIC_ENABLE_BASE,  0x0C002000
.equ PLIC_THRESHOLD,    0x0C200000
.equ PLIC_CLAIM,        0x0C200004
.equ UART_BASE,         0x10000000

.equ UART_REG_RBR,      0
.equ UART_REG_THR,      0
.equ UART_REG_IER,      1
.equ UART_REG_LCR,      3

.section .text
.option norvc
.globl _start

_start:
    # ---- M init: mtvec / mie.MEIP / mstatus.MIE ----
    la      t0, m_handler
    csrw    mtvec, t0
    li      t0, 0x800               # mie.MEIP
    csrw    mie, t0
    csrw    0x310, x0               # mstatush 清 MDT (A.7 prelude; 否则下条 MIE 写被强制 0)
    csrrsi  x0, mstatus, 0x8        # mstatus.MIE = 1

    # ---- 配 PLIC: priority[10]=1 ----
    li      t0, PLIC_PRIO_BASE
    li      t1, 1
    sw      t1, 40(t0)

    # ---- 配 PLIC: threshold[ctx0]=0 ----
    li      t0, PLIC_THRESHOLD
    sw      x0, 0(t0)

    # ---- 配 PLIC: enable[ctx0][word0] bit 10 = 1 ----
    li      t0, PLIC_ENABLE_BASE
    li      t1, 0x400
    sw      t1, 0(t0)

    # ---- 配 UART: LCR=0x03 / IER bit 0 (ERBFI) = 1 ----
    li      t0, UART_BASE
    li      t1, 0x03
    sb      t1, UART_REG_LCR(t0)
    li      t1, 0x01                # ERBFI bit
    sb      t1, UART_REG_IER(t0)

    # ---- s0 = RX 计数; s1 = \n done flag; a5 = spin counter (调试用) ----
    li      s0, 0
    li      s1, 0
    li      a5, 0

# 主帧"或条件"退出: (s1 != 0 \n 触发) OR (s0 >= 3 字符阈值触发).
# 两路设计动机: pipe 注入 'abc' (无 \n) 走 s0 阈值; 手测 cooked TTY Enter 走 \n.
# 修法 a (单独 s0 阈值) 跟修法 b (单独 \n) 都各有 use case 退不掉的盲区, 或条件兼容.
# RX 密集 IRQ 让 s0 跳过 3 也 OK — bltu 用 < 不用 != 检测.
wait_loop:
    addi    a5, a5, 1
    bnez    s1, done                # \n 触发 → 跳出
    li      t0, 3
    bltu    s0, t0, wait_loop       # s0 < 3 spin; s0 >= 3 fall through 跳出

done:
    # halt: test_dev sifive FINISHER PASS → 干净停机, exit_code=0
    # 注: 替代旧 triple-fault 停机 (Smdbltrp MDT abort 后不再可用)
    li      t0, 0x00100000          # TEST_DEV_BASE
    li      t1, 0x5555              # FINISHER_PASS
    sw      t1, 0(t0)               # sifive FINISHER off 0
1:  j       1b                      # 防 fall-through: SRS=0 下个 block 边界退

m_handler:
    # 验 mcause
    csrr    s2, mcause

    # PLIC claim ctx 0
    li      t0, PLIC_CLAIM
    lw      s3, 0(t0)               # 期望 s3 = 10

    # 读 UART RBR (pop 一字节)
    li      t1, UART_BASE
    lb      t2, UART_REG_RBR(t1)    # t2 = 收到字符

    # echo: sb 写回 THR (TX 路径; IER.ETBEI=0 不拉 TX 中断)
    sb      t2, UART_REG_THR(t1)

    # PLIC complete = 10
    sw      s3, 0(t0)

    # 计数 +1; 检 \n 触发 done flag
    addi    s0, s0, 1
    li      t3, 0x0a                # '\n' = LF
    beq     t2, t3, m_handler_set_done
    mret

m_handler_set_done:
    li      s1, 1                   # 收到 \n, 让主帧跳 done
    mret

# ===== 期望结果 =====
# 两路退出条件 (或):
#
# (a) pipe 注入路径: `printf 'abc' | jit-emu out.bin` (timeout 包装防 hang)
#     s0 = 3 (字符阈值触发, 'abc' 无 \n)
#     s1 = 0
#     stdout = "abc"
#
# (b) cooked TTY 手测路径: Console 键盘输入 "xyz" + Enter (Enter 自动转 \n)
#     s0 >= 1 (至少 1 字符, 含 \n; 用户长输入时 s0 可能 = N 远 > 3)
#     s1 = 1 (\n 触发)
#     stdout = 用户输入回显 (例 "xyz\n")
#
# 注: RX 中断密集 fire 时 s0 可能跳过 3 直接到 N, bltu 用 < 而不是 != 比较 — s0 >= 3
#     fall through 即可, 不靠精确等于 3.
#
# 期望最终寄存器 (两路共通):
#   s2 = 0x8000000B   (mcause)
#   s3 = 10           (claim_src)
#   a5 > 0
#
# 期望停机 (test_dev sifive FINISHER PASS):
#   shell 退出码 $? = 0   (clean 退出; reader 启动竞态 / pipe EOF 时外部 timeout 包装防 hang)
#   [dispatcher] halted: ... (SRS=0 退主循环)
#
# 不期望:
#   - 程序 hang (timeout 命中 — 说明 reader / PLIC fanout / 中断 deliver 哪环断了)
#   - [bus] CAUSE_*_ACCESS_FAULT (UART/PLIC 不该产 fault)
#   - stdout 漏字符大段 (reader push 漏 / handler 漏处理 — FIFO 满 silent drop 偶有几个字节
#     是可接受的, 短输入 (≤ 16 字节) 不该 drop)
