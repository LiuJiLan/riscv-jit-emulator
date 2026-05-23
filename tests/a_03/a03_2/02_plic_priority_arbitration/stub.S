# a03_2 / 02_plic_priority_arbitration
#
# ===== 测试目的 =====
# PLIC claim 仲裁按 priority 排序 (RV PLIC spec §7): 同时 set src=3 (prio=2) 和
# src=5 (prio=5), 第 1 次 claim 期望返 src=5 (高 prio 优先); complete 5 后 src=3
# 仍 pending, 第 2 次 fire 时 claim 期望返 src=3.
#
# 同 prio 选小 src_id (spec §7) — 走 fixture 06 单独覆盖, 此处只验 prio 差异仲裁.
#
# ===== 期望结果 =====
#   s0 = 1            (wait_loop 退出 flag, handler 第 2 次设)
#   s1 = 5            (第 1 次 claim, prio 高的 src=5)
#   s2 = 3            (第 2 次 claim, 剩下的 src=3)
#   s3 = 2            (handler 进入次数, 期望 2)
#   in_trap = 3 / pc = 0  (halt path triple fault)

.equ CSR_TOHOST, 0x800
.section .text
.option norvc
.globl _start

_start:
    la      t0, m_handler
    csrw    mtvec, t0
    li      t0, 0x800
    csrw    mie, t0

    # PLIC: priority[3]=2, priority[5]=5
    li      t0, 0x0C000000
    li      t1, 2;   sw t1, 12(t0)        # priority[3] = 2
    li      t1, 5;   sw t1, 20(t0)        # priority[5] = 5

    # threshold[ctx0] = 0
    li      t0, 0x0C200000;  sw x0, 0(t0)

    # enable[ctx0][word0] = (1<<3) | (1<<5) = 0x28
    li      t0, 0x0C002000;  li t1, 0x28;  sw t1, 0(t0)

    # init counters/flags 必须在 csrw 0x800 之前 (避免 mret 后被 reset)
    li      s0, 0
    li      s1, 0
    li      s2, 0
    li      s3, 0

    # set src=3 + src=5
    li      t0, 0x80000003;  csrw CSR_TOHOST, t0
    li      t0, 0x80000005;  csrw CSR_TOHOST, t0

    # 开 MIE → 下一 block trap_check fire MEIP
    csrrsi  x0, mstatus, 0x8

wait_loop:
    beqz    s0, wait_loop

    csrw    mtvec, x0
    ecall                                  # halt

m_handler:
    addi    s3, s3, 1                       # 进入次数 ++

    # lw PLIC claim
    li      t0, 0x0C200004
    lw      t1, 0(t0)

    # 按进入次数分发记录
    li      t2, 1
    beq     s3, t2, first_claim
    li      t2, 2
    beq     s3, t2, second_claim
    j       handler_exit                    # 第 3 次不该发生 (防御)

first_claim:
    mv      s1, t1                          # 期望 s1 = 5
    # device_clear src=5 + complete 5
    li      t0, 5;   csrw CSR_TOHOST, t0
    li      t0, 0x0C200004
    sw      s1, 0(t0)
    mret                                    # 回 wait_loop; src=3 仍 pending → 再 fire

second_claim:
    mv      s2, t1                          # 期望 s2 = 3
    li      t0, 3;   csrw CSR_TOHOST, t0
    li      t0, 0x0C200004
    sw      s2, 0(t0)
    li      s0, 1                           # 退出 wait_loop
    mret

handler_exit:
    li      s0, 1
    mret
