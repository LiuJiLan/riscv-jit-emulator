# a02_5 / 03_int_mie_mask_off
# mstatus.MIE = 0 时不触发 trap (验全局 IE 屏蔽; trap_check_interrupt 内 deliver_mask = 0):
#   1. 设 mtimecmp 小 + mie.MTIE 但 不设 mstatus.MIE (默认 0)
#   2. mtime 由 timer 辅助线程异步累加越过 mtimecmp 后, trap_check_interrupt: mip.MTIP=1
#      但 mstatus.MIE=0 → deliver_mask = 0 → ready = 0 → 'c' (check no fire), 永不
#      trap 跳 mtvec
#   3. csrr mip 看到 MTIP=1 (验合成读跟 IE 屏蔽 解耦)
#   4. main 跑一段后直接 halt (没有 handler 进入)
#
# 跟 fixture 01 区别: 不调 csrrsi mstatus, MIE; 不进 m_handler; halt 走 main 自己的 ecall.
# trace 期望: 全程 'cf' (从不 't' fire). a0 验 csrr mip 时看到 MTIP=1 不被 IE 屏蔽掩盖.

.section .text
.option norvc
.globl _start
_start:
    # mie.MTIE = 0x80 (虽然 mstatus.MIE=0 不会 fire, 但保持 mie.MTIE 模拟"启用但屏蔽")
    li      t0, 0x80
    csrw    mie, t0

    # mtimecmp[0] = 10 (很小, 立即 pending)
    li      t0, 0x02004000
    li      t1, 10
    sw      t1, 0(t0)
    sw      x0, 4(t0)

    # 故意不设 mstatus.MIE (保持 0)

    # busy loop 让 mtime 步进; 不会 fire interrupt 因 mstatus.MIE=0
    li      t2, 0
    li      t3, 100                 # 跑 100 轮自旋
busy_loop:
    addi    t2, t2, 1
    blt     t2, t3, busy_loop

    # 验合成读: csrr mip 应看到 MTIP=1 (虽然 IE 屏蔽不 trap, 但合成读不动)
    csrr    a0, mip                 # 期望 a0 = 0x80 (MTIP bit 7 set, 其他 0)

    # 验 mcause 没变 (没有 trap 发生)
    csrr    a1, mcause              # 期望 a1 = 0

    # halt: ecall (cause=11; mtvec=0 默认 → fetch 0 fail → triple fault)
    ecall

# 期望最终寄存器:
#   a0 = 0x80                (mip.MTIP=1, 合成读看到; 但 IE 屏蔽不 trap)
#   a1 = 0                   (mcause; trap 没发生过 (handler ecall 第 2 次 trap 后 cause=11
#                              但本 fixture halt 之后才发生, csrr a1 时仍 0))
#
# 期望 trap dump:
#   in_trap = 3              (halt 的 ecall 触发 triple fault)
#   mcause  = 11             (halt 的 ecall_from_M; 字段保留第 2 次)
#
# 关键验证:
#   - 整个 busy_loop 期间没有 trap 跳 (a0 csrr 在 busy_loop 完成后才执行, 跟 mcause=0 一起证实)
#   - mip.MTIP 合成读跟 mstatus.MIE 解耦 (IE off 不影响 mip readout)
