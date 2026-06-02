# a03_14 / 03_wfi_irq_wakeup_race
#
# ===== 运行参数 =====
# RUN-BLK: disk.img
# RUN-EXPECT-EXIT: 0
# RUN-TIMEOUT: 5
# 注: debug+ASan 跑 ~300ms (release ~165ms, ~1.8x slowdown), 离默认 5s timeout
# 还远, 不用 RUN-RELEASE. ASan 顺便覆盖 wfi/plic/virtio-blk 跨线程 use-after-free
# 跟 atomic store 顺序异常等检查 — 比单跑 release 更值。
# 但 tests/review/out/riscv_jit_emulator 必须是 sticky kick 落地后的版本 (旧版
# debug frozen 撞 case A race 会 hang).
#
# ===== 测试目的 =====
# 重复 read virtio-blk sector 0 走 IRQ + wfi 路径, 验 wfi-IRQ wakeup race 闭合
# (session 026/027 fix: kick_sticky atomic flag, src/core/wfi.c)。
#
# race 触发链 (case A, session 027 trace 实证):
#   1. 软件 sw QueueNotify=0 → emulator io_worker drain → device_set_pending →
#      mip MEIP=1 + wfi_kick (此时 hart 还没进 wfi cond_wait, signal 落空)
#   2. dispatcher 主循环顶 trap_check 检 mip MEIP=1 → deliver trap
#      (mepc = wfi inst PC, 严格派 mepc=pc 实装)
#   3. handler ack: lw INT_STATUS → sw INT_ACK → device_clear_pending →
#      mip MEIP=0; sb done=1; mret
#   4. hart mret 回 wfi inst, 进 wfi_wait, predicate (mip & mie) = 0 → false
#   5. 无 sticky kick: cond_timedwait 500ms 兜底但 predicate 仍 false → 又
#      cond_wait → forever hang (session 026 baseline ~52%/5000 iter 触发)
#   6. 有 sticky kick: wfi_kick 在 step 1 atomic_store(sticky=1, release);
#      wfi_wait 入口 atomic_exchange(0, acquire) 消费, sticky_seen=true 立即退;
#      pc += 4 = 跳过 wfi → 软件 loop 顶 lbu done=1 → 出
#
# 单 fixture 单次跑触发率 ~50%/5000 iter (跟 tags/demo_test baseline 一致); 必
# 触发不靠 fixture, 靠 regression batch 跑多次 (CI 抓 flaky 率上升即 fix 退化)。
# fix 在: 跨多次 CI 跑稳过 0 hang. 详 notes/context/wfi_race_analysis.md 场景 2/3。
#
# CLINT timer mtimecmp 设 MAX 关掉 ambient 中断兜底, 让 case A race 在单源闭环
# 下显形 (跟 tags/demo_test 体例同, 不靠 timer 救命)。
#
# ===== 思路 =====
# 单 hart M-mode + PLIC source 1 (virtio-blk) + wfi 等 done flag.
# desc[0..2] 一次性 setup (header IN sector 0 + 512B data buf + status byte);
# avail.ring[0..7] 预填 0 (永远指 desc 0); 每 iter 只改 avail.idx (递增).
# m_handler: lw INT_STATUS → sw INT_ACK → PLIC complete → sb done=1 → mret.
# 主帧: for i = 0..N: done=0, idx+=1, NOTIFY=0, while(!done) wfi.
# 完成: lhu a6 = used.idx (期望 = N); sw 0x5555 to test_dev FINISHER → exit 0.
#
# 寄存器用法 (handler-safe):
#   s0 = current iter count (handler 不碰 s*, 安全跨 trap)
#   s1 = N (iter limit)
#   s2 = DONE_FLAG_ADDR (handler 用 t0/t1 临时, 不动 s2)
#   s3 = avail.idx (跨 iter 累计)
#   s4 = VRING_BASE (用作 sh imm 基址)
# handler 用 t0..t3, main 在 wait loop 期间不依赖 t* 任何持续状态 (每 iter 头
# 重 li 设地址, lbu 每轮 reload).

.equ PLIC_PRIO_BASE,    0x0C000000
.equ PLIC_ENABLE_BASE,  0x0C002000
.equ PLIC_THRESHOLD,    0x0C200000
.equ PLIC_CLAIM,        0x0C200004
.equ VBLK_BASE,         0x10001000

.equ CLINT_BASE,        0x02000000
.equ CLINT_MTIMECMP_LO, 0x02004000
.equ CLINT_MTIMECMP_HI, 0x02004004

.equ VBLK_REG_QUEUE_SEL,    0x030
.equ VBLK_REG_QUEUE_NUM,    0x038
.equ VBLK_REG_QUEUE_ALIGN,  0x03C
.equ VBLK_REG_QUEUE_PFN,    0x040
.equ VBLK_REG_QUEUE_NOTIFY, 0x050
.equ VBLK_REG_INT_STATUS,   0x060
.equ VBLK_REG_INT_ACK,      0x064
.equ VBLK_REG_STATUS,       0x070

.equ VBLK_STATUS_FULL,  0x0F

.equ VRING_BASE,        0x80100000
.equ VRING_AVAIL_OFF,   0x80                # avail flags @ 0x80, idx @ 0x82, ring[i] @ 0x84+2i
.equ VRING_USED_ADDR,   0x80101000

.equ HEADER_ADDR,       0x80100200
.equ DATA_BUF_ADDR,     0x80200000
.equ STATUS_ADDR,       0x80200200
.equ DONE_FLAG_ADDR,    0x80300000

.equ ITER_N,            5000

.section .text
.option norvc
.globl _start

_start:
    # ---- mtvec / mtimecmp=MAX 关 timer / mie+mstatus ----
    la      t0, m_handler
    csrw    mtvec, t0

    li      t0, CLINT_MTIMECMP_LO
    li      t1, 0xffffffff
    sw      t1, 0(t0)
    sw      t1, 4(t0)                       # mtimecmp_hi (0x4004)

    li      t0, 0x800                       # mie.MEIE bit 11
    csrw    mie, t0
    csrw    0x310, x0                       # mstatush 清 MDT (Smdbltrp prelude)
    csrrsi  x0, mstatus, 0x8                # mstatus.MIE

    # ---- PLIC: priority[1]=1 / threshold=0 / enable bit 1 ----
    li      t0, PLIC_PRIO_BASE
    li      t1, 1
    sw      t1, 4(t0)                       # priority[src=1] = 1
    li      t0, PLIC_THRESHOLD
    sw      x0, 0(t0)
    li      t0, PLIC_ENABLE_BASE
    li      t1, 0x02                        # enable bit 1
    sw      t1, 0(t0)

    # ---- 配 queue ----
    li      t0, VBLK_BASE
    sw      x0, VBLK_REG_QUEUE_SEL(t0)
    li      t1, 8
    sw      t1, VBLK_REG_QUEUE_NUM(t0)
    li      t1, 4096
    sw      t1, VBLK_REG_QUEUE_ALIGN(t0)
    li      t1, 0x80100                     # PFN → base 0x80100000
    sw      t1, VBLK_REG_QUEUE_PFN(t0)

    # ---- desc[0]: header @ HEADER_ADDR, len=16, flags=NEXT, next=1 ----
    li      t0, VRING_BASE
    li      t1, HEADER_ADDR
    sw      t1, 0(t0)
    sw      x0, 4(t0)
    li      t1, 16
    sw      t1, 8(t0)
    li      t1, 0x00010001                  # flags=NEXT | (next=1 << 16)
    sw      t1, 12(t0)

    # ---- desc[1]: data buf, len=512, flags=NEXT|WRITE, next=2 ----
    li      t1, DATA_BUF_ADDR
    sw      t1, 16(t0)
    sw      x0, 20(t0)
    li      t1, 512
    sw      t1, 24(t0)
    li      t1, 0x00020003                  # flags=NEXT|WRITE(3) | (next=2 << 16)
    sw      t1, 28(t0)

    # ---- desc[2]: status, len=1, flags=WRITE ----
    li      t1, STATUS_ADDR
    sw      t1, 32(t0)
    sw      x0, 36(t0)
    li      t1, 1
    sw      t1, 40(t0)
    li      t1, 0x00000002                  # WRITE
    sw      t1, 44(t0)

    # ---- header: type=IN(0), reserved=0, sector=0 ----
    li      t0, HEADER_ADDR
    sw      x0, 0(t0)                       # type = 0 (VIRTIO_BLK_T_IN)
    sw      x0, 4(t0)
    sw      x0, 8(t0)                       # sector lo
    sw      x0, 12(t0)                      # sector hi

    # ---- avail.flags=0, avail.idx=0, ring[0..7]=0 (永远指 desc 0) ----
    li      t0, VRING_BASE
    sh      x0, VRING_AVAIL_OFF(t0)         # flags @ 0x80
    sh      x0, VRING_AVAIL_OFF + 2(t0)     # idx   @ 0x82
    sh      x0, VRING_AVAIL_OFF + 4(t0)     # ring[0]
    sh      x0, VRING_AVAIL_OFF + 6(t0)
    sh      x0, VRING_AVAIL_OFF + 8(t0)
    sh      x0, VRING_AVAIL_OFF + 10(t0)
    sh      x0, VRING_AVAIL_OFF + 12(t0)
    sh      x0, VRING_AVAIL_OFF + 14(t0)
    sh      x0, VRING_AVAIL_OFF + 16(t0)
    sh      x0, VRING_AVAIL_OFF + 18(t0)

    # ---- used 起步清 0 ----
    li      t0, VRING_USED_ADDR
    sw      x0, 0(t0)
    sw      x0, 4(t0)

    # ---- vblk Status: ACK|DRIVER|FEATURES_OK|DRIVER_OK ----
    li      t0, VBLK_BASE
    li      t1, VBLK_STATUS_FULL
    sw      t1, VBLK_REG_STATUS(t0)

    # ---- 主循环初始 ----
    li      s0, 0                           # iter counter
    li      s1, ITER_N                      # iter limit
    li      s2, DONE_FLAG_ADDR              # done flag base (handler 也用同地址)
    li      s3, 0                           # avail.idx running
    li      s4, VRING_BASE                  # for sh s3, 0x82(s4)

loop_top:
    # ---- iter setup: done=0, status=0xff, idx++, NOTIFY ----
    sb      x0, 0(s2)                       # done = 0

    li      t0, STATUS_ADDR
    li      t1, 0xff
    sb      t1, 0(t0)                       # status_byte = 0xff (in progress)

    addi    s3, s3, 1
    sh      s3, VRING_AVAIL_OFF + 2(s4)     # avail.idx = s3

    li      t0, VBLK_BASE
    sw      x0, VBLK_REG_QUEUE_NOTIFY(t0)   # NOTIFY → race window opens

wait_iter:
    lbu     t1, 0(s2)                       # 每轮 reload done (handler 可能改)
    bnez    t1, iter_done
    wfi                                     # case A race: 进 wfi 前 handler 已跑完
                                            # 时, sticky kick 救场让 wfi 立即退
    j       wait_iter

iter_done:
    addi    s0, s0, 1
    blt     s0, s1, loop_top

    # ---- 全 N 完成: 验 used.idx ----
    li      t0, VRING_USED_ADDR
    lhu     a6, 2(t0)                       # used.idx, 期望 = N

    # ---- halt: sifive FINISHER PASS → exit 0 ----
    li      t0, 0x00100000                  # TEST_DEV_BASE
    li      t1, 0x5555                      # FINISHER_PASS
    sw      t1, 0(t0)
1:  j       1b                              # 防 fall-through

m_handler:
    # handler 只用 t0..t3 (caller-save, main 在 wait loop 期间不依赖 t* 持续值);
    # 不动 s* (跨 trap 寄存器 main 主帧依赖).
    li      t0, PLIC_CLAIM
    lw      t1, 0(t0)                       # claim (期望 1)

    li      t2, VBLK_BASE
    lw      t3, VBLK_REG_INT_STATUS(t2)
    sw      t3, VBLK_REG_INT_ACK(t2)        # → device_clear_pending → mip MEIP=0

    sw      t1, 0(t0)                       # PLIC complete

    li      t0, DONE_FLAG_ADDR
    li      t1, 1
    sb      t1, 0(t0)                       # done = 1 (main 的 wait_iter lbu 看)
    mret

# ===== 期望结果 =====
# 寄存器:
#   s0 = 5000                (iter 完成数)
#   s1 = 5000                (iter limit)
#   s3 = 5000                (累计 avail.idx)
#   a6 = 5000 mod 65536      (used.idx, virtio 16-bit 计数器; 5000 < 65536 = 5000)
#
# 退出: test_dev sifive FINISHER PASS → shell 退出码 $? = 0
#
# 不期望:
#   - 任何 hang (RUN-TIMEOUT: 5 兜底; 触发 race 且 sticky kick 缺失 → 进
#     wfi cond_wait 永等 → 5s timeout 砍 → test runner FAIL)
#   - 任何 [bus] CAUSE_*_ACCESS_FAULT
#   - 程序崩溃
