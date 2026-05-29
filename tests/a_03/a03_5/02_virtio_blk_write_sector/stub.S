# a03_5 / 02_virtio_blk_write_sector
#
# ===== 运行参数 =====
# RUN-BLK: disk.img
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# virtio-blk 写后读自闭环 — 两笔 IO 验落盘:
#   阶段 1: header.type=OUT + data buf 填 0xa5a5a5a5 (头 16 B) + sector=0
#           → sw QueueNotify → worker pwrite 512 B 进 disk.img sector 0 →
#           used.idx=1 + IRQ → handler s0=1
#   阶段 2: header.type=IN + 清 data buf → 同 sector → sw QueueNotify →
#           worker pread 512 B 从 disk.img sector 0 → 写回 data buf →
#           used.idx=2 + IRQ → handler s0=2
#   主帧验: data buf 头 4 byte == 0xa5a5a5a5 (说明先写后读拿到了刚写的内容).
#
# disk.img 全零起点 (Makefile dd if=/dev/zero); 不需 host 预填.
# 单 fixture 自闭环不依赖 host 端 image 比对.
#
# ===== 思路 =====
# 跟 fixture 01 同 vring 布局 (queue_num=8, qalign=4096);
# 两笔 IO 共用同一 desc[0..2] 链, 但 desc[1].flags 阶段 1 = NEXT (no WRITE),
# 阶段 2 = NEXT|WRITE (按 virtio spec: device-readable 在前, device-writable
# 在后; emulator 实装不强校验 WRITE bit, 但 fixture 守 spec); header.type 字段
# 决定真正 IO 方向.
#
# m_handler 不分阶段, 每次 +1 累加 s0 (s0=1 后阶段 2 才进, s0=2 收尾).

.equ PLIC_PRIO_BASE,    0x0C000000
.equ PLIC_ENABLE_BASE,  0x0C002000
.equ PLIC_THRESHOLD,    0x0C200000
.equ PLIC_CLAIM,        0x0C200004
.equ VBLK_BASE,         0x10001000

.equ VBLK_REG_MAGIC,        0x000
.equ VBLK_REG_QUEUE_SEL,    0x030
.equ VBLK_REG_QUEUE_NUM,    0x038
.equ VBLK_REG_QUEUE_ALIGN,  0x03C
.equ VBLK_REG_QUEUE_PFN,    0x040
.equ VBLK_REG_QUEUE_NOTIFY, 0x050
.equ VBLK_REG_INT_STATUS,   0x060
.equ VBLK_REG_INT_ACK,      0x064
.equ VBLK_REG_STATUS,       0x070

.equ VBLK_STATUS_FULL,      0x0F

.equ VRING_BASE,            0x80100000
.equ VRING_AVAIL_OFF,       0x80             # < 2047, 可直接 imm(reg)
.equ VRING_USED_ADDR,       0x80101000       # used 区超 12-bit imm, 单独 li

.equ HEADER_ADDR,           0x80100200
.equ DATA_BUF_ADDR,         0x80200000
.equ STATUS_ADDR,           0x80200200

.equ PATTERN,               0xa5a5a5a5

.section .text
.option norvc
.globl _start

_start:
    # ---- M init ----
    la      t0, m_handler
    csrw    mtvec, t0
    li      t0, 0x800               # mie.MEIP
    csrw    mie, t0
    csrw    0x310, x0               # mstatush 清 MDT (A.7 prelude; 否则下条 MIE 写被强制 0)
    csrrsi  x0, mstatus, 0x8        # mstatus.MIE

    # ---- PLIC: priority[1]=1 / threshold=0 / enable bit 1 ----
    li      t0, PLIC_PRIO_BASE
    li      t1, 1
    sw      t1, 4(t0)
    li      t0, PLIC_THRESHOLD
    sw      x0, 0(t0)
    li      t0, PLIC_ENABLE_BASE
    li      t1, 0x02
    sw      t1, 0(t0)

    # ---- 配 queue ----
    li      t0, VBLK_BASE
    sw      x0, VBLK_REG_QUEUE_SEL(t0)
    li      t1, 8
    sw      t1, VBLK_REG_QUEUE_NUM(t0)
    li      t1, 4096
    sw      t1, VBLK_REG_QUEUE_ALIGN(t0)
    li      t1, 0x80100
    sw      t1, VBLK_REG_QUEUE_PFN(t0)

    # ---- 摆 desc[0]: header @ HEADER_ADDR, len=16, flags=NEXT, next=1 ----
    li      t0, VRING_BASE
    li      t1, HEADER_ADDR
    sw      t1, 0(t0)
    sw      x0, 4(t0)
    li      t1, 16
    sw      t1, 8(t0)
    li      t1, 0x00010001          # flags=NEXT | (next=1 << 16)
    sw      t1, 12(t0)

    # ---- 摆 desc[1]: data buf, len=512, flags=NEXT (阶段 1 OUT; no WRITE), next=2 ----
    li      t1, DATA_BUF_ADDR
    sw      t1, 16(t0)
    sw      x0, 20(t0)
    li      t1, 512
    sw      t1, 24(t0)
    li      t1, 0x00020001          # flags=NEXT(1) | (next=2 << 16)
    sw      t1, 28(t0)

    # ---- 摆 desc[2]: status, len=1, flags=WRITE (no NEXT) ----
    li      t1, STATUS_ADDR
    sw      t1, 32(t0)
    sw      x0, 36(t0)
    li      t1, 1
    sw      t1, 40(t0)
    li      t1, 0x00000002
    sw      t1, 44(t0)

    # ---- 阶段 1: 摆 header (type=OUT=1, sector=0), 填 data buf pattern ----
    li      t0, HEADER_ADDR
    li      t1, 1                   # type = VIRTIO_BLK_T_OUT
    sw      t1, 0(t0)
    sw      x0, 4(t0)               # reserved
    sw      x0, 8(t0)               # sector lo
    sw      x0, 12(t0)              # sector hi

    li      t0, DATA_BUF_ADDR
    li      t1, PATTERN
    sw      t1, 0(t0)               # data[0:4] = 0xa5a5a5a5
    sw      t1, 4(t0)
    sw      t1, 8(t0)
    sw      t1, 12(t0)
    # data[16:512] 保持 0 (BSS-clean); pwrite 写整 512 B, 后 496 B 都是 0.

    # ---- 摆 avail: ring[0]=0 (head_desc_idx), idx=1 ----
    li      t0, VRING_BASE
    sh      x0, VRING_AVAIL_OFF(t0)         # avail.flags = 0
    sh      x0, VRING_AVAIL_OFF + 4(t0)     # avail.ring[0] = 0
    li      t1, 1
    sh      t1, VRING_AVAIL_OFF + 2(t0)     # avail.idx = 1 (启动阶段 1)

    # ---- 清 used / status byte ----
    li      t1, VRING_USED_ADDR
    sw      x0, 0(t1)
    sw      x0, 4(t1)
    sw      x0, 8(t1)
    li      t0, STATUS_ADDR
    li      t1, 0xff
    sb      t1, 0(t0)

    # ---- 写 Status: ACK|DRIVER|FEATURES_OK|DRIVER_OK ----
    li      t0, VBLK_BASE
    li      t1, VBLK_STATUS_FULL
    sw      t1, VBLK_REG_STATUS(t0)

    li      s0, 0                   # handler done counter
    li      a5, 0                   # spin counter

    # ---- 触发阶段 1 ----
    sw      x0, VBLK_REG_QUEUE_NOTIFY(t0)

wait_phase1:
    addi    a5, a5, 1
    li      t1, 1
    bne     s0, t1, wait_phase1

    # ---- 阶段 2: 改 desc[1].flags = NEXT|WRITE (按 spec) ----
    li      t0, VRING_BASE
    li      t1, 0x00020003          # flags=NEXT|WRITE(3) | (next=2 << 16)
    sw      t1, 28(t0)

    # ---- 改 header.type = IN ----
    li      t0, HEADER_ADDR
    li      t1, 0                   # type = VIRTIO_BLK_T_IN
    sw      t1, 0(t0)
    # sector / reserved 不变 (仍 = 0)

    # ---- 清 data buf 头 16 byte (确保 worker pread 真把 pattern 写回来) ----
    li      t0, DATA_BUF_ADDR
    sw      x0, 0(t0)
    sw      x0, 4(t0)
    sw      x0, 8(t0)
    sw      x0, 12(t0)

    # ---- 摆 avail.ring[1]=0, avail.idx=2 ----
    li      t0, VRING_BASE
    sh      x0, VRING_AVAIL_OFF + 6(t0)     # avail.ring[1] = 0
    li      t1, 2
    sh      t1, VRING_AVAIL_OFF + 2(t0)     # avail.idx = 2 (启动阶段 2)

    # ---- 清 status byte ----
    li      t0, STATUS_ADDR
    li      t1, 0xff
    sb      t1, 0(t0)

    # ---- 触发阶段 2 ----
    li      t0, VBLK_BASE
    sw      x0, VBLK_REG_QUEUE_NOTIFY(t0)

wait_phase2:
    addi    a5, a5, 1
    li      t1, 2
    bne     s0, t1, wait_phase2

    # ---- 验 used.idx == 2 ----
    li      t0, VRING_USED_ADDR
    lhu     a6, 2(t0)

    # ---- 验 data buf 头 4 byte == 0xa5a5a5a5 (sector 0 写后读回) ----
    li      t0, DATA_BUF_ADDR
    lw      a7, 0(t0)

    # ---- 验 status byte == 0 ----
    li      t0, STATUS_ADDR
    lbu     t6, 0(t0)

    # ---- halt: test_dev sifive FINISHER PASS → 干净停机, exit_code=0 ----
    # 注: 替代旧 triple-fault 停机 (Smdbltrp MDT abort 后不再可用)
    li      t0, 0x00100000          # TEST_DEV_BASE
    li      t1, 0x5555              # FINISHER_PASS
    sw      t1, 0(t0)               # sifive FINISHER off 0
1:  j       1b                      # 防 fall-through: SRS=0 下个 block 边界退

m_handler:
    csrr    s2, mcause              # 期望 0x8000000B

    li      t0, PLIC_CLAIM
    lw      s3, 0(t0)               # 期望 1

    # 读 InterruptStatus
    li      t1, VBLK_BASE
    lw      s4, VBLK_REG_INT_STATUS(t1)

    # 写 InterruptACK = 1
    li      t2, 1
    sw      t2, VBLK_REG_INT_ACK(t1)

    # PLIC complete = 1
    sw      s3, 0(t0)

    addi    s0, s0, 1
    mret

# ===== 期望结果 =====
# 寄存器:
#   a5 > 0                   (spin 总次数)
#   a6 = 2                   (used.idx 推进两次)
#   a7 = 0xa5a5a5a5          (sector 0 头 4 byte; 阶段 1 写, 阶段 2 读回)
#   t6 = 0                   (status byte = OK, 阶段 2 的 worker 写入)
#   s0 = 2                   (handler 跑两次)
#   s2 = 0x8000000B          (最后一次 mcause)
#   s3 = 1                   (最后一次 claim)
#   s4 = 1                   (最后一次 InterruptStatus bit0)
#
# 停机 (test_dev sifive FINISHER PASS): shell 退出码 $? = 0; SRS=0 退主循环.
#
# 不期望:
#   - 任何 [bus] CAUSE_*_ACCESS_FAULT
#   - 程序崩溃
#   - wait_phase1 / wait_phase2 永不退
