# a03_5 / 03_virtio_blk_flush
#
# ===== 运行参数 =====
# RUN-BLK: disk.img
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# virtio-blk legacy v1.0 VIRTIO_BLK_F_FLUSH / VIRTIO_BLK_T_FLUSH 闭环 — 验:
#   1. DeviceFeatures(bank0) bit9 (F_FLUSH) 被 offer (设备声称支持 flush)
#   2. legacy 状态机 + ack F_FLUSH (DriverFeatures echo)
#   3. 单 queue 配置 (Sel=0 / Num=8 / PFN=0x80100 / Align=4096)
#   4. round 1: T_OUT 写 sector 0 (3 段链 header/data/status) → QueueNotify →
#      poll used.idx==1 → 验 OUT status == S_OK(0)
#   5. round 2: T_FLUSH (2 段链 header/status, 无 data) → QueueNotify →
#      poll used.idx==2 → 验 FLUSH status == S_OK(0) (而非 S_UNSUPP=2)
#   全过 → test_dev sifive FINISHER PASS (exit 0)
#
# ===== 思路 =====
# 沿用 01_virtio_blk_read_sector 的内存布局 + desc 编码 (flags|next<<16 单 sw)。
# 不用 IRQ handler, 直接 poll guest RAM 里的 used.idx (worker thread 异步写, 单 hart
# 主帧 spin 读得到)。FLUSH 的真 durability 进程内不可见 (fsync 落盘对回读无影响),
# 断言落在 "feature 暴露 + FLUSH 返 OK 而非 UNSUPP"; OUT 在前保证 FLUSH 有前序写
# 可刷 (单 io_worker + FIFO queue → fsync 必在 pwrite 之后)。
#
# 注: disk.img 是空白可写盘 (Makefile zero-fill); OUT data 内容不校验, 只看 status。

.equ VBLK_BASE,         0x10001000

.equ VBLK_REG_DEVICE_FEAT,     0x010
.equ VBLK_REG_DEVICE_FEAT_SEL, 0x014
.equ VBLK_REG_DRIVER_FEAT,     0x020
.equ VBLK_REG_DRIVER_FEAT_SEL, 0x024
.equ VBLK_REG_QUEUE_SEL,       0x030
.equ VBLK_REG_QUEUE_NUM,       0x038
.equ VBLK_REG_QUEUE_ALIGN,     0x03C
.equ VBLK_REG_QUEUE_PFN,       0x040
.equ VBLK_REG_QUEUE_NOTIFY,    0x050
.equ VBLK_REG_STATUS,          0x070

# Status bits 合并 (ACK=1 | DRIVER=2 | FEATURES_OK=8 | DRIVER_OK=4)
.equ VBLK_STATUS_FULL,  0x0F

# vring 布局 (queue_num=8, queue_align=4096) — 同 01
.equ VRING_BASE,        0x80100000
.equ VRING_AVAIL_OFF,   0x80
.equ VRING_USED_ADDR,   0x80101000
.equ HEADER_ADDR,       0x80100200
.equ DATA_BUF_ADDR,     0x80200000
.equ STATUS_ADDR,       0x80200200

# virtio-blk request type
.equ T_OUT,             1
.equ T_FLUSH,           4

.section .text
.option norvc
.globl _start

_start:
    li      t0, VBLK_BASE

    # ---- FEATURES: DeviceFeatures(bank0) 必须 offer F_FLUSH (bit9) ----
    sw      x0, VBLK_REG_DEVICE_FEAT_SEL(t0)    # sel = 0 (bank0)
    lw      a0, VBLK_REG_DEVICE_FEAT(t0)        # a0 = DeviceFeatures low32
    srli    t1, a0, 9
    andi    t1, t1, 1
    beqz    t1, fail                            # bit9 未置 → 设备没 offer FLUSH

    # ---- handshake: ACK | DRIVER ----
    li      t1, 1
    sw      t1, VBLK_REG_STATUS(t0)
    li      t1, 3
    sw      t1, VBLK_REG_STATUS(t0)

    # ---- ack F_FLUSH: DriverFeaturesSel=0, DriverFeatures = echo (含 bit9) ----
    sw      x0, VBLK_REG_DRIVER_FEAT_SEL(t0)
    sw      a0, VBLK_REG_DRIVER_FEAT(t0)

    # ---- FEATURES_OK | DRIVER_OK ----
    li      t1, VBLK_STATUS_FULL
    sw      t1, VBLK_REG_STATUS(t0)

    # ---- 配 queue: Sel=0 / Num=8 / Align=4096 / PFN=0x80100 ----
    sw      x0, VBLK_REG_QUEUE_SEL(t0)
    li      t1, 8
    sw      t1, VBLK_REG_QUEUE_NUM(t0)
    li      t1, 4096
    sw      t1, VBLK_REG_QUEUE_ALIGN(t0)
    li      t1, 0x80100
    sw      t1, VBLK_REG_QUEUE_PFN(t0)

    # =========================================================================
    # round 1: T_OUT 写 sector 0 (3 段链)
    # =========================================================================
    # desc[0] header @ HEADER_ADDR len16 flags=NEXT next=1
    li      t2, VRING_BASE
    li      t1, HEADER_ADDR
    sw      t1, 0(t2)
    sw      x0, 4(t2)
    li      t1, 16
    sw      t1, 8(t2)
    li      t1, 0x00010001          # flags=NEXT(1) | next=1<<16
    sw      t1, 12(t2)
    # desc[1] data @ DATA_BUF len512 flags=NEXT next=2 (OUT: device 读, W=0)
    li      t1, DATA_BUF_ADDR
    sw      t1, 16(t2)
    sw      x0, 20(t2)
    li      t1, 512
    sw      t1, 24(t2)
    li      t1, 0x00020001          # flags=NEXT(1) | next=2<<16
    sw      t1, 28(t2)
    # desc[2] status @ STATUS_ADDR len1 flags=WRITE next=0
    li      t1, STATUS_ADDR
    sw      t1, 32(t2)
    sw      x0, 36(t2)
    li      t1, 1
    sw      t1, 40(t2)
    li      t1, 0x00000002          # flags=WRITE(2), next=0
    sw      t1, 44(t2)

    # header: type=OUT sector=0
    li      t3, HEADER_ADDR
    li      t1, T_OUT
    sw      t1, 0(t3)
    sw      x0, 4(t3)
    sw      x0, 8(t3)
    sw      x0, 12(t3)

    # data buf 头写一个识别字 (内容不校验, 只为 pwrite 有数据)
    li      t3, DATA_BUF_ADDR
    li      t1, 0x12345678
    sw      t1, 0(t3)

    # status sentinel = 0xFF
    li      t3, STATUS_ADDR
    li      t1, 0xFF
    sb      t1, 0(t3)

    # 清 used (flags+idx 合 4B)
    li      t3, VRING_USED_ADDR
    sw      x0, 0(t3)

    # avail: flags=0 idx=1 ring[0]=0
    sh      x0, VRING_AVAIL_OFF(t2)
    li      t1, 1
    sh      t1, VRING_AVAIL_OFF + 2(t2)
    sh      x0, VRING_AVAIL_OFF + 4(t2)

    sw      x0, VBLK_REG_QUEUE_NOTIFY(t0)

    # poll used.idx == 1
    li      t3, VRING_USED_ADDR
poll_out:
    lhu     t1, 2(t3)
    li      t4, 1
    bne     t1, t4, poll_out

    # 验 OUT status == S_OK(0)
    li      t3, STATUS_ADDR
    lbu     t1, 0(t3)
    bnez    t1, fail

    # =========================================================================
    # round 2: T_FLUSH (2 段链 header/status, 无 data)
    # =========================================================================
    li      t2, VRING_BASE
    # desc[0] header @ HEADER_ADDR len16 flags=NEXT next=1
    li      t1, HEADER_ADDR
    sw      t1, 0(t2)
    sw      x0, 4(t2)
    li      t1, 16
    sw      t1, 8(t2)
    li      t1, 0x00010001
    sw      t1, 12(t2)
    # desc[1] status @ STATUS_ADDR len1 flags=WRITE next=0 (链到此结束)
    li      t1, STATUS_ADDR
    sw      t1, 16(t2)
    sw      x0, 20(t2)
    li      t1, 1
    sw      t1, 24(t2)
    li      t1, 0x00000002
    sw      t1, 28(t2)

    # header: type=FLUSH sector=0 (FLUSH 忽略 sector)
    li      t3, HEADER_ADDR
    li      t1, T_FLUSH
    sw      t1, 0(t3)
    sw      x0, 8(t3)

    # status sentinel = 0xFF
    li      t3, STATUS_ADDR
    li      t1, 0xFF
    sb      t1, 0(t3)

    # avail: ring[1]=0 idx=2
    sh      x0, VRING_AVAIL_OFF + 6(t2)     # ring[1] = head desc idx 0
    li      t1, 2
    sh      t1, VRING_AVAIL_OFF + 2(t2)     # avail.idx = 2

    sw      x0, VBLK_REG_QUEUE_NOTIFY(t0)

    # poll used.idx == 2
    li      t3, VRING_USED_ADDR
poll_flush:
    lhu     t1, 2(t3)
    li      t4, 2
    bne     t1, t4, poll_flush

    # 验 FLUSH status == S_OK(0), 不是 S_UNSUPP(2)
    li      t3, STATUS_ADDR
    lbu     t1, 0(t3)
    bnez    t1, fail

    # ---- PASS ----
    li      t0, 0x00100000          # TEST_DEV_BASE
    li      t1, 0x5555              # FINISHER_PASS
    sw      t1, 0(t0)
1:  j       1b                      # 防 fall-through

fail:
    li      t0, 0x00100000
    li      t1, 0x3333              # FINISHER_FAIL
    sw      t1, 0(t0)
2:  j       2b

# ===== 期望结果 =====
#   a0      = DeviceFeatures low32 (bit9 置位; F_FLUSH offered)
#   round 1 OUT status byte (STATUS_ADDR) = 0  (S_OK)
#   round 2 FLUSH status byte             = 0  (S_OK, 非 2/UNSUPP)
#   used.idx 推进 0→1→2
#   停机: test_dev FINISHER PASS → shell 退出码 0
# 不期望:
#   - DeviceFeatures bit9 = 0 (会走 fail)
#   - FLUSH status = 2 (UNSUPP; 说明 T_FLUSH 没被识别 → fail)
#   - poll 永不退 (worker drain 失败 / fsync 卡死)
#   - 任何 [bus] ACCESS_FAULT / ASan 报错
