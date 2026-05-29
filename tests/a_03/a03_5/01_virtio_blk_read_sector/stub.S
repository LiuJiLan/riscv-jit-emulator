# a03_5 / 01_virtio_blk_read_sector
#
# ===== 运行参数 =====
# RUN-BLK: disk.img
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# virtio-mmio block device (legacy v1.0) 读 sector 0 端到端 — 验:
#   1. magic/version/device_id/vendor_id 4 个 R-only 寄存器值
#   2. legacy 状态机 (Status: ACK | DRIVER | FEATURES_OK | DRIVER_OK)
#   3. 单 queue 配置 (QueueSel=0 / QueueNum=8 / QueuePFN=0x80100 / QueueAlign=4096)
#   4. 摆 desc 链 (3 段: header / data / status) + avail.idx=1 → sw QueueNotify
#   5. worker drain → pread sector 0 → 写 used.ring + InterruptStatus.bit0 →
#      device_set_pending(VIRTIO_BLK_PLIC_IRQ=1) → PLIC 仲裁 → MEIP fire
#   6. m_handler: 验 mcause = 0x8000000B / claim=1 → read InterruptStatus →
#      write InterruptACK=1 → complete=1 → mret
#   7. 主帧验 used.idx==1 + data buf 头 4 byte == 0xdeadbeef + status==0
#
# host image: 4 KB disk.img, sector 0 前 4 byte = 'ef be ad de' (LE 顺) →
# RV32 lw 读 → 0xdeadbeef.
#
# ===== 思路 =====
# vring 摆在 0x80100000 (RAM 内远离 .text); data buf 在 0x80200000;
# status byte 在 0x80200200. RV32 addr 字段 8 byte 写入用两次 sw (低 32 + 高 32 = 0).
#
# desc layout 16 B/entry: addr u64 + len u32 + flags u16 + next u16
#   sw 写 12(desc) 时: lo16 = flags @ byte 12/13, hi16 = next @ byte 14/15.
#   合并 = flags | (next << 16); RV32 LE 跟 desc struct 字段顺一致.
#
# 异步: worker thread drain 是异步的, MEIP fire 时机不确定; 用 wait_loop spin
# 直到 handler 完 (s0=1).

.equ PLIC_PRIO_BASE,    0x0C000000
.equ PLIC_ENABLE_BASE,  0x0C002000
.equ PLIC_THRESHOLD,    0x0C200000
.equ PLIC_CLAIM,        0x0C200004
.equ VBLK_BASE,         0x10001000

# virtio-mmio 寄存器 off (跟 virtio_blk.c 一致)
.equ VBLK_REG_MAGIC,        0x000
.equ VBLK_REG_VERSION,      0x004
.equ VBLK_REG_DEVICE_ID,    0x008
.equ VBLK_REG_VENDOR_ID,    0x00C
.equ VBLK_REG_QUEUE_SEL,    0x030
.equ VBLK_REG_QUEUE_NUM,    0x038
.equ VBLK_REG_QUEUE_ALIGN,  0x03C
.equ VBLK_REG_QUEUE_PFN,    0x040
.equ VBLK_REG_QUEUE_NOTIFY, 0x050
.equ VBLK_REG_INT_STATUS,   0x060
.equ VBLK_REG_INT_ACK,      0x064
.equ VBLK_REG_STATUS,       0x070

# Status bits 合并 (ACK=1 | DRIVER=2 | DRIVER_OK=4 | FEATURES_OK=8)
.equ VBLK_STATUS_FULL,      0x0F

# vring 布局 (queue_num=8, queue_align=4096):
#   desc[8] @ 0x80100000   (16*8 = 128 B)
#   avail   @ 0x80100080   (flags/idx/ring[8]/used_event = 22 B)
#   pad to 4 KB
#   used    @ 0x80101000   (flags/idx/ring[8]elem/avail_event = 70 B)
.equ VRING_BASE,            0x80100000
.equ VRING_AVAIL_OFF,       0x80             # < 2047, 可直接 imm(reg)
.equ VRING_USED_ADDR,       0x80101000       # used 区超 12-bit imm, 单独 li 装基址

.equ HEADER_ADDR,           0x80100200      # 单独区, 跟 desc/avail/used 不冲突
.equ DATA_BUF_ADDR,         0x80200000
.equ STATUS_ADDR,           0x80200200

.section .text
.option norvc
.globl _start

_start:
    # ---- M init: mtvec / mie.MEIP / mstatus.MIE ----
    la      t0, m_handler
    csrw    mtvec, t0
    li      t0, 0x800               # mie.MEIP = bit 11
    csrw    mie, t0
    csrw    0x310, x0               # mstatush 清 MDT (A.7 prelude; 否则下条 MIE 写被强制 0)
    csrrsi  x0, mstatus, 0x8        # mstatus.MIE = bit 3

    # ---- 配 PLIC: priority[1]=1, threshold[ctx0]=0, enable[ctx0].bit1=1 ----
    li      t0, PLIC_PRIO_BASE
    li      t1, 1
    sw      t1, 4(t0)               # priority[1] @ 4*1 = 4
    li      t0, PLIC_THRESHOLD
    sw      x0, 0(t0)
    li      t0, PLIC_ENABLE_BASE
    li      t1, 0x02                # enable bit 1 (source 1)
    sw      t1, 0(t0)

    # ---- 校验 virtio MMIO header 4 R-only 寄存器 ----
    li      t0, VBLK_BASE
    lw      a0, VBLK_REG_MAGIC(t0)      # 期望 0x74726976 ("virt")
    lw      a1, VBLK_REG_VERSION(t0)    # 期望 1
    lw      a2, VBLK_REG_DEVICE_ID(t0)  # 期望 2 (block)
    lw      a3, VBLK_REG_VENDOR_ID(t0)  # 期望 0x554d4551 ("QEMU")

    # ---- 配 queue: Sel=0 / Num=8 / Align=4096 / PFN=0x80100 ----
    sw      x0, VBLK_REG_QUEUE_SEL(t0)
    li      t1, 8
    sw      t1, VBLK_REG_QUEUE_NUM(t0)
    li      t1, 4096
    sw      t1, VBLK_REG_QUEUE_ALIGN(t0)
    li      t1, 0x80100             # PFN = base / 4096
    sw      t1, VBLK_REG_QUEUE_PFN(t0)

    # ---- 摆 desc[0]: header @ HEADER_ADDR, len=16, flags=NEXT, next=1 ----
    li      t0, VRING_BASE
    li      t1, HEADER_ADDR
    sw      t1, 0(t0)               # desc[0].addr_lo
    sw      x0, 4(t0)               # desc[0].addr_hi
    li      t1, 16
    sw      t1, 8(t0)               # desc[0].len = 16
    li      t1, 0x00010001          # flags=NEXT(1) | (next=1 << 16)
    sw      t1, 12(t0)

    # ---- 摆 desc[1]: data buf @ DATA_BUF_ADDR, len=512, flags=NEXT|WRITE, next=2 ----
    li      t1, DATA_BUF_ADDR
    sw      t1, 16(t0)              # desc[1].addr_lo
    sw      x0, 20(t0)              # desc[1].addr_hi
    li      t1, 512
    sw      t1, 24(t0)              # desc[1].len = 512
    li      t1, 0x00020003          # flags=NEXT|WRITE(3) | (next=2 << 16)
    sw      t1, 28(t0)

    # ---- 摆 desc[2]: status @ STATUS_ADDR, len=1, flags=WRITE (no NEXT) ----
    li      t1, STATUS_ADDR
    sw      t1, 32(t0)              # desc[2].addr_lo
    sw      x0, 36(t0)
    li      t1, 1
    sw      t1, 40(t0)              # desc[2].len = 1
    li      t1, 0x00000002          # flags=WRITE(2), next=0 (last)
    sw      t1, 44(t0)

    # ---- 摆 header (@ HEADER_ADDR): type=0 (IN) / reserved=0 / sector=0 ----
    li      t0, HEADER_ADDR
    sw      x0, 0(t0)               # type = 0 (VIRTIO_BLK_T_IN)
    sw      x0, 4(t0)               # reserved
    sw      x0, 8(t0)               # sector lo = 0
    sw      x0, 12(t0)              # sector hi

    # ---- 摆 avail: flags=0 / idx=1 / ring[0]=0 (head_desc_idx=0) ----
    li      t0, VRING_BASE
    sh      x0, VRING_AVAIL_OFF(t0)         # avail.flags = 0
    li      t1, 1
    sh      t1, VRING_AVAIL_OFF + 2(t0)     # avail.idx = 1
    sh      x0, VRING_AVAIL_OFF + 4(t0)     # avail.ring[0] = 0

    # ---- 清 used (worker drain 完会写; 先清确保比对干净) ----
    li      t1, VRING_USED_ADDR
    sw      x0, 0(t1)                       # used.flags=0, used.idx=0 (合 4B sw)
    sw      x0, 4(t1)                       # used.ring[0].id
    sw      x0, 8(t1)                       # used.ring[0].len

    # ---- 清 data buf 头 4 byte (确保 worker pread 真写进来, 不是初始就在) ----
    li      t0, DATA_BUF_ADDR
    sw      x0, 0(t0)

    # ---- 清 status byte 为 0xff (确保 worker 真改成 0) ----
    li      t0, STATUS_ADDR
    li      t1, 0xff
    sb      t1, 0(t0)

    # ---- 写 Status: ACK|DRIVER|FEATURES_OK|DRIVER_OK ----
    li      t0, VBLK_BASE
    li      t1, VBLK_STATUS_FULL
    sw      t1, VBLK_REG_STATUS(t0)

    # ---- s0/a5 init ----
    li      s0, 0                   # handler done flag
    li      a5, 0                   # spin counter

    # ---- 触发: sw QueueNotify = 0 ----
    sw      x0, VBLK_REG_QUEUE_NOTIFY(t0)

wait_loop:
    addi    a5, a5, 1
    beqz    s0, wait_loop

    # ---- 验 used.idx == 1 ----
    li      t0, VRING_USED_ADDR
    lhu     a6, 2(t0)                       # used.idx

    # ---- 验 data buf 头 4 byte == 0xdeadbeef ----
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
    csrr    s2, mcause              # 期望 0x8000000B (INT_BIT | IRQ_M_EXT)

    li      t0, PLIC_CLAIM
    lw      s3, 0(t0)               # 期望 1 (virtio-blk source_id)

    # 读 InterruptStatus, 验 bit0=1
    li      t1, VBLK_BASE
    lw      s4, VBLK_REG_INT_STATUS(t1)

    # 写 InterruptACK = 1 (清 bit0 → device_clear_pending)
    li      t2, 1
    sw      t2, VBLK_REG_INT_ACK(t1)

    # PLIC complete = 1
    sw      s3, 0(t0)

    li      s0, 1
    mret

# ===== 期望结果 =====
# 寄存器:
#   a0 = 0x74726976 (MAGIC = "virt")
#   a1 = 1          (VERSION = legacy)
#   a2 = 2          (DEVICE_ID = block)
#   a3 = 0x554d4551 (VENDOR_ID = "QEMU")
#   a5 > 0          (wait_loop spin 次数)
#   a6 = 1          (used.idx; worker drain 1 笔后推进)
#   a7 = 0xdeadbeef (sector 0 头 4 byte; disk.img Makefile 写入)
#   t6 = 0          (status byte = VIRTIO_BLK_S_OK; worker 写入)
#   s0 = 1          (handler 跑过)
#   s2 = 0x8000000B (mcause = INT_BIT | IRQ_M_EXT=11)
#   s3 = 1          (PLIC claim → virtio-blk source_id)
#   s4 = 1          (InterruptStatus bit0 = used buffer notification)
#
# 停机 (test_dev sifive FINISHER PASS):
#   shell 退出码 $? = 0
#   [dispatcher] halted: ... (SRS=0 退主循环)
#
# 不期望:
#   - 任何 [bus] CAUSE_*_ACCESS_FAULT (vring/data 都在 RAM 内, virtio_blk 寄存器合法)
#   - 程序崩溃 (ASan/UBSan 报错)
#   - wait_loop 永不退 (worker drain 失败 / IRQ 没 fire / handler 没跑)
