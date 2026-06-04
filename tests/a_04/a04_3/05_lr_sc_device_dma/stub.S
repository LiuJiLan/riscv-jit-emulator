# a04_3 / 05_lr_sc_device_dma
#
# ===== 运行参数 =====
# RUN-BLK: disk.img
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# T4 端到端 — virtio_blk DMA pread 写 RAM 后 lrsc_on_device_write blanket clear
# 所有 hart reservation, guest 后续 SC.W 必失败. 验 RV spec §14.2.1 line 108-109
# "An SC may succeed only if no write from a device ... can be observed to have
# occurred between the LR and the SC" 强制清除路径接通.
#
# 流程:
#   1. hart 配 virtio_blk + PLIC, 摆 desc 链读 sector 0 → DATA_BUF (0x80200000)
#   2. hart LR.W @ DATA_BUF (此时 buf=0, LR 读 0, 建 reservation @ DATA_BUF)
#   3. hart sw QueueNotify → 触发 io_worker drain
#   4. io_worker: pread sector 0 → 写 DATA_BUF (改 0 → 0xDEADBEEF) → 写 used →
#      drain 完 → lrsc_on_device_write blanket clear → device_set_pending(IRQ=1)
#   5. hart polling PLIC.claim (非 0 = device_set_pending 已调, 等价 lrsc_on_device_write
#      已完成; atomic ordering happens-before 传递)
#   6. hart SC.W @ DATA_BUF, value=0x99999999 → 应失败 rd=1
#   7. 验 mem = 0xDEADBEEF (disk.img sector 0 byte 0-3 LE 'ef be ad de'), 不是
#      0x99999999 (SC 失败不改 mem)
#
# ===== 关键设计选择 =====
# 不开 mie/mstatus.MIE (跟 a_03/a03_5/01 同 fixture 体例 IRQ 路径区别):
# 本 fixture 走 polling 等 io_worker 完成, 不进 trap. 理由: trap entry
# trap_set_*_state 入口调 lrsc_clear_self (Q5 B1 七类清除时机 #1), hart 自己进
# trap 也会清自己 reservation — fixture 不能区分"device clear" vs "trap entry
# self-clear" 两个路径. polling PLIC.claim 让 hart 不进 trap, SC 失败的原因
# 唯一来源 = lrsc_on_device_write.
#
# polling PLIC.claim (而非 used.idx 或 InterruptStatus):
# io_worker 顺序: drain (写 used.idx + 写 interrupt_status bit0) → state_unlock
# → lrsc_on_device_write → device_set_pending (atomic).
# 如果 hart polling used.idx 或 InterruptStatus, 看到值变化时
# lrsc_on_device_write 可能还在路上 (drain 完 unlock 后) — race window 存在.
# PLIC.claim 在 device_set_pending 之后才能 read 到 non-zero (atomic ordering 保
# happens-before), 而 device_set_pending 严格在 lrsc_on_device_write 之后调 →
# hart 看到 claim non-zero 时, lrsc_on_device_write 必已完成. race-free.

.equ PLIC_PRIO_BASE,    0x0C000000
.equ PLIC_ENABLE_BASE,  0x0C002000
.equ PLIC_THRESHOLD,    0x0C200000
.equ PLIC_CLAIM,        0x0C200004
.equ VBLK_BASE,         0x10001000

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

.equ VBLK_STATUS_FULL,      0x0F

.equ VRING_BASE,            0x80100000
.equ VRING_AVAIL_OFF,       0x80
.equ HEADER_ADDR,           0x80100200
.equ DATA_BUF_ADDR,         0x80200000
.equ STATUS_ADDR,           0x80200200

.equ TEST_DEV_BASE,         0x00100000
.equ FINISHER_PASS,         0x5555
.equ FINISHER_FAIL,         0x3333

.equ DISK_SECTOR0_DATA,     0xDEADBEEF      # disk.img sector 0 byte 0-3 = ef be ad de (LE)
.equ SC_VALUE,              0x99999999      # SC 想写的值 (跟 DISK 值不同, 区分 SC 改/不改)

.section .text
.option norvc
.globl _start

_start:
    # ---- Boot prelude (清 MDT/SDT) ----
    csrw    0x310, x0
    csrw    sstatus, x0

    # 不开 mie / mstatus.MIE → IRQ 永不进 trap (hart 不被 self-clear reservation)

    # ---- 配 PLIC: priority[1]=1, threshold[ctx0]=0, enable[ctx0].bit1=1 ----
    # 即使 IRQ 不 trap, PLIC.claim 仍会被 device_set_pending → fire 写到 claim.
    # hart polling claim 来知道 device 完成.
    li      t0, PLIC_PRIO_BASE
    li      t1, 1
    sw      t1, 4(t0)               # priority[1] = 1
    li      t0, PLIC_THRESHOLD
    sw      x0, 0(t0)
    li      t0, PLIC_ENABLE_BASE
    li      t1, 0x02                # enable source 1 (virtio-blk)
    sw      t1, 0(t0)

    # ---- 校验 virtio_blk header 4 R-only 寄存器 ----
    li      t0, VBLK_BASE
    lw      a0, VBLK_REG_MAGIC(t0)      # 期望 0x74726976 ("virt")
    lw      a1, VBLK_REG_VERSION(t0)    # 期望 1
    lw      a2, VBLK_REG_DEVICE_ID(t0)  # 期望 2
    lw      a3, VBLK_REG_VENDOR_ID(t0)  # 期望 0x554d4551 ("QEMU")

    # ---- 配 queue ----
    sw      x0, VBLK_REG_QUEUE_SEL(t0)
    li      t1, 8
    sw      t1, VBLK_REG_QUEUE_NUM(t0)
    li      t1, 4096
    sw      t1, VBLK_REG_QUEUE_ALIGN(t0)
    li      t1, 0x80100
    sw      t1, VBLK_REG_QUEUE_PFN(t0)

    # ---- 摆 desc[0]: header @ HEADER_ADDR ----
    li      t0, VRING_BASE
    li      t1, HEADER_ADDR
    sw      t1, 0(t0)
    sw      x0, 4(t0)
    li      t1, 16
    sw      t1, 8(t0)
    li      t1, 0x00010001          # flags=NEXT | next=1
    sw      t1, 12(t0)

    # ---- 摆 desc[1]: data buf @ DATA_BUF_ADDR ----
    li      t1, DATA_BUF_ADDR
    sw      t1, 16(t0)
    sw      x0, 20(t0)
    li      t1, 512
    sw      t1, 24(t0)
    li      t1, 0x00020003          # flags=NEXT|WRITE | next=2
    sw      t1, 28(t0)

    # ---- 摆 desc[2]: status @ STATUS_ADDR ----
    li      t1, STATUS_ADDR
    sw      t1, 32(t0)
    sw      x0, 36(t0)
    li      t1, 1
    sw      t1, 40(t0)
    li      t1, 0x00000002          # flags=WRITE, next=0
    sw      t1, 44(t0)

    # ---- 摆 header: type=0 IN, sector=0 ----
    li      t0, HEADER_ADDR
    sw      x0, 0(t0)
    sw      x0, 4(t0)
    sw      x0, 8(t0)
    sw      x0, 12(t0)

    # ---- 摆 avail: idx=1, ring[0]=0 ----
    li      t0, VRING_BASE
    sh      x0, VRING_AVAIL_OFF(t0)
    li      t1, 1
    sh      t1, VRING_AVAIL_OFF + 2(t0)
    sh      x0, VRING_AVAIL_OFF + 4(t0)

    # ---- 清 DATA_BUF (确保 pread 真改 0 → 0xDEADBEEF, 不是初始就在) ----
    li      t0, DATA_BUF_ADDR
    sw      x0, 0(t0)

    # ---- 清 status byte ----
    li      t0, STATUS_ADDR
    li      t1, 0xff
    sb      t1, 0(t0)

    # ---- 写 Status: full (ACK | DRIVER | FEATURES_OK | DRIVER_OK) ----
    li      t0, VBLK_BASE
    li      t1, VBLK_STATUS_FULL
    sw      t1, VBLK_REG_STATUS(t0)

    # ============================================================
    # 关键步骤 1: LR.W @ DATA_BUF (此时 buf=0, 建 reservation)
    # ============================================================
    # LR 在 sw QueueNotify *之前* 做 — 保证 reservation 建立 *早于* io_worker
    # pread 写 RAM. 如果 LR 在 io_worker 完成之后才做, lrsc_on_device_write 也
    # 已经在 device_set_pending 之前完成 — LR 后无 device write, SC 应成功. 这
    # 不是本 fixture 测的场景.
    li      s1, DATA_BUF_ADDR
    lr.w    s2, (s1)                # rd=0 (DATA_BUF=0), 建 reservation @ DATA_BUF

    # ============================================================
    # 关键步骤 2: sw QueueNotify (触发 io_worker drain)
    # ============================================================
    li      t0, VBLK_BASE
    sw      x0, VBLK_REG_QUEUE_NOTIFY(t0)

    # ============================================================
    # 关键步骤 3: polling PLIC.claim 等 io_worker 完成 (race-free)
    # ============================================================
    # io_worker 顺序: drain → state_unlock → lrsc_on_device_write → device_set_pending.
    # device_set_pending 走 atomic_store; hart 这里 lw PLIC.claim 走 atomic_load
    # acquire (PLIC hot path 体例) — atomic ordering 保证 hart 看到 claim non-zero
    # 时, lrsc_on_device_write 之前所有动作都已完成 (happens-before 传递).
    # 所以 SC.W 时 reservation 必已被 device clear.
    li      t3, PLIC_CLAIM
poll_claim:
    lw      s3, 0(t3)
    beqz    s3, poll_claim

    # PLIC complete (cleanup PLIC 内部 claimed flag, fixture 干净退出)
    sw      s3, 0(t3)

    # ============================================================
    # 关键步骤 4: SC.W @ DATA_BUF — 必失败 (device clear 已清 reservation)
    # ============================================================
    li      t2, SC_VALUE
    sc.w    s4, t2, (s1)            # 应失败 rd=1
    li      t4, 1
    bne     s4, t4, fail            # rd != 1 → SC 虚假成功 → fail
                                    # (虚假成功 = lrsc_on_device_write 漏调或 race)

    # ============================================================
    # 关键步骤 5: 验 mem 是 disk 数据 (0xDEADBEEF), 不是 SC value (0x99999999)
    # ============================================================
    lw      s5, 0(s1)
    li      t4, DISK_SECTOR0_DATA
    bne     s5, t4, fail            # mem 应是 disk sector 0 内容

    li      t4, SC_VALUE
    beq     s5, t4, fail            # mem 是 SC value = SC 错误成功改了 mem

    # ---- 验 InterruptStatus + ACK (cleanup virtio_blk 状态) ----
    li      t0, VBLK_BASE
    lw      s6, VBLK_REG_INT_STATUS(t0)
    li      t4, 1                   # 期望 bit0 (VBLK_INT_VRING)
    bne     s6, t4, fail
    sw      t4, VBLK_REG_INT_ACK(t0)

    # ---- PASS ----
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_PASS
    sw      t1, 0(t0)
1:  j       1b

fail:
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_FAIL
    sw      t1, 0(t0)
2:  j       2b

# ===== 期望结果 =====
#   s2 = 0          (LR.W 读 DATA_BUF=0 初始值)
#   s3 = 1          (PLIC.claim = virtio-blk source_id)
#   s4 = 1          (SC.W 失败 — lrsc_on_device_write 已清 reservation)
#   s5 = 0xDEADBEEF (DATA_BUF = disk.img sector 0; SC 失败不改 mem)
#   s6 = 1          (InterruptStatus VBLK_INT_VRING)
#   exit_code = 0   (FINISHER_PASS)
#   wall time < 200ms (polling PLIC.claim 等 worker drain + IRQ, 几百次 spin)
# 不期望:
#   - SC.W 成功 (s4=0) → lrsc_on_device_write 漏调 / hook 失效 / race
#   - s5 = 0x99999999 → SC 错改 mem (SC 失败不应改 mem)
#   - SC.W trap (cause 5/6/7/15) → BARE 路径 + DATA_BUF 在 RAM 内, 不应 trap
#   - polling 永不退 (io_worker drain 失败 / PLIC fanout 失效)
