//
// Created by liujilan on 2026/5/24.
// virtio-mmio block device (legacy v1.0, DeviceID=2) — host file 后端;
// 接 PLIC source 1.
//
// 跟 QEMU virt machine virtio-mmio.0 对齐 (base 0x10001000 紧邻 UART, IRQ 1);
// legacy v1.0 寄存器布局 (Version=1, 单 QueuePFN 单 queue_align). modern v1.1
// 三 PFN 形态属未来工作.
//
// 访问宽度 4 byte only (含 Config space @ +0x100); size != 4 → CAUSE_*_ACCESS_FAULT
// (跟 PLIC/test_dev 同形态).
//
// ----------------------------------------------------------------------------
// monitor 模型 (dummy.txt §7) — virtio-blk 是 "monitor + io_worker 辅助线程"
// ----------------------------------------------------------------------------
//
// monitor 范式四态:
//   CLINT      = "monitor + timer 辅助线程"     (mtime 由 host wall clock 推进)
//   UART       = "monitor + reader 辅助线程"    (RX 源 = host stdin)
//   virtio-blk = "monitor + io_worker 辅助线程" (异步执行 pread/pwrite + 触发 IRQ)
//   PLIC       = "monitor 但无辅助线程"         (atomic 字段直接做 hot path 优化)
//   test_dev   = 不是 monitor                   (无内部状态, 纯 fanout)
//
// io_worker 辅助线程职责 (io_worker_run, 实装在 virtio_blk.c):
//   - hart 写 QueueNotify 时 enqueue token (work queue cap=8; 通知"该 queue 有
//     新 avail entry"); hart 在 queue 满时 cond_timedwait(not_full, 100ms) +
//     SDS 检 (cooperative shutdown 体例, dummy.txt §12)
//   - worker 循环 cond_timedwait(not_empty, 100ms) + SDS 检 → 拉 1 token →
//     drain avail ring (按 QueueNum 上限循环) → 解析 desc 链 → pread/pwrite
//     → 写 used ring → 设 InterruptStatus.bit0 → device_set_pending(
//     VIRTIO_BLK_PLIC_IRQ)
//   - shutdown_signal=0 自然退出 (跟 uart_reader_run 同体例)
//
// 异步默认体例: hart fast path 不阻塞优先, pread/pwrite blocking syscall 必走
// 异步 worker. 详 dummy.txt §7 (monitor 模型) + trade_off_log §T.7 (异步默认体例).
//
// ----------------------------------------------------------------------------
// 读写抽象 (monitor + 双 pthread_mutex_t)
// ----------------------------------------------------------------------------
//
// virtio-blk 用双 mutex (跟 UART 单锁 / PLIC rwlock 区分):
//   state_mutex — 寄存器 + InterruptStatus + ring 解析 + pread/pwrite (worker 长持锁)
//   queue_mutex — work queue ring (hart 入队短持锁) + cond_not_empty / cond_not_full
//
// 锁顺序硬约束: queue_mutex 永不嵌套 state_mutex (避免 hart drain ring 长持锁
// 挡 worker enqueue; 单向无 deadlock 风险).
//   - hart 写 QueueNotify 只持 queue_mutex, 不碰 state_mutex
//   - 写其他寄存器 (Status/QueueSel/etc) 只持 state_mutex, 不碰 queue_mutex
//   - worker dequeue 时只持 queue_mutex, unlock 后再 lock state_mutex drain ring
//   - 单向, 不存在交叉路径 → 无 deadlock 风险
//
// 不 rwlock — 没"高频纯读"路径 (跟 PLIC csr_mip_read 那种区分); 跟 UART 区分点 =
// 状态读写跟 IO drain 时间尺度差 3 个数量级 (drain ring + pread/pwrite 长持锁),
// 单锁会让 hart 入队撞 worker IO 阻塞 → 拆双 mutex.
//
// ----------------------------------------------------------------------------
// 字段模型 vs virtio v1.0 legacy spec — 简化点 + 真实装点
// ----------------------------------------------------------------------------
//
// 寄存器子集 (off / 用途):
//   0x000 MagicValue        R = 0x74726976 ("virt")
//   0x004 Version           R = 1 (legacy)
//   0x008 DeviceID          R = 2 (block)
//   0x00C VendorID          R = 0x554d4551 (QEMU 一致)
//   0x010 DeviceFeatures    R = 0 (无可选 feature; driver 不需要 negotiate)
//   0x014 DeviceFeaturesSel W (silent accept; DeviceFeatures 永远返 0)
//   0x020 DriverFeatures    W (silent accept; 不强校验 driver 写入)
//   0x024 DriverFeaturesSel W
//   0x028 GuestPageSize     W (legacy, silent accept; 假设 4 KB)
//   0x030 QueueSel          W (v1 单 queue, 期望写 0)
//   0x034 QueueNumMax       R = VIRTIO_BLK_QUEUE_NUM_MAX
//   0x038 QueueNum          W
//   0x03C QueueAlign        W (legacy, silent accept; 假设 4 KB)
//   0x040 QueuePFN          RW (4 KB 单元; 写 0 复位 last_avail_idx)
//   0x050 QueueNotify       W (入 work queue, worker 异步 drain)
//   0x060 InterruptStatus   R (bit0 = used buffer notification)
//   0x064 InterruptACK      W (清 InterruptStatus 对应 bit; 清完调
//                              device_clear_pending(VIRTIO_BLK_PLIC_IRQ))
//   0x070 Status            RW (legacy 状态机; 写 0 = reset 寄存器;
//                              不强校验状态机, fixture 守规矩就够)
//   0x100 Config:cap_lo     R = (uint32_t) capacity_sectors
//   0x104 Config:cap_hi     R = (uint32_t)(capacity_sectors >> 32)
//   其他 off: 读 silent 返 0 / 写 silent 吞 (跟 PLIC reserved 区体例)
//
// 后端 = host file (pread/pwrite; 不 fsync 不 mmap):
//   - virtio_blk_init(path) 时 open(O_RDWR) + fstat 取 capacity (st_size/512)
//   - 完成 IO 不 fsync (write-back 默认; durable IO 留 TODO)
//   - st_size 必须 512 对齐 (否则 init 返 -1)
//
// ----------------------------------------------------------------------------
// 五函数 lifecycle
// ----------------------------------------------------------------------------
//
// virtio_blk_init(path)
//   - path==NULL: image_fd=-1, **不 bus 注册**, 整个模块退化为不存在 (return 0).
//     fixture 不传 --blk 时正常路径, 不报错.
//   - path!=NULL: open + fstat + capacity + mutex/cond_init + bus 注册.
//     失败 fprintf + 返 -1 (dummy.txt §5).
//
// virtio_blk_reset()
//   - system reset 每 iter (main while 顶段); state_mutex 持有内清所有寄存器 +
//     InterruptStatus=0 + device_clear_pending(VIRTIO_BLK_PLIC_IRQ);
//     image_fd / work queue / io_worker_thread 不动 (跟 UART/PLIC 体例: 跨
//     reset 一直跑).
//   - image_fd==-1 时 no-op (整个模块退化).
//
// virtio_blk_destroy()
//   - POR 收尾; image_fd != -1 时 close + cond_destroy*2 + mutex_destroy*2.
//     io_worker_thread 已由 virtio_blk_join_io_worker_thread 收回.
//   - image_fd==-1 时 no-op.
//
// virtio_blk_start_io_worker_thread()
//   - POR spawn (uart_start_reader_thread 之后); SDS=0 或 image_fd==-1 时 skip;
//     spawn fail fprintf + SRS=0 + SDS=0 (跟 UART/PLIC 同形态).
//
// virtio_blk_join_io_worker_thread()
//   - POR 退出段 (uart_join_reader_thread 之后); SDS=0 之后调; pthread_join +
//     ESRCH 容错 (BSS 0 init, spawn skip / fail 路径下 join 返 ESRCH).
//   - image_fd==-1 时 no-op (没 spawn 也不 join).
//

#ifndef DEVICE_VIRTIO_BLK_H
#define DEVICE_VIRTIO_BLK_H

int  virtio_blk_init(const char *image_path);
int  virtio_blk_reset(void);
void virtio_blk_destroy(void);

void virtio_blk_start_io_worker_thread(void);
void virtio_blk_join_io_worker_thread (void);

#endif //DEVICE_VIRTIO_BLK_H
