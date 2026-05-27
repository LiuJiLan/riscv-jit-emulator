//
// Created by liujilan on 2026/5/24.
// virtio-mmio block device (legacy v1.0) 实现 — host file 后端 (pread/pwrite) +
// 异步 io_worker 辅助线程 + work queue + bus 注册 + InterruptStatus 同步驱动
// device_line.
//
// 接口形态 + monitor 模型 + 字段对应 spec + 五函数 lifecycle 见 virtio_blk.h
// 顶段 doc. 地址布局 / 容量参数 见 config.h VIRTIO_BLK_* 宏. 报错风格见
// dummy.txt §5; "0=成功 / 非0=cause" 接口约定见 dummy.txt §9. thread spawn/join
// 协议 (谁 spawn 谁 join) 见 dummy.txt §12. 异步默认体例 (hart fast path 不阻塞
// → blocking syscall 必走异步 worker) 见 trade_off_log §T.7.
//

#define _POSIX_C_SOURCE 200809L   // pread / pwrite / clock_gettime CLOCK_REALTIME

#include "virtio_blk.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "config.h"          // VIRTIO_BLK_* / VIRTIO_BLK_PLIC_IRQ
#include "platform/bus.h"    // mmio_dev_t / bus_register_mmio
#include "platform/plic.h"   // device_set_pending / device_clear_pending
#include "platform/ram.h"    // IS_GPA_RAM / gpa_to_hva_offset
#include "riscv.h"           // CAUSE_LOAD/STORE_ACCESS_FAULT
#include "runtime.h"         // shutdown_signal / system_reset_signal


// ----------------------------------------------------------------------------
// 寄存器 offset + 常量 (virtio v1.0 legacy spec §4.2.2 + virtio-blk §5.2)
// ----------------------------------------------------------------------------

#define VBLK_REG_MAGIC            0x000u   /* R "virt" */
#define VBLK_REG_VERSION          0x004u   /* R 1 (legacy) */
#define VBLK_REG_DEVICE_ID        0x008u   /* R 2 (block) */
#define VBLK_REG_VENDOR_ID        0x00Cu   /* R QEMU 一致 */
#define VBLK_REG_DEVICE_FEAT      0x010u   /* R 0 (no optional feature) */
#define VBLK_REG_DEVICE_FEAT_SEL  0x014u   /* W silent accept */
#define VBLK_REG_DRIVER_FEAT      0x020u   /* W silent accept */
#define VBLK_REG_DRIVER_FEAT_SEL  0x024u   /* W silent accept */
#define VBLK_REG_GUEST_PAGE_SIZE  0x028u   /* W legacy silent accept */
#define VBLK_REG_QUEUE_SEL        0x030u   /* W v1 单 queue, expect 0 */
#define VBLK_REG_QUEUE_NUM_MAX    0x034u   /* R VIRTIO_BLK_QUEUE_NUM_MAX */
#define VBLK_REG_QUEUE_NUM        0x038u   /* W */
#define VBLK_REG_QUEUE_ALIGN      0x03Cu   /* W legacy silent accept (假设 4 KB) */
#define VBLK_REG_QUEUE_PFN        0x040u   /* RW (4 KB 单元; 写 0 复位 last_avail_idx) */
#define VBLK_REG_QUEUE_NOTIFY     0x050u   /* W ★ 入 work queue */
#define VBLK_REG_INT_STATUS       0x060u   /* R bit0 = used buffer notification */
#define VBLK_REG_INT_ACK          0x064u   /* W ★ 清 InterruptStatus bit */
#define VBLK_REG_STATUS           0x070u   /* RW legacy 状态机 (写 0 = reset) */
#define VBLK_REG_CONFIG_BASE      0x100u   /* Config space 起点 */
#define VBLK_REG_CONFIG_CAP_LO    0x100u   /* R capacity_sectors 低 32 */
#define VBLK_REG_CONFIG_CAP_HI    0x104u   /* R capacity_sectors 高 32 */

#define VBLK_MAGIC                0x74726976u   /* "virt" little-endian */
#define VBLK_VERSION              1u            /* legacy */
#define VBLK_DEVICE_ID            2u            /* block */
#define VBLK_VENDOR_ID            0x554d4551u   /* "QEMU" */

#define VBLK_INT_VRING            0x1u          /* InterruptStatus bit0 */

/* virtio-blk request header type (spec §5.2.6.1) */
#define VIRTIO_BLK_T_IN           0u            /* read from device → guest buf */
#define VIRTIO_BLK_T_OUT          1u            /* write from guest buf → device */

/* virtio-blk status byte (spec §5.2.6.2) */
#define VIRTIO_BLK_S_OK           0u
#define VIRTIO_BLK_S_IOERR        1u
#define VIRTIO_BLK_S_UNSUPP       2u

/* virtq desc flags (spec §2.6.5) */
#define VRING_DESC_F_NEXT         0x1u
#define VRING_DESC_F_WRITE        0x2u
#define VRING_DESC_F_INDIRECT     0x4u   /* 不支持 — DeviceFeatures=0 不宣告 */

/* desc 链 hop 上限 (防恶意 cycle / 长链卡死 worker) */
#define VBLK_DESC_CHAIN_MAX       16u


// ----------------------------------------------------------------------------
// 内部状态 (单例 file-static; worker thread + hart 主帧跨线程读写; 锁形态见
// virtio_blk.h 顶段)
// ----------------------------------------------------------------------------
//
// 字段 NOT _Atomic — 走 pthread_mutex_t (state_mutex + queue_mutex 包装); 锁
// 本身是 happens-before 边界, _Atomic 在锁内冗余 (跟 PLIC/UART 同思路)。
//
// io_worker_thread: pthread_t 句柄, BSS 0 init; spawn fail / spawn skip
// (image_fd==-1 或 SDS=0) 时 POSIX 不修改 thread 参数, pthread_join(0) glibc 下
// 返 ESRCH 一行 fprintf 不 fatal (跟 clint/uart/plic 同体例; dummy.txt §12)。

static struct {
    /* image backing — pread/pwrite (host file 后端; destroy 时 fsync 一次防丢盘) */
    int       image_fd;                /* -1 = no image (整个模块退化) */
    uint64_t  capacity_sectors;        /* st_size / VIRTIO_BLK_SECTOR_SIZE */

    /* mmio 寄存器子集 (state_mutex 保护) */
    pthread_mutex_t state_mutex;
    uint32_t  status;
    uint32_t  device_feat_sel;
    uint32_t  driver_feat_lo;
    uint32_t  driver_feat_hi;
    uint32_t  driver_feat_sel;
    uint32_t  queue_sel;
    uint32_t  queue_num;
    uint32_t  queue_align;             /* legacy silent accept, 默认按 4 KB */
    uint32_t  queue_pfn;               /* 4 KB 单元 */
    uint32_t  interrupt_status;        /* bit0 = used buffer notification */
    uint16_t  last_avail_idx;          /* worker drain 进度; queue_pfn=0 时清 0 */

    /* work queue — hart enqueue / worker dequeue; cap=8 */
    pthread_mutex_t queue_mutex;
    pthread_cond_t  cond_not_empty;
    pthread_cond_t  cond_not_full;
    /* ring: head 写位 / tail 读位; 浪费一个 slot 区分空 vs 满 (跟 PLIC refresh
       queue 同体例). work item = 空 token (v1 单 queue, queue_sel 永远 0), buf
       本体不携带信息, 只用 head/tail 计数. */
    uint32_t  work_head;
    uint32_t  work_tail;
    uint8_t   work_buf[VIRTIO_BLK_WORK_QUEUE_CAP];

    pthread_t io_worker_thread;
} g_vblk;


// ----------------------------------------------------------------------------
// 锁 helper (file-static)
// ----------------------------------------------------------------------------

static void vblk_state_lock  (void) { (void)pthread_mutex_lock  (&g_vblk.state_mutex); }
static void vblk_state_unlock(void) { (void)pthread_mutex_unlock(&g_vblk.state_mutex); }
static void vblk_queue_lock  (void) { (void)pthread_mutex_lock  (&g_vblk.queue_mutex); }
static void vblk_queue_unlock(void) { (void)pthread_mutex_unlock(&g_vblk.queue_mutex); }


// ----------------------------------------------------------------------------
// vring 布局 helper (legacy v1.0 spec §2.4.2 + Linux vring_size)
// ----------------------------------------------------------------------------
//
//   desc[N]:   16*N bytes (off 0)
//   avail:     6 + 2*N bytes (flags + idx + ring[N] + used_event;
//              legacy 总预留 used_event 即使不启用 EVENT_IDX)
//   pad:       到 queue_align (legacy 默认 4 KB) 对齐
//   used:      6 + 8*N bytes (flags + idx + ring[N]{id,len} + avail_event)
//
// 这跟 QEMU virtio_legacy / Linux drivers/virtio/virtio_ring.c 一致。

static inline uint32_t vblk_avail_off(uint32_t qnum) {
    return 16u * qnum;
}

static inline uint32_t vblk_used_off(uint32_t qnum, uint32_t qalign) {
    uint32_t avail_end = 16u * qnum + 6u + 2u * qnum;
    return (avail_end + qalign - 1u) & ~(qalign - 1u);
}


// ----------------------------------------------------------------------------
// drain_one_avail_round_locked — 消费 avail ring 一轮 (调用方持 state_mutex)
// ----------------------------------------------------------------------------
//
// 跑完后所有 (avail.idx - last_avail_idx) 笔 IO 都已 pread/pwrite + 写 used
// ring; 触发 IRQ 由调用方 unlock state_mutex 后调 device_set_pending (避免锁
// 嵌套, 跟 InterruptACK 同体例)。
//
// 健壮性: queue_pfn=0 / queue_num=0 / qnum 超上限 / vring 不在 RAM 区 → 静默
// 返回 (fixture 错配置不该让 emulator crash). desc 链 hop 上限 VBLK_DESC_CHAIN_MAX
// 防恶意 cycle / 长链.
//
// guest 内存通过 gpa_to_hva_offset + pa 直读直写 (跟 PLIC test_dev RAM 直通同
// 体例; guest 内存不进 virtio_blk 内部锁 — 单 hart guest 写完 avail 才 sw
// QueueNotify, happens-before 已由 worker thread cond_signal 边界保证).

static void drain_one_avail_round_locked(void) {
    if (g_vblk.queue_pfn == 0u || g_vblk.queue_num == 0u) return;
    if (g_vblk.queue_num > VIRTIO_BLK_QUEUE_NUM_MAX)     return;

    uint32_t qnum     = g_vblk.queue_num;
    uint32_t qalign   = (g_vblk.queue_align != 0u) ? g_vblk.queue_align : 4096u;
    uint32_t base_pa  = g_vblk.queue_pfn * 4096u;
    uint32_t avail_o  = vblk_avail_off(qnum);
    uint32_t used_o   = vblk_used_off(qnum, qalign);
    uint32_t vring_sz = used_o + 6u + 8u * qnum;

    /* bounds check: 整个 vring 必须在 RAM 区 */
    if (!IS_GPA_RAM(base_pa) || !IS_GPA_RAM(base_pa + vring_sz - 1u)) {
        fprintf(stderr, "[virtio_blk] vring out of RAM: pfn=0x%x size=0x%x\n",
                g_vblk.queue_pfn, vring_sz);
        return;
    }

    uint8_t *desc_base  = gpa_to_hva_offset + base_pa;
    uint8_t *avail_base = desc_base + avail_o;
    uint8_t *used_base  = desc_base + used_o;

    /* 读 avail.idx (host LE, guest LE, 直读) */
    uint16_t avail_idx;
    memcpy(&avail_idx, avail_base + 2, 2);

    int produced_used = 0;

    while (g_vblk.last_avail_idx != avail_idx) {
        uint16_t head_desc_idx;
        memcpy(&head_desc_idx,
               avail_base + 4 + 2u * (g_vblk.last_avail_idx % qnum), 2);

        /* 跑 desc 链: 第一段必须是 header (16 byte); 中间是 data; 末段是 1 byte
           status (no NEXT)。VRING_DESC_F_WRITE 标 device-writable (即 IN/读 IO
           的 data + status 必为 W=1; OUT/写 IO 的 data 必为 W=0, status 仍为 W=1)。
           简化: fixture 守规矩, 不做严格 W bit 校验, 只按 type + NEXT/last 链结构走. */
        uint8_t  status_byte   = VIRTIO_BLK_S_OK;
        uint8_t *status_addr   = NULL;
        uint64_t sector        = 0;
        uint32_t req_type      = 0;
        int      header_parsed = 0;
        uint32_t total_len     = 0;
        uint16_t cur           = head_desc_idx;

        for (uint32_t hop = 0; hop < VBLK_DESC_CHAIN_MAX; hop++) {
            if (cur >= qnum) { status_byte = VIRTIO_BLK_S_IOERR; break; }

            uint8_t *desc = desc_base + 16u * cur;
            uint64_t addr;  uint32_t len;  uint16_t flags;  uint16_t next;
            memcpy(&addr,  desc + 0,  8);
            memcpy(&len,   desc + 8,  4);
            memcpy(&flags, desc + 12, 2);
            memcpy(&next,  desc + 14, 2);

            if (flags & VRING_DESC_F_INDIRECT) {
                /* DeviceFeatures=0 没宣告 INDIRECT, 但 driver 误用时拒绝 */
                status_byte = VIRTIO_BLK_S_UNSUPP; break;
            }

            /* RV32 guest, 高 32 位 addr 必须 0 */
            if ((addr >> 32) != 0u) { status_byte = VIRTIO_BLK_S_IOERR; break; }
            uint32_t pa = (uint32_t)addr;
            if (len == 0u) { status_byte = VIRTIO_BLK_S_IOERR; break; }
            if (!IS_GPA_RAM(pa) || !IS_GPA_RAM(pa + len - 1u)) {
                status_byte = VIRTIO_BLK_S_IOERR; break;
            }
            uint8_t *hva = gpa_to_hva_offset + pa;

            int last_in_chain = ((flags & VRING_DESC_F_NEXT) == 0);

            if (!header_parsed) {
                /* 第一段必须是 header */
                if (len < 16u) { status_byte = VIRTIO_BLK_S_IOERR; break; }
                uint32_t type_lo;
                memcpy(&type_lo, hva + 0, 4);
                memcpy(&sector,  hva + 8, 8);
                req_type      = type_lo;
                header_parsed = 1;
                if (last_in_chain) {
                    /* 单段链 (只 header) 不合法; 走 IOERR + 没 status 写 */
                    status_byte = VIRTIO_BLK_S_IOERR; break;
                }
                cur = next;
                continue;
            }

            if (last_in_chain) {
                /* 末段 = status (1 byte). 即使 len > 1 也只写头 1 byte. */
                status_addr = hva;
                break;
            }

            /* 中间段 = data */
            if (req_type == VIRTIO_BLK_T_IN) {
                /* host file → guest buf */
                ssize_t n = pread(g_vblk.image_fd, hva, (size_t)len,
                                  (off_t)(sector * (uint64_t)VIRTIO_BLK_SECTOR_SIZE));
                if (n != (ssize_t)len) {
                    fprintf(stderr, "[virtio_blk] pread sector=%lu len=%u failed: %s\n",
                            (unsigned long)sector, len,
                            (n < 0) ? strerror(errno) : "short read");
                    status_byte = VIRTIO_BLK_S_IOERR;
                }
                total_len += len;
            } else if (req_type == VIRTIO_BLK_T_OUT) {
                ssize_t n = pwrite(g_vblk.image_fd, hva, (size_t)len,
                                   (off_t)(sector * (uint64_t)VIRTIO_BLK_SECTOR_SIZE));
                if (n != (ssize_t)len) {
                    fprintf(stderr, "[virtio_blk] pwrite sector=%lu len=%u failed: %s\n",
                            (unsigned long)sector, len,
                            (n < 0) ? strerror(errno) : "short write");
                    status_byte = VIRTIO_BLK_S_IOERR;
                }
            } else {
                status_byte = VIRTIO_BLK_S_UNSUPP;
            }
            /* IO 后 sector 推进 (len/512); 多段 data 时跨 sector 顺序 IO */
            sector += (uint64_t)len / VIRTIO_BLK_SECTOR_SIZE;

            cur = next;
        }

        if (status_addr != NULL) {
            *status_addr = status_byte;
        }
        /* spec §2.6.8.2: used.ring[].len = total bytes device wrote 进 buf;
           OUT 路径 device 没写 data 段, 仅写 status byte → len = 1;
           IN 路径 device 写了 data + status → len = total_len + 1。
           简化: 不强分支, total_len 累加 data IN 的 len (OUT 路径不累加),
           最后 +1 包 status byte (如果有 status_addr)。 */
        uint32_t reported_len = total_len + (status_addr != NULL ? 1u : 0u);

        /* 写 used.ring[used.idx % qnum] = { head_desc_idx, reported_len } */
        uint16_t used_idx;
        memcpy(&used_idx, used_base + 2, 2);
        uint8_t *used_elem = used_base + 4 + 8u * ((uint32_t)used_idx % qnum);
        uint32_t id32 = head_desc_idx;
        memcpy(used_elem + 0, &id32,         4);
        memcpy(used_elem + 4, &reported_len, 4);

        /* release fence: guest 读 used.idx 看到新值时, 上面 ring[] 写入必先可见
           (跟 spec §2.6.8.2 ordering 一致; x86 strong model 下硬件已保, 加 fence
           跨 host 安全). */
        atomic_thread_fence(memory_order_release);
        used_idx++;
        memcpy(used_base + 2, &used_idx, 2);

        g_vblk.last_avail_idx++;
        produced_used = 1;
    }

    if (produced_used) {
        g_vblk.interrupt_status |= VBLK_INT_VRING;
    }
}


// ----------------------------------------------------------------------------
// io_worker_run — 后台 thread routine (file-static; 跟 plic_pending_refresh_run
// 同 cooperative shutdown 体例; dummy.txt §12)
// ----------------------------------------------------------------------------
//
// 循环: cond_timedwait not_empty 100ms 心跳检 SDS → dequeue 1 token → 释放
// queue_mutex → lock state_mutex drain_one_avail_round_locked → unlock state →
// 持 IRQ flag 时 device_set_pending(VIRTIO_BLK_PLIC_IRQ) (锁外, 避免嵌套)。
//
// 单 token = 单次 QueueNotify 通知; worker drain 一轮 (按 avail.idx 推进) 即可
// 消费当时所有 entry; 后续 QueueNotify 若赶不上 worker (worker 还在 drain) 也
// 等效 — drain 时已经看到最新 avail.idx, token 多消费几次 = no-op。

static void *io_worker_run(void *arg) {
    (void)arg;

    while (atomic_load_explicit(&shutdown_signal, memory_order_acquire) == 0u) {
        int have_token = 0;

        vblk_queue_lock();

        /* 等 not_empty 或 timeout; while 重检防 spurious wakeup + SDS 退 */
        while (g_vblk.work_head == g_vblk.work_tail &&
               atomic_load_explicit(&shutdown_signal, memory_order_acquire) == 0u) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_nsec += 100000000L;   /* 100 ms */
            if (ts.tv_nsec >= 1000000000L) {
                ts.tv_sec  += ts.tv_nsec / 1000000000L;
                ts.tv_nsec %= 1000000000L;
            }
            int rc = pthread_cond_timedwait(&g_vblk.cond_not_empty,
                                            &g_vblk.queue_mutex, &ts);
            if (rc == ETIMEDOUT) continue;   /* 重检 SDS + queue */
            if (rc != 0) {
                fprintf(stderr, "[virtio_blk worker] cond_timedwait failed: %s\n",
                        strerror(rc));
                vblk_queue_unlock();
                shutdown_signal_set_bit(SHUTDOWN_BIT_DEVICE_FAIL);
                return NULL;
            }
            /* rc == 0: signaled or spurious; while 重检 */
        }

        if (g_vblk.work_head != g_vblk.work_tail) {
            g_vblk.work_tail = (g_vblk.work_tail + 1u) % VIRTIO_BLK_WORK_QUEUE_CAP;
            have_token = 1;
            pthread_cond_signal(&g_vblk.cond_not_full);
        }

        vblk_queue_unlock();

        if (!have_token) continue;   /* SDS=0 退路: while 退出后 queue 仍空 */

        /* drain: 持 state_mutex (单条临界区跑完释放, 不嵌套 queue_mutex) */
        int need_irq = 0;
        vblk_state_lock();
        if (g_vblk.status != 0u) {
            uint32_t before = g_vblk.interrupt_status;
            drain_one_avail_round_locked();
            if ((g_vblk.interrupt_status & VBLK_INT_VRING) &&
                !(before & VBLK_INT_VRING)) {
                need_irq = 1;
            }
        }
        vblk_state_unlock();

        if (need_irq) {
            device_set_pending((uint32_t)VIRTIO_BLK_PLIC_IRQ);
        }
    }

    return NULL;
}


// ----------------------------------------------------------------------------
// vblk_enqueue_work_token — QueueNotify 入口 helper (file-static; hart 主帧调)
// ----------------------------------------------------------------------------
//
// 跟 plic_enqueue_refresh 同体例: queue 满则 cond_timedwait not_full 100ms 心跳
// 检 SDS, 跟 PLIC refresh queue producer 完全对偶。
//
// cond_timedwait 三种返回 (跟 PLIC 同):
//   - rc=0           consumer 拉走或 spurious wakeup; while 重检 queue_full
//   - rc=ETIMEDOUT   检 SDS, 触发 → 放弃这笔直返 (POR 收尾路径; hart 不该再
//                    产生 IO); SDS 未触发 → continue 再 wait 一轮
//   - rc 其他        真异常, log + 放弃 + 触发 SRS=0+SDS=0 走 cleanup
//
// 锁顺序: 只持 queue_mutex, 不碰 state_mutex (锁顺序硬约束, virtio_blk.h 顶段)。

static void vblk_enqueue_work_token(void) {
    vblk_queue_lock();

    while (((g_vblk.work_head + 1u) % VIRTIO_BLK_WORK_QUEUE_CAP) == g_vblk.work_tail) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 100000000L;   /* 100 ms */
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec  += ts.tv_nsec / 1000000000L;
            ts.tv_nsec %= 1000000000L;
        }
        int rc = pthread_cond_timedwait(&g_vblk.cond_not_full,
                                        &g_vblk.queue_mutex, &ts);
        if (rc == ETIMEDOUT) {
            if (atomic_load_explicit(&shutdown_signal, memory_order_acquire) != 0u) {
                vblk_queue_unlock();
                return;
            }
            continue;   /* SDS 仍 0 (允许执行), 再 wait 一轮 */
        }
        if (rc != 0) {
            fprintf(stderr, "[virtio_blk] enqueue cond_timedwait failed: %s\n",
                    strerror(rc));
            vblk_queue_unlock();
            shutdown_signal_set_bit(SHUTDOWN_BIT_DEVICE_FAIL);
            return;
        }
        /* rc == 0: signaled, while 重检 */
    }

    g_vblk.work_buf[g_vblk.work_head] = 0;   /* token 无信息, 占位 */
    g_vblk.work_head = (g_vblk.work_head + 1u) % VIRTIO_BLK_WORK_QUEUE_CAP;
    pthread_cond_signal(&g_vblk.cond_not_empty);

    vblk_queue_unlock();
}


// ----------------------------------------------------------------------------
// virtio_blk_read / virtio_blk_write — bus 派发入口
// ----------------------------------------------------------------------------
//
// 接口语义 (dummy.txt §9): 返 0=成功; 返非 0=cause。
//
// fault 政策: size != 4 → CAUSE_*_ACCESS_FAULT; 其他 off 越界 / reserved →
// silent (R 返 0 / W 吞).
//
// 锁顺序: mmio 顶层不持锁; 各 case 按需 lock (state_mutex 短临界区或
// queue_mutex 单独路径). QueueNotify case **只持 queue_mutex**, 不碰
// state_mutex (锁顺序硬约束, 详 virtio_blk.h). InterruptACK case 持 state_mutex 改完
// unlock 后调 device_clear_pending (PLIC 自有并发安全, 不嵌套)。

static int virtio_blk_read(void *ctx, uint32_t off, void *buf, uint32_t size) {
    (void)ctx;
    if (size != 4u) return CAUSE_LOAD_ACCESS_FAULT;

    uint32_t value = 0;

    switch (off) {
      case VBLK_REG_MAGIC:           value = VBLK_MAGIC;       break;
      case VBLK_REG_VERSION:         value = VBLK_VERSION;     break;
      case VBLK_REG_DEVICE_ID:       value = VBLK_DEVICE_ID;   break;
      case VBLK_REG_VENDOR_ID:       value = VBLK_VENDOR_ID;   break;
      case VBLK_REG_DEVICE_FEAT:     value = 0u;               break;   /* 无可选 feature */
      case VBLK_REG_QUEUE_NUM_MAX:   value = VIRTIO_BLK_QUEUE_NUM_MAX; break;

      case VBLK_REG_QUEUE_PFN:
        vblk_state_lock();
        value = g_vblk.queue_pfn;
        vblk_state_unlock();
        break;

      case VBLK_REG_INT_STATUS:
        vblk_state_lock();
        value = g_vblk.interrupt_status;
        vblk_state_unlock();
        break;

      case VBLK_REG_STATUS:
        vblk_state_lock();
        value = g_vblk.status;
        vblk_state_unlock();
        break;

      case VBLK_REG_CONFIG_CAP_LO:
        value = (uint32_t)g_vblk.capacity_sectors;            /* image_fd 后不变, 不锁 */
        break;
      case VBLK_REG_CONFIG_CAP_HI:
        value = (uint32_t)(g_vblk.capacity_sectors >> 32);
        break;

      default:
        /* reserved / 未实装寄存器 silent 返 0 */
        value = 0u;
        break;
    }

    memcpy(buf, &value, 4);
    return 0;
}

static int virtio_blk_write(void *ctx, uint32_t off, const void *buf, uint32_t size) {
    (void)ctx;
    if (size != 4u) return CAUSE_STORE_ACCESS_FAULT;

    uint32_t value;
    memcpy(&value, buf, 4);

    switch (off) {
      case VBLK_REG_DEVICE_FEAT_SEL:
        vblk_state_lock();  g_vblk.device_feat_sel = value;   vblk_state_unlock();  break;

      case VBLK_REG_DRIVER_FEAT:
        vblk_state_lock();
        if (g_vblk.driver_feat_sel == 0u) g_vblk.driver_feat_lo = value;
        else                              g_vblk.driver_feat_hi = value;
        vblk_state_unlock();
        break;

      case VBLK_REG_DRIVER_FEAT_SEL:
        vblk_state_lock();  g_vblk.driver_feat_sel = value;   vblk_state_unlock();  break;

      case VBLK_REG_GUEST_PAGE_SIZE:
        /* legacy silent accept; 假设 4 KB */
        break;

      case VBLK_REG_QUEUE_SEL:
        vblk_state_lock();  g_vblk.queue_sel = value;         vblk_state_unlock();  break;

      case VBLK_REG_QUEUE_NUM:
        vblk_state_lock();
        if (value <= VIRTIO_BLK_QUEUE_NUM_MAX) g_vblk.queue_num = value;
        vblk_state_unlock();
        break;

      case VBLK_REG_QUEUE_ALIGN:
        vblk_state_lock();  g_vblk.queue_align = value;       vblk_state_unlock();  break;

      case VBLK_REG_QUEUE_PFN:
        vblk_state_lock();
        g_vblk.queue_pfn = value;
        g_vblk.last_avail_idx = 0;   /* 写 PFN 复位 drain 进度 (跟 spec 一致) */
        vblk_state_unlock();
        break;

      case VBLK_REG_QUEUE_NOTIFY: {
        /* 热路径: 只持 queue_mutex 入 work queue, 不碰 state_mutex (锁顺序硬
           约束, 详 virtio_blk.h). v1 单 queue, value 应为 0; 不强校验. */
        (void)value;
        vblk_enqueue_work_token();
        break;
      }

      case VBLK_REG_INT_ACK: {
        int cleared = 0;
        vblk_state_lock();
        uint32_t before = g_vblk.interrupt_status;
        g_vblk.interrupt_status &= ~value;
        if (before != 0u && g_vblk.interrupt_status == 0u) cleared = 1;
        vblk_state_unlock();
        if (cleared) {
            device_clear_pending((uint32_t)VIRTIO_BLK_PLIC_IRQ);
        }
        break;
      }

      case VBLK_REG_STATUS:
        vblk_state_lock();
        if (value == 0u) {
            /* 写 0 = reset 寄存器 (spec §4.2.2.1); InterruptStatus 也清 */
            g_vblk.status            = 0;
            g_vblk.device_feat_sel   = 0;
            g_vblk.driver_feat_lo    = 0;
            g_vblk.driver_feat_hi    = 0;
            g_vblk.driver_feat_sel   = 0;
            g_vblk.queue_sel         = 0;
            g_vblk.queue_num         = 0;
            g_vblk.queue_align       = 0;
            g_vblk.queue_pfn         = 0;
            g_vblk.interrupt_status  = 0;
            g_vblk.last_avail_idx    = 0;
        } else {
            g_vblk.status = value;
        }
        vblk_state_unlock();
        if (value == 0u) {
            device_clear_pending((uint32_t)VIRTIO_BLK_PLIC_IRQ);
        }
        break;

      default:
        /* reserved / RO 寄存器写 silent 吞 */
        break;
    }

    return 0;
}


// ----------------------------------------------------------------------------
// lifecycle: virtio_blk_init / virtio_blk_reset / virtio_blk_destroy
// ----------------------------------------------------------------------------

int virtio_blk_init(const char *image_path) {
    memset(&g_vblk, 0, sizeof(g_vblk));
    g_vblk.image_fd = -1;

    /* image_path==NULL: 整个模块退化 (不 bus 注册, lifecycle 不持任何资源) */
    if (image_path == NULL) {
        return 0;
    }

    int fd = open(image_path, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "virtio_blk_init: open(%s) failed: %s\n",
                image_path, strerror(errno));
        return -1;
    }
    struct stat st;
    if (fstat(fd, &st) != 0) {
        fprintf(stderr, "virtio_blk_init: fstat(%s) failed: %s\n",
                image_path, strerror(errno));
        (void)close(fd);
        return -1;
    }
    if (!S_ISREG(st.st_mode)) {
        fprintf(stderr, "virtio_blk_init: %s is not a regular file\n", image_path);
        (void)close(fd);
        return -1;
    }
    if (st.st_size < 0 ||
        ((uint64_t)st.st_size % (uint64_t)VIRTIO_BLK_SECTOR_SIZE) != 0u) {
        fprintf(stderr, "virtio_blk_init: %s size %lld not %u-aligned\n",
                image_path, (long long)st.st_size, VIRTIO_BLK_SECTOR_SIZE);
        (void)close(fd);
        return -1;
    }

    g_vblk.image_fd          = fd;
    g_vblk.capacity_sectors  = (uint64_t)st.st_size / (uint64_t)VIRTIO_BLK_SECTOR_SIZE;

    int rc = pthread_mutex_init(&g_vblk.state_mutex, NULL);
    if (rc != 0) {
        fprintf(stderr, "virtio_blk_init: state_mutex init failed: %s\n", strerror(rc));
        (void)close(fd); g_vblk.image_fd = -1; return -1;
    }
    rc = pthread_mutex_init(&g_vblk.queue_mutex, NULL);
    if (rc != 0) {
        fprintf(stderr, "virtio_blk_init: queue_mutex init failed: %s\n", strerror(rc));
        (void)pthread_mutex_destroy(&g_vblk.state_mutex);
        (void)close(fd); g_vblk.image_fd = -1; return -1;
    }
    rc = pthread_cond_init(&g_vblk.cond_not_empty, NULL);
    if (rc != 0) {
        fprintf(stderr, "virtio_blk_init: cond_not_empty init failed: %s\n", strerror(rc));
        (void)pthread_mutex_destroy(&g_vblk.queue_mutex);
        (void)pthread_mutex_destroy(&g_vblk.state_mutex);
        (void)close(fd); g_vblk.image_fd = -1; return -1;
    }
    rc = pthread_cond_init(&g_vblk.cond_not_full, NULL);
    if (rc != 0) {
        fprintf(stderr, "virtio_blk_init: cond_not_full init failed: %s\n", strerror(rc));
        (void)pthread_cond_destroy (&g_vblk.cond_not_empty);
        (void)pthread_mutex_destroy(&g_vblk.queue_mutex);
        (void)pthread_mutex_destroy(&g_vblk.state_mutex);
        (void)close(fd); g_vblk.image_fd = -1; return -1;
    }

    mmio_dev_t dev = {
        .gpa_start = (uint32_t)VIRTIO_BLK_BASE,
        .gpa_end   = (uint32_t)(VIRTIO_BLK_BASE + VIRTIO_BLK_SIZE),
        .ctx       = &g_vblk,
        .read      = virtio_blk_read,
        .write     = virtio_blk_write,
        .name      = "virtio_blk",
    };
    if (bus_register_mmio(&dev) != 0) {
        fprintf(stderr, "virtio_blk_init: bus_register_mmio failed\n");
        (void)pthread_cond_destroy  (&g_vblk.cond_not_full);
        (void)pthread_cond_destroy  (&g_vblk.cond_not_empty);
        (void)pthread_mutex_destroy (&g_vblk.queue_mutex);
        (void)pthread_mutex_destroy (&g_vblk.state_mutex);
        (void)close(fd); g_vblk.image_fd = -1; return -1;
    }
    return 0;
}

int virtio_blk_reset(void) {
    if (g_vblk.image_fd < 0) return 0;

    vblk_state_lock();
    g_vblk.status            = 0;
    g_vblk.device_feat_sel   = 0;
    g_vblk.driver_feat_lo    = 0;
    g_vblk.driver_feat_hi    = 0;
    g_vblk.driver_feat_sel   = 0;
    g_vblk.queue_sel         = 0;
    g_vblk.queue_num         = 0;
    g_vblk.queue_align       = 0;
    g_vblk.queue_pfn         = 0;
    g_vblk.interrupt_status  = 0;
    g_vblk.last_avail_idx    = 0;
    vblk_state_unlock();

    /* work queue 不 reset (跟 UART reader / PLIC refresh 体例: 跨 system reset
       一直跑); SR 期间未消费的 token 等于"已经被 reset 抹掉的中断", 走完即可. */

    device_clear_pending((uint32_t)VIRTIO_BLK_PLIC_IRQ);
    return 0;
}

void virtio_blk_destroy(void) {
    if (g_vblk.image_fd < 0) return;

    /* 关 fd 前 fsync 一次 — pwrite 后内核 page cache 没刷盘, 极端情况 (host 强杀
       / 断电) 可能丢最近 sector write. fsync fail 仅 log 不 fatal (destroy 在
       shutdown 末段无 fallback; 真 fail 也 close 走). 不做 per-IO fsync (太慢). */
    if (fsync(g_vblk.image_fd) != 0) {
        fprintf(stderr, "[virtio_blk] destroy: fsync failed: %s\n", strerror(errno));
    }

    (void)close(g_vblk.image_fd);
    g_vblk.image_fd = -1;

    (void)pthread_cond_destroy  (&g_vblk.cond_not_full);
    (void)pthread_cond_destroy  (&g_vblk.cond_not_empty);
    (void)pthread_mutex_destroy (&g_vblk.queue_mutex);
    (void)pthread_mutex_destroy (&g_vblk.state_mutex);
}


// ----------------------------------------------------------------------------
// thread lifecycle: virtio_blk_start_io_worker_thread / virtio_blk_join_io_worker_thread
// (dummy.txt §12; 跟 plic_start/join_pending_refresh_thread 同体例)
// ----------------------------------------------------------------------------

void virtio_blk_start_io_worker_thread(void) {
    /* image_fd<0 退化路径: 不 spawn (work queue 永远空, worker 跑空转无意义);
       io_worker_thread 保 BSS 0, join 时 ESRCH 一行 fprintf 不 fatal. */
    if (g_vblk.image_fd < 0) {
        return;
    }
    if (atomic_load_explicit(&shutdown_signal, memory_order_acquire) != 0u) {
        return;
    }

    int rc = pthread_create(&g_vblk.io_worker_thread, NULL, io_worker_run, NULL);
    if (rc != 0) {
        fprintf(stderr, "virtio_blk_start_io_worker_thread: pthread_create failed: %s\n",
                strerror(rc));
        shutdown_signal_set_bit(SHUTDOWN_BIT_DEVICE_FAIL);
    }
}

void virtio_blk_join_io_worker_thread(void) {
    if (g_vblk.image_fd < 0) {
        return;   /* spawn skip 路径下 io_worker_thread 是 BSS 0, 不必 join */
    }
    int rc = pthread_join(g_vblk.io_worker_thread, NULL);
    if (rc != 0) {
        fprintf(stderr, "virtio_blk_join_io_worker_thread: pthread_join failed: %s\n",
                strerror(rc));
    }
}
