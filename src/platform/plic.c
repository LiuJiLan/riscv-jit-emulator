//
// Created by liujilan on 2026/5/23.
// PLIC 实现 — per-source <device_line, claimed, priority> + per-ctx <threshold,
// enable bitmap> + plic_ctx_map + bus 注册 + 完整 claim/complete 仲裁 + hart 侧
// is_plic_*_pending 合成 view + 外设侧 device_set/clear_pending 接口。
//
// T6.2 升级: 加 refresh ring + plic_pending_refresh_thread + plic_ctx_eip atomic
// 子集, 把"pending → MEIP/SEIP_hw"路径从同步 rwlock 拉成异步消费, dispatcher 主帧
// is_plic_*_pending 改 atomic_load 直返 (零 lock); claim/complete 慢路径仍持 wrlock
// 不变. 详 plic.h 顶段 monitor 模型段 + dummy.txt §7 PLIC entry。
//
// 接口形态 + monitor 模型 + rwlock 抽象 + 字段对应 spec + 命名选择见 plic.h 顶段 doc。
// 地址布局见 config.h PLIC_* 宏 (跟 QEMU virt + RV PLIC spec v1.0.0 兼容)。
// 报错风格见 dummy.txt §5; "0=成功 / 非0=cause" 接口约定见 dummy.txt §9。
// thread spawn/join 协议 (谁 spawn 谁 join) 见 dummy.txt §12。
//

#define _POSIX_C_SOURCE 200809L   // clock_gettime CLOCK_REALTIME / pthread_cond_timedwait

#include "plic.h"

#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "config.h"          // PLIC_* / MAX_HARTS / PLIC_REFRESH_QUEUE_CAP
#include "platform/bus.h"    // mmio_dev_t / bus_register_mmio
#include "riscv.h"           // CAUSE_LOAD/STORE_ACCESS_FAULT / PRIV_M / PRIV_S
#include "runtime.h"         // system_reset_signal / shutdown_signal


// ----------------------------------------------------------------------------
// 锁顺序协议 (避免 deadlock; well-define 在此模块顶部, 任何 helper / lifecycle 路径
// 都遵循; dummy.txt §7 PLIC entry "锁顺序" 段同步)
// ----------------------------------------------------------------------------
//
// 模块内有两把锁: plic_refresh_queue.lock (mutex) + plic.lock (rwlock); 跨锁路径
// 都遵循"从不同时持两个锁"协议, 拿一个锁的临界区跑完 + 释放, 再拿下一个锁。
//
//   refresh thread:        mutex 拉 event → 释放 mutex → wrlock 算 + 改 plic +
//                          plic_ctx_eip → 释放 wrlock
//   plic_reset:            mutex 清 queue (head=tail=0 + broadcast cond) → 释放
//                          mutex → wrlock 清 plic.sources/contexts + plic_ctx_eip
//                          → 释放 wrlock
//   plic_enqueue_refresh:  只持 mutex; 满则 cond_timedwait not_full 100ms 心跳检 SDS
//   hart is_plic_*_pending: 不持任何锁; atomic_load(plic_ctx_eip[ctx_id])
//   hart claim/complete (plic_read/write): 只持 wrlock (跟 priority/threshold/enable 写一致)
//
// 单向, 不存在交叉路径 → 无 deadlock 风险。


// ----------------------------------------------------------------------------
// PLIC 内部状态 (单例 file-static; rwlock 保护)
// ----------------------------------------------------------------------------
//
// 字段 NOT _Atomic — PLIC 走 pthread_rwlock_t (plic_rdlock / plic_wrlock 包装), 锁本身
// 是 happens-before 边界, _Atomic 在锁内冗余 (plan §1.9 "shared 数据 atomic 起步"
// 针对 clint 那种无锁场景: timer thread 跟主帧并发 mtime; 有锁场景不适用)。SMP 真切
// 锁拆细 / RCU 时才需要补 _Atomic。
//
// sources 数组: per-source 三字段:
//   device_line — 设备拉线状态 (level 模型镜像); T6.2 后: 由 plic_pending_refresh_thread
//                 持 wrlock 改 (消费 SET/CLEAR event); device_set/clear_pending 不再
//                 直接写, 改成 plic_enqueue_refresh 入队
//   claimed     — gateway forward latch (0=未 claim / 非 0 = ctx_id + 1; +1 编码避坑
//                 Linux 启动遇 ctx_id=-1 退出 init)
//   priority    — RV PLIC source priority (0 = 永不触发, RV PLIC spec v1.0.0 §4)
//
// contexts 数组: per-ctx 两字段:
//   threshold   — context priority threshold (priority > threshold 才送; reset=0 接受所有 >0)
//   enable      — context interrupt enable bitmap, 1 bit/source, (N_SOURCES+31)/32 words
//                 (reset=0 全 disable, RV PLIC spec 默认; v1 全广播语义靠 fixture 显式写)
//
// plic_ctx_map: hart_priv → ctx_id 反向映射。index = (hartid << 2) + priv (复用
// cpu_t.tlb_table priv encoding: U=0/S=1/VS=2/M=3); 元素 = 0..PLIC_N_CONTEXTS-1;
// -1 = 没连线。v1 plic_init 内 hardcoded 填: 每 hart MSU 各 2 ctx, M ctx 小 S ctx 大
// (跟 QEMU virt sifive_plic 一致)。
//
// lock: pthread_rwlock_t; helper plic_rdlock / plic_wrlock / plic_unlock 包装 (file-
// static, 调用方都是 plic.c 内部函数, 不对外暴露)。
static struct {
    struct {
        uint8_t   device_line;     /* bool 形态; 设备拉线状态 (level 模型镜像) */
        uint32_t  claimed;         /* 0=未 claim; 非 0 = ctx_id + 1 (PLIC 内部编码) */
        uint32_t  priority;        /* RV PLIC source priority; 0=disabled */
    } sources[PLIC_N_SOURCES];     /* source 0 保留, 索引从 1 起用 */
    struct {
        uint32_t  threshold;
        uint32_t  enable[(PLIC_N_SOURCES + 31u) / 32u];   /* bitmap */
    } contexts[PLIC_N_CONTEXTS];
    int8_t            plic_ctx_map[MAX_HARTS * 4];        /* index = hartid<<2+priv; -1 = 没连线 */
    pthread_rwlock_t  lock;
} plic;


// ----------------------------------------------------------------------------
// plic_ctx_eip — per-ctx external interrupt pending atomic 子集 (T6.2 新加)
// ----------------------------------------------------------------------------
//
// 独立 file-static, NOT 进 plic struct; 跟 CLINT 全 atomic monitor 体例对偶
// (CLINT 字段直 _Atomic file-static, 没 rwlock); PLIC 的"atomic 子集"提取出来
// 跟 CLINT 一个味道 — atomic 自带 ordering, 不持 lock。
//
// 语义: per-ctx visible-pending boolean. consumer = is_plic_meip/seip_pending
// (dispatcher 主帧 csr_mip_read 调) 直接 atomic_load_explicit(acquire), 零 lock,
// 是 T6.2 的核心 fast path。
//
// producer 路径 (所有写都持 plic.lock wrlock, atomic_store_explicit(release)):
//   - plic_pending_refresh_thread 消费 SET/CLEAR event 后重算
//   - plic_read 命中 claim 副作用 (set claimed 后重算)
//   - plic_write 命中 complete (清 claimed 后重算)
//   - plic_write 命中 priority/threshold/enable 写后重算
//   - plic_reset 清 0
//   - plic_init 清 0
//
// wrlock 不是为了保护 plic_ctx_eip (atomic 自带 ordering); 是为了保证 sources/
// contexts 改完后 plic_ctx_eip 反映最新 effective view (单次 wrlock 临界区内
// "状态改 + view 重算 + view atomic_store" 三件事原子可见)。
//
// 数组按 ctx_id 索引 (= 0..PLIC_N_CONTEXTS-1); is_plic_*_pending 内先查
// plic_ctx_map[(hartid<<2)|priv] 拿 ctx_id, 再 atomic_load(plic_ctx_eip[ctx_id])。
static _Atomic int plic_ctx_eip[PLIC_N_CONTEXTS];


// ----------------------------------------------------------------------------
// plic_refresh_queue — 单 consumer + 多 producer ring buffer (T6.2 新加)
// ----------------------------------------------------------------------------
//
// device_set/clear_pending (producer) 入队 event, plic_pending_refresh_thread
// (consumer) 拉空。ring 经典 head/tail 划分: head 指写位 / tail 指读位; 空 =
// (head == tail); 满 = ((head+1) % CAP == tail), 浪费一个 slot 区分空 vs 满。
//
// op 编码: enum plic_refresh_op_t (SET / CLEAR; 未来加 PRIORITY_CHG / ENABLE_CHG
// 等把 hart MMIO 写也异步化, v1 不做但 enum 留口)。
//
// mutex (不 rwlock — ring 是单 consumer + 多 producer 串行化, rwlock 没意义);
// 双 pthread_cond_t: not_full (consumer 拉走后 signal) / not_empty (producer
// 入队后 signal); producer 满时 cond_timedwait not_full 100ms 心跳检 SDS;
// consumer 空时 cond_timedwait not_empty 100ms 心跳检 SDS (跟 clint timer
// thread / uart reader thread 同 cooperative shutdown 体例; dummy.txt §12)。
//
// queue 独立 file-static, NOT 进 plic struct (跟 plic_ctx_eip 一样独立; 锁形态
// 跟 plic.lock 完全分开, 锁顺序协议见模块顶部)。
//
// pthread_t refresh_thread: BSS 0 init; spawn fail 时 POSIX 不修改 thread 参数,
// pthread_join(0) glibc 下返 ESRCH 容错 (跟 clint.timer_thread / uart.reader_thread
// 同体例; dummy.txt §12)。

typedef enum {
    PLIC_REFRESH_OP_SET   = 0,
    PLIC_REFRESH_OP_CLEAR = 1,
} plic_refresh_op_t;

typedef struct {
    uint32_t  source_id;
    uint8_t   op;                  /* plic_refresh_op_t */
} plic_refresh_event_t;

static struct {
    plic_refresh_event_t  buf[PLIC_REFRESH_QUEUE_CAP];
    uint32_t              head;       /* 写位; producer 入队 */
    uint32_t              tail;       /* 读位; consumer 拉走 */
    pthread_mutex_t       lock;
    pthread_cond_t        not_full;   /* producer 等; consumer 拉走后 signal */
    pthread_cond_t        not_empty;  /* consumer 等; producer 入队后 signal */
    pthread_t             refresh_thread;
} plic_refresh_queue;


// ----------------------------------------------------------------------------
// monitor RW helper (file-static)
// ----------------------------------------------------------------------------
//
// 包装 pthread_rwlock_t 的 rdlock / wrlock / unlock; 不检 rc (POSIX rwlock 在 init
// 成功后, lock/unlock 只在 EDEADLK/EINVAL 等编程错误下失败, v1 不处理 — 真撞 ASan
// + sanitizer 抓住)。SMP 单 hart 下 contention=0, glibc 走 fast path 零开销。
//
// T6.2 后: rdlock 调用方仅剩 plic_read 内 priority/threshold/enable/pending 读
// (is_plic_*_pending 改 atomic_load 不再调 rdlock); wrlock 调用方仍多。

static void plic_rdlock(void) { (void)pthread_rwlock_rdlock(&plic.lock); }
static void plic_wrlock(void) { (void)pthread_rwlock_wrlock(&plic.lock); }
static void plic_unlock(void) { (void)pthread_rwlock_unlock(&plic.lock); }


// ----------------------------------------------------------------------------
// 仲裁 helper (file-static; 调用方已持 wrlock; "_locked" 后缀提醒)
// ----------------------------------------------------------------------------
//
// plic_arbitrate_locked — claim 寄存器读时调; 扫所有 source, 选满足:
//   enable[ctx][src/32] bit (src%32) == 1 AND
//   sources[src].device_line == 1 AND
//   sources[src].claimed == 0 AND
//   sources[src].priority > contexts[ctx].threshold
// 中 priority 最高 (同 priority 选 source_id 小的, RV PLIC spec §7 仲裁规则)。
// 返 source_id; 0 = 无可送 source (跟 spec claim 返 0 一致, source 0 不用所以无歧义)。
//
// plic_ctx_has_pending_locked — refresh thread / plic_recompute_ctx_eip_locked 调;
// 只判 "有任一可送 source", 不算具体哪个 (短路返 1)。

static uint32_t plic_arbitrate_locked(uint32_t ctx_id) {
    uint32_t best_id   = 0;
    uint32_t best_prio = 0;
    uint32_t threshold = plic.contexts[ctx_id].threshold;

    for (uint32_t src = 1; src < PLIC_N_SOURCES; src++) {
        uint32_t enable_bit = (plic.contexts[ctx_id].enable[src / 32u] >> (src % 32u)) & 0x1u;
        if (!enable_bit)                            continue;
        if (!plic.sources[src].device_line)         continue;
        if ( plic.sources[src].claimed)             continue;
        if ( plic.sources[src].priority <= threshold) continue;

        if (plic.sources[src].priority > best_prio) {
            best_prio = plic.sources[src].priority;
            best_id   = src;
        }
        /* 同 priority 时 src 升序; 已 best 在前不覆盖 (因为我们 for src 升序遍历,
           "更高 prio" 才更新, 同 prio 不更新 → 保留较小 src) */
    }
    return best_id;
}

static int plic_ctx_has_pending_locked(uint32_t ctx_id) {
    uint32_t threshold = plic.contexts[ctx_id].threshold;
    for (uint32_t src = 1; src < PLIC_N_SOURCES; src++) {
        uint32_t enable_bit = (plic.contexts[ctx_id].enable[src / 32u] >> (src % 32u)) & 0x1u;
        if (!enable_bit)                            continue;
        if (!plic.sources[src].device_line)         continue;
        if ( plic.sources[src].claimed)             continue;
        if ( plic.sources[src].priority <= threshold) continue;
        return 1;
    }
    return 0;
}


// ----------------------------------------------------------------------------
// plic_ctx_eip 重算 helper (file-static; 调用方持 wrlock; "_locked")
// ----------------------------------------------------------------------------
//
// plic_recompute_ctx_eip_locked(ctx_id) — 仅重算单 ctx (claim/complete/threshold/
// enable 改时 ctx_id 已知场景调)
//
// plic_recompute_all_ctx_eip_locked()   — 所有 ctx 重算 (refresh thread 消费 event /
// priority 改 / plic_reset 后调; PLIC_N_CONTEXTS=2 全 scan 成本可忽略, 简单避免"算
// 哪些 ctx 受 source X 影响"的精确性维护)
//
// 写 plic_ctx_eip 用 atomic_store_explicit(release), 跟 is_plic_*_pending 的
// atomic_load_explicit(acquire) 配对建立 happens-before (producer/consumer 范式;
// dummy.txt §7 monitor 模型 memory_order 配对规则)。

static void plic_recompute_ctx_eip_locked(uint32_t ctx_id) {
    int v = plic_ctx_has_pending_locked(ctx_id);
    atomic_store_explicit(&plic_ctx_eip[ctx_id], v, memory_order_release);
}

static void plic_recompute_all_ctx_eip_locked(void) {
    for (uint32_t c = 0; c < PLIC_N_CONTEXTS; c++) {
        plic_recompute_ctx_eip_locked(c);
    }
}


// ----------------------------------------------------------------------------
// plic_enqueue_refresh — producer 入队 (file-static; 调用方 = device_set/clear_pending)
// ----------------------------------------------------------------------------
//
// 入队 (source_id, op) event; 满则 cond_timedwait not_full 100ms 心跳检 SDS (跟
// clint timer / uart reader thread cooperative shutdown 同体例; dummy.txt §12)。
//
// cond_timedwait 三种返回必须全处理 (POSIX 规范, 不挑):
//   - rc=0           consumer signal 或 spurious wakeup; while loop 重检 queue_full
//                    (Mesa-style cond var 必须重检防 spurious wakeup)
//   - rc=ETIMEDOUT   检 SDS, 触发 → 丢 event 直返 (system reset 路径下"丢"合理;
//                    reset 本身就清状态, 丢未消费 event 跟 reset 语义一致);
//                    SDS 未触发 → continue 再 wait 一轮 100ms
//   - rc 其他        真异常, log + 丢 event + 触发 SRS=0+SDS=0 走 cleanup 路径
//
// 时钟源选择 — CLOCK_REALTIME (pthread_cond 默认 attr 解读 abstime 时用的时钟):
//
// pthread_cond_timedwait 第 3 参数是个绝对时刻 (timespec), 默认按 CLOCK_REALTIME
// 解读。换 CLOCK_MONOTONIC 需多走一道: pthread_condattr_t attr +
// pthread_condattr_setclock(attr, CLOCK_MONOTONIC) + pthread_cond_init(cv, attr)
// + condattr_destroy; cond_init 不能传 NULL。多 5-8 行 init + fail path 多一层
// destroy chain。
//
// 跟 clint timer_run 用 CLOCK_MONOTONIC + clock_nanosleep ABSTIME 不对偶 — clint
// 有"严格累加 mtime 不漂 + perf 耗时不受 wall clock 跳变干扰"硬需求, refresh 心跳
// 没这两个需求 (作用纯 cooperative shutdown 检 SDS, 不要求精度)。
//
// wall clock 跳变命中分析: NTP 大跳基本在开机几分钟内一次, 夏令时一年两次, 手动
// date -s 几乎不在 emu 跑期出现; 项目当前 fixture 跑期 ~1s release / 几秒 debug,
// 命中概率 ~0。即便命中, jump backward → abstime 已过 → ETIMEDOUT → 重检 SDS,
// 等于"心跳跳一次"; jump forward → 这轮 wait 提前结束 → 同样 OK。**正确性不
// 受影响**, 最多跳一次心跳。
//
// 结论: v1 保持默认 CLOCK_REALTIME; 多 8 行代码换 style 一致 + 几乎不命中的"跳变
// 心跳跳一次"消除, ROI 不值。

static void plic_enqueue_refresh(uint32_t source_id, plic_refresh_op_t op) {
    pthread_mutex_lock(&plic_refresh_queue.lock);

    while (((plic_refresh_queue.head + 1u) % PLIC_REFRESH_QUEUE_CAP) ==
           plic_refresh_queue.tail) {
        /* queue full — 等 consumer 拉走 signal not_full */
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 100000000L;   /* 100 ms */
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec  += ts.tv_nsec / 1000000000L;
            ts.tv_nsec %= 1000000000L;
        }
        int rc = pthread_cond_timedwait(&plic_refresh_queue.not_full,
                                        &plic_refresh_queue.lock, &ts);
        if (rc == ETIMEDOUT) {
            if (atomic_load_explicit(&shutdown_signal, memory_order_acquire) == 0) {
                /* SDS triggered — drop event, return */
                pthread_mutex_unlock(&plic_refresh_queue.lock);
                return;
            }
            continue;   /* SDS still up, retry another 100ms wait */
        }
        if (rc != 0) {
            fprintf(stderr, "[plic refresh] enqueue cond_timedwait failed: %s\n",
                    strerror(rc));
            pthread_mutex_unlock(&plic_refresh_queue.lock);
            atomic_store_explicit(&system_reset_signal, 0, memory_order_release);
            atomic_store_explicit(&shutdown_signal,     0, memory_order_release);
            return;
        }
        /* rc == 0: signaled (consumer 拉走或 spurious wakeup); while 重检 queue_full */
    }

    plic_refresh_queue.buf[plic_refresh_queue.head].source_id = source_id;
    plic_refresh_queue.buf[plic_refresh_queue.head].op        = (uint8_t)op;
    plic_refresh_queue.head = (plic_refresh_queue.head + 1u) % PLIC_REFRESH_QUEUE_CAP;
    pthread_cond_signal(&plic_refresh_queue.not_empty);

    pthread_mutex_unlock(&plic_refresh_queue.lock);
}


// ----------------------------------------------------------------------------
// plic_pending_refresh_run — consumer 线程主 routine (file-static; 跟 clint timer_run
// 同 cooperative shutdown 体例)
// ----------------------------------------------------------------------------
//
// 跨 system reset 一直跑 (受 SDS 控制; SDS=0 自然退); plic_reset 系列由 main 串行
// 调时 refresh thread 也并发跑, 锁顺序协议 (模块顶部) 保 deadlock-free。
//
// 消费循环: mutex + cond_timedwait not_empty 100ms 心跳检 SDS → 拉 1 个 event → 释放
// mutex → wrlock 改 plic.sources[].device_line + 重算 plic_ctx_eip → 释放 wrlock
// (从不同时持两个锁)。
//
// 一次拉 1 个 event 简单 (而非批量拉空), 跟 producer 入队成本对偶; PLIC_N_CONTEXTS=2
// recompute_all 成本可忽略, 不引入"多 event 合并重算"优化。

static void *plic_pending_refresh_run(void *arg) {
    (void)arg;

    while (atomic_load_explicit(&shutdown_signal, memory_order_acquire)) {
        plic_refresh_event_t ev;
        int                  have_ev = 0;

        pthread_mutex_lock(&plic_refresh_queue.lock);

        /* 等 not_empty 或 timeout; while 重检防 spurious wakeup + SDS 退 */
        while (plic_refresh_queue.head == plic_refresh_queue.tail &&
               atomic_load_explicit(&shutdown_signal, memory_order_acquire)) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_nsec += 100000000L;   /* 100 ms */
            if (ts.tv_nsec >= 1000000000L) {
                ts.tv_sec  += ts.tv_nsec / 1000000000L;
                ts.tv_nsec %= 1000000000L;
            }
            int rc = pthread_cond_timedwait(&plic_refresh_queue.not_empty,
                                            &plic_refresh_queue.lock, &ts);
            if (rc == ETIMEDOUT) continue;   /* 重检 SDS + queue */
            if (rc != 0) {
                fprintf(stderr, "[plic refresh] consume cond_timedwait failed: %s\n",
                        strerror(rc));
                pthread_mutex_unlock(&plic_refresh_queue.lock);
                atomic_store_explicit(&system_reset_signal, 0, memory_order_release);
                atomic_store_explicit(&shutdown_signal,     0, memory_order_release);
                return NULL;
            }
            /* rc == 0: signaled or spurious; while 重检 */
        }

        if (plic_refresh_queue.head != plic_refresh_queue.tail) {
            ev = plic_refresh_queue.buf[plic_refresh_queue.tail];
            plic_refresh_queue.tail = (plic_refresh_queue.tail + 1u) % PLIC_REFRESH_QUEUE_CAP;
            have_ev = 1;
            pthread_cond_signal(&plic_refresh_queue.not_full);
        }

        pthread_mutex_unlock(&plic_refresh_queue.lock);

        if (!have_ev) continue;   /* SDS=0 退路: 上面 while 退出后 queue 仍空 */

        /* 应用 event: 持 wrlock 改 sources[].device_line + 重算所有 ctx eip */
        plic_wrlock();
        if (ev.source_id > 0u && ev.source_id < PLIC_N_SOURCES) {
            plic.sources[ev.source_id].device_line =
                (ev.op == (uint8_t)PLIC_REFRESH_OP_SET) ? 1 : 0;
            plic_recompute_all_ctx_eip_locked();
        }
        plic_unlock();
    }

    return NULL;
}


// ----------------------------------------------------------------------------
// plic_read / plic_write — bus 派发入口
// ----------------------------------------------------------------------------
//
// 接口语义 (dummy.txt §9):
//   返 0    = 成功; read 路径返字段值 / write 路径修改字段
//   返非 0  = cause (RV spec exception code); bus 透传给 trap_raise_exception
//
// fault 政策 (跟 RV PLIC spec / QEMU sifive_plic 一致):
//   - size != 4 / off 未对齐 → CAUSE_*_ACCESS_FAULT (硬件级硬限制, PLIC 寄存器都
//     32-bit 对齐, 1B/2B/8B 访问 + 非 4 对齐 不是合法访问形态)
//   - PLIC region 内越界 / reserved → silent ignore (read 返 0 / write 丢弃, 不 fault)
//   - PLIC region 外 (off >= PLIC_SIZE) → bus 已 range check 拦, 内部 final else
//     保留 fault 是防御性
//
// 锁策略: 按字段访问行为分:
//   - rdlock: priority / threshold / enable / pending 读
//   - wrlock: priority / threshold / enable / complete 写; claim 读 (改 claimed);
//             这些 wrlock 路径释放锁前都顺手调 plic_recompute_ctx_eip_locked
//             保持 plic_ctx_eip 反映最新 effective view
//   - 不 lock: pending 写 (spec RO, silent ignore 不动字段); reserved (silent 返 0);
//              size/对齐 fault (前置 check, 不进 lock)
//
// claim 路径 wrlock 内顺手重算 plic_ctx_eip[ctx_id] (set claimed 后, 该 ctx 可送
// source 集合变, has_pending 可能变 0); complete 路径同理 (清 claimed 后, source
// 重新可选, has_pending 可能变 1)。priority 改影响所有 ctx, recompute_all。

static int plic_read(void *ctx, uint32_t off, void *buf, uint32_t size) {
    (void)ctx;
    if (size != 4u)              return CAUSE_LOAD_ACCESS_FAULT;
    if ((off & 0x3u) != 0u)      return CAUSE_LOAD_ACCESS_FAULT;

    uint32_t value = 0;

    if (off < (uint32_t)PLIC_PRIORITY_OFF + 0x1000u) {
        /* priority 区: [0, 0x1000); per-source 4B */
        uint32_t src = off / 4u;
        if (src < PLIC_N_SOURCES) {
            plic_rdlock();
            value = plic.sources[src].priority;
            plic_unlock();
        }

    } else if (off < (uint32_t)PLIC_ENABLE_OFF) {
        /* pending 区: [0x1000, 0x2000); 1 bit/source, 32 src/word */
        uint32_t word_idx  = (off - (uint32_t)PLIC_PENDING_OFF) / 4u;
        uint32_t src_base  = word_idx * 32u;
        if (src_base < PLIC_N_SOURCES) {
            plic_rdlock();
            for (uint32_t i = 0; i < 32u && (src_base + i) < PLIC_N_SOURCES; i++) {
                uint32_t src = src_base + i;
                if (src == 0u) continue;          /* source 0 保留 */
                if (plic.sources[src].device_line && !plic.sources[src].claimed) {
                    value |= (1u << i);
                }
            }
            plic_unlock();
        }

    } else if (off < (uint32_t)PLIC_CONTEXT_OFF) {
        /* enable 区: [0x2000, 0x200000); per-ctx, 0x80 B/ctx, 1 bit/source */
        uint32_t bo        = off - (uint32_t)PLIC_ENABLE_OFF;
        uint32_t ctx_id    = bo / (uint32_t)PLIC_ENABLE_STRIDE;
        uint32_t word_idx  = (bo % (uint32_t)PLIC_ENABLE_STRIDE) / 4u;
        if (ctx_id < PLIC_N_CONTEXTS &&
            word_idx < (PLIC_N_SOURCES + 31u) / 32u) {
            plic_rdlock();
            value = plic.contexts[ctx_id].enable[word_idx];
            plic_unlock();
        }

    } else if (off < (uint32_t)PLIC_SIZE) {
        /* context 区: [0x200000, 0x600000); per-ctx, 4 KB/ctx; +0 threshold, +4 claim */
        uint32_t bo         = off - (uint32_t)PLIC_CONTEXT_OFF;
        uint32_t ctx_id     = bo / (uint32_t)PLIC_CONTEXT_STRIDE;
        uint32_t off_in_ctx = bo % (uint32_t)PLIC_CONTEXT_STRIDE;
        if (ctx_id < PLIC_N_CONTEXTS) {
            if (off_in_ctx == 0u) {
                /* threshold */
                plic_rdlock();
                value = plic.contexts[ctx_id].threshold;
                plic_unlock();
            } else if (off_in_ctx == 4u) {
                /* claim: 仲裁 + set claimed; wrlock (read 但改字段) */
                plic_wrlock();
                uint32_t id = plic_arbitrate_locked(ctx_id);
                if (id != 0u) {
                    plic.sources[id].claimed = ctx_id + 1u;
                }
                value = id;
                plic_recompute_ctx_eip_locked(ctx_id);   /* claimed 改 → ctx eip 可能变 0 */
                plic_unlock();
            }
            /* reserved (off_in_ctx 非 0/4): silent 返 0 */
        }

    } else {
        /* off >= PLIC_SIZE: 实际 bus range check 已拦, 这里防御性 fault. */
        return CAUSE_LOAD_ACCESS_FAULT;
    }

    memcpy(buf, &value, 4);
    return 0;
}

static int plic_write(void *ctx, uint32_t off, const void *buf, uint32_t size) {
    (void)ctx;
    if (size != 4u)              return CAUSE_STORE_ACCESS_FAULT;
    if ((off & 0x3u) != 0u)      return CAUSE_STORE_ACCESS_FAULT;

    uint32_t value;
    memcpy(&value, buf, 4);

    if (off < (uint32_t)PLIC_PRIORITY_OFF + 0x1000u) {
        uint32_t src = off / 4u;
        if (src < PLIC_N_SOURCES && src != 0u) {
            plic_wrlock();
            plic.sources[src].priority = value;
            plic_recompute_all_ctx_eip_locked();   /* priority 影响所有 ctx 仲裁 */
            plic_unlock();
        }

    } else if (off < (uint32_t)PLIC_ENABLE_OFF) {
        /* pending 区: spec RO, 软件写 silent ignore (不 lock, 不动字段) */

    } else if (off < (uint32_t)PLIC_CONTEXT_OFF) {
        uint32_t bo        = off - (uint32_t)PLIC_ENABLE_OFF;
        uint32_t ctx_id    = bo / (uint32_t)PLIC_ENABLE_STRIDE;
        uint32_t word_idx  = (bo % (uint32_t)PLIC_ENABLE_STRIDE) / 4u;
        if (ctx_id < PLIC_N_CONTEXTS &&
            word_idx < (PLIC_N_SOURCES + 31u) / 32u) {
            plic_wrlock();
            plic.contexts[ctx_id].enable[word_idx] = value;
            /* word 0 bit 0 = source 0 (保留), 软件写也由仲裁 src 从 1 起遍历过滤掉 */
            plic_recompute_ctx_eip_locked(ctx_id); /* enable 改 → 该 ctx 可送集合变 */
            plic_unlock();
        }

    } else if (off < (uint32_t)PLIC_SIZE) {
        uint32_t bo         = off - (uint32_t)PLIC_CONTEXT_OFF;
        uint32_t ctx_id     = bo / (uint32_t)PLIC_CONTEXT_STRIDE;
        uint32_t off_in_ctx = bo % (uint32_t)PLIC_CONTEXT_STRIDE;
        if (ctx_id < PLIC_N_CONTEXTS) {
            if (off_in_ctx == 0u) {
                /* threshold */
                plic_wrlock();
                plic.contexts[ctx_id].threshold = value;
                plic_recompute_ctx_eip_locked(ctx_id); /* threshold 改 → 该 ctx 仲裁结果可能变 */
                plic_unlock();
            } else if (off_in_ctx == 4u) {
                /* complete: 校验 sources[value].claimed == ctx+1 后清; 否则 silent
                   (RV PLIC spec §8 "If complete value does not match active claim,
                    write is ignored"). */
                if (value > 0u && value < PLIC_N_SOURCES) {
                    plic_wrlock();
                    if (plic.sources[value].claimed == ctx_id + 1u) {
                        plic.sources[value].claimed = 0;
                        plic_recompute_ctx_eip_locked(ctx_id); /* 清 claimed → 该 ctx 可能重新有 pending */
                    }
                    plic_unlock();
                }
            }
            /* reserved (off_in_ctx 非 0/4): silent ignore */
        }

    } else {
        return CAUSE_STORE_ACCESS_FAULT;
    }
    return 0;
}


// ----------------------------------------------------------------------------
// 外部接口: is_plic_*_pending (hart 侧 csr_mip_read 合成; T6.2 改 atomic_load 直返)
// ----------------------------------------------------------------------------
//
// T6.2 前: rdlock + plic_ctx_has_pending_locked scan (热路径走锁 + scan, 中断检查
// 实测 ~10x 拖累 03_csr_heavy 等 fixture)。
// T6.2 后: atomic_load_explicit(plic_ctx_eip[ctx_id], acquire) 直返, 零 lock + 零
// scan; producer 在 refresh thread / wrlock 路径内异步更新, 主帧不感知。
//
// memory_order_acquire 跟 producer (refresh thread + wrlock 路径) 的
// atomic_store_explicit(release) 配对, 建立 happens-before (consumer 看到 producer
// 的最新更新; dummy.txt §7 monitor 模型 memory_order 配对规则)。
//
// 函数签名透明 (csr_mip_read / trap_check_interrupt 调用点零侵入); hartid 越界
// 防御 + plic_ctx_map[index] < 0 防御 (没连线 hart×priv) 仍保留, 走 atomic_load
// 之前提前 return 0。

int is_plic_meip_pending(uint32_t hartid) {
    if (hartid >= MAX_HARTS) return 0;
    int8_t ctx = plic.plic_ctx_map[(hartid << 2) | PRIV_M];
    if (ctx < 0) return 0;
    return atomic_load_explicit(&plic_ctx_eip[ctx], memory_order_acquire) ? 1 : 0;
}

int is_plic_seip_pending(uint32_t hartid) {
    if (hartid >= MAX_HARTS) return 0;
    int8_t ctx = plic.plic_ctx_map[(hartid << 2) | PRIV_S];
    if (ctx < 0) return 0;
    return atomic_load_explicit(&plic_ctx_eip[ctx], memory_order_acquire) ? 1 : 0;
}


// ----------------------------------------------------------------------------
// 外部接口: device_set/clear_pending (外设侧; T6.2 改异步入队)
// ----------------------------------------------------------------------------
//
// 外设内部状态机自己决定何时拉高/拉低 device_line, 通过这俩接口通知 PLIC (UART /
// virtio-blk 等)。source_id 0 / 越界 silent ignore (设备树没分配的 source 走这里
// 不该出问题 — 防御性)。
//
// T6.2 前: 直接 wrlock + 改 sources[id].device_line (同步路径)。
// T6.2 后: 入队 PLIC_REFRESH_OP_SET/CLEAR event, plic_pending_refresh_thread 异步
// 消费 + wrlock + 改 sources[].device_line + 重算 plic_ctx_eip。接口签名不变,
// 调用方 (UART reader thread / test_dev MMIO 写 / 未来 virtio-blk) 不需改。
//
// 异步可见性 trade: guest MMIO 读 PLIC pending bitmap 可能 lag (refresh thread 还
// 没消费 event); RV spec 不要求"立即可见", guest 通常看 MEIP/SEIP + claim, 而
// MEIP/SEIP 由 plic_ctx_eip 反映 (refresh thread 消费完才置位 → guest 接到中断时
// pending bitmap 必同步可见, 不存在 "MEIP 已置位但 bitmap 仍为 0" 不一致)。
//
// fixture 通过 test_dev MMIO 写 (sw TEST_DEV_SET_OFF / CLEAR_OFF) 触发; 详
// src/device/test_dev.{h,c}。

void device_set_pending(uint32_t source_id) {
    if (source_id == 0u || source_id >= PLIC_N_SOURCES) return;
    plic_enqueue_refresh(source_id, PLIC_REFRESH_OP_SET);
}

void device_clear_pending(uint32_t source_id) {
    if (source_id == 0u || source_id >= PLIC_N_SOURCES) return;
    plic_enqueue_refresh(source_id, PLIC_REFRESH_OP_CLEAR);
}


// ----------------------------------------------------------------------------
// lifecycle: plic_init / plic_reset / plic_destroy
// ----------------------------------------------------------------------------

int plic_init(void) {
    /* 字段 0 init — BSS 已经 0, 显式 memset 仅 lifecycle 可读 (跟 clint_init 同体例)。 */
    memset(&plic, 0, sizeof(plic));

    /* plic_ctx_map 全 -1 init, 然后按 v1 假定全 MSU 覆盖 M/S 槽 (U/VS 保持 -1)。
       ctx_id 顺序 M 小 S 大 (跟 QEMU virt sifive_plic 一致): hart h → ctx (2h, 2h+1)
       = (M, S)。未来 dtb 接入后这段换成解析后调 plic_set_ctx_map 接口。 */
    for (uint32_t i = 0; i < MAX_HARTS * 4u; i++) {
        plic.plic_ctx_map[i] = -1;
    }
    for (uint32_t h = 0; h < MAX_HARTS; h++) {
        plic.plic_ctx_map[(h << 2) | PRIV_M] = (int8_t)(2u * h);
        plic.plic_ctx_map[(h << 2) | PRIV_S] = (int8_t)(2u * h + 1u);
    }

    /* plic_ctx_eip 全 0 init (BSS 已 0, 显式 atomic_store lifecycle 可读 +
       memory_order_release 确保后续线程看到 0 起点)。 */
    for (uint32_t c = 0; c < PLIC_N_CONTEXTS; c++) {
        atomic_store_explicit(&plic_ctx_eip[c], 0, memory_order_release);
    }

    /* plic_refresh_queue init: 字段 0 + mutex + 双 cond. refresh_thread 句柄不在此处
       写 — plic_start_pending_refresh_thread 才 pthread_create (dummy.txt §12)。 */
    plic_refresh_queue.head = 0;
    plic_refresh_queue.tail = 0;
    memset(plic_refresh_queue.buf, 0, sizeof(plic_refresh_queue.buf));

    int rc = pthread_mutex_init(&plic_refresh_queue.lock, NULL);
    if (rc != 0) {
        fprintf(stderr, "plic_init: queue mutex_init failed: %s\n", strerror(rc));
        return -1;
    }
    rc = pthread_cond_init(&plic_refresh_queue.not_full, NULL);
    if (rc != 0) {
        fprintf(stderr, "plic_init: queue cond_init(not_full) failed: %s\n", strerror(rc));
        (void)pthread_mutex_destroy(&plic_refresh_queue.lock);
        return -1;
    }
    rc = pthread_cond_init(&plic_refresh_queue.not_empty, NULL);
    if (rc != 0) {
        fprintf(stderr, "plic_init: queue cond_init(not_empty) failed: %s\n", strerror(rc));
        (void)pthread_cond_destroy (&plic_refresh_queue.not_full);
        (void)pthread_mutex_destroy(&plic_refresh_queue.lock);
        return -1;
    }

    /* pthread_rwlock_init: 默认 attr; 失败按 dummy.txt §5 fprintf + 返 -1。 */
    rc = pthread_rwlock_init(&plic.lock, NULL);
    if (rc != 0) {
        fprintf(stderr, "plic_init: pthread_rwlock_init failed: rc=%d\n", rc);
        (void)pthread_cond_destroy (&plic_refresh_queue.not_empty);
        (void)pthread_cond_destroy (&plic_refresh_queue.not_full);
        (void)pthread_mutex_destroy(&plic_refresh_queue.lock);
        return -1;
    }

    mmio_dev_t dev = {
        .gpa_start = (uint32_t)PLIC_BASE,
        .gpa_end   = (uint32_t)(PLIC_BASE + PLIC_SIZE),
        .ctx       = &plic,           /* 单例 device, ctx 接口风格统一占位 (fn 内 (void)ctx;) */
        .read      = plic_read,
        .write     = plic_write,
        .name      = "plic",
    };
    if (bus_register_mmio(&dev) != 0) {
        fprintf(stderr, "plic_init: bus_register_mmio failed\n");
        (void)pthread_rwlock_destroy(&plic.lock);
        (void)pthread_cond_destroy  (&plic_refresh_queue.not_empty);
        (void)pthread_cond_destroy  (&plic_refresh_queue.not_full);
        (void)pthread_mutex_destroy (&plic_refresh_queue.lock);
        return -1;
    }
    return 0;
}

int plic_reset(void) {
    /* system reset 每 iter: sources / contexts 字段清 + queue 清 + plic_ctx_eip 清;
       plic_ctx_map 不动 (硬件 wired 状态不掉电不停, 跟 mtime 不动同性质); 锁不动
       (基础设施)。
       refresh thread 跨 reset 一直跑 (跟 clint timer / uart reader 同形态), 此函数
       内部跟 refresh thread 真并发 → 持锁清空 (锁顺序协议: 模块顶部)。 */

    /* 1) mutex 清 queue head/tail + broadcast cond 让 producer/consumer 醒来重检 */
    pthread_mutex_lock(&plic_refresh_queue.lock);
    plic_refresh_queue.head = 0;
    plic_refresh_queue.tail = 0;
    pthread_cond_broadcast(&plic_refresh_queue.not_full);
    pthread_cond_broadcast(&plic_refresh_queue.not_empty);
    pthread_mutex_unlock(&plic_refresh_queue.lock);

    /* 2) wrlock 清 sources/contexts + plic_ctx_eip */
    plic_wrlock();
    for (uint32_t i = 0; i < PLIC_N_SOURCES; i++) {
        plic.sources[i].device_line = 0;
        plic.sources[i].claimed     = 0;
        plic.sources[i].priority    = 0;
    }
    for (uint32_t c = 0; c < PLIC_N_CONTEXTS; c++) {
        plic.contexts[c].threshold = 0;
        for (uint32_t w = 0; w < (PLIC_N_SOURCES + 31u) / 32u; w++) {
            plic.contexts[c].enable[w] = 0;
        }
        atomic_store_explicit(&plic_ctx_eip[c], 0, memory_order_release);
    }
    plic_unlock();

    return 0;
}

void plic_destroy(void) {
    /* 纯模块 cleanup — 跟 clint_destroy / cpu_destroy 同 lifecycle 对称; 不含
       pthread_join (refresh thread 由 main 调 plic_join_pending_refresh_thread 显式
       回收; dummy.txt §12 谁 spawn 谁 join)。
       pthread_*_destroy 释放内部资源 (glibc 下基本零开销; 但 valgrind/ASan 在意
       内部 leak, 显式 destroy 干净)。 */
    (void)pthread_rwlock_destroy(&plic.lock);
    (void)pthread_cond_destroy  (&plic_refresh_queue.not_empty);
    (void)pthread_cond_destroy  (&plic_refresh_queue.not_full);
    (void)pthread_mutex_destroy (&plic_refresh_queue.lock);
}


// ----------------------------------------------------------------------------
// thread lifecycle: plic_start_pending_refresh_thread / plic_join_pending_refresh_thread
// (dummy.txt §12; 跟 clint_start/join_timer_thread + uart_start/join_reader_thread 同体例)
// ----------------------------------------------------------------------------

void plic_start_pending_refresh_thread(void) {
    /* 前置 check: 若 SDS 已 0 (前面有别的 init 失败, 通过 runtime signal 通道
       提前通知"别再 spawn"), 跳过 pthread_create。跟 clint_start_timer_thread
       同形态死代码 (当前 main flow 下 spawn 时 SDS 必 1)。 */
    if (atomic_load_explicit(&shutdown_signal, memory_order_acquire) == 0) {
        return;
    }

    int rc = pthread_create(&plic_refresh_queue.refresh_thread, NULL,
                            plic_pending_refresh_run, NULL);
    if (rc != 0) {
        fprintf(stderr, "plic_start_pending_refresh_thread: pthread_create failed: %s\n",
                strerror(rc));
        /* 错误走 runtime signal 通道 — 一次 set 两 flag (SDS 蕴含 SRS 触发关系契约;
           runtime.h doc + dummy.txt §12)。 */
        atomic_store_explicit(&system_reset_signal, 0, memory_order_release);
        atomic_store_explicit(&shutdown_signal,     0, memory_order_release);
        /* plic_refresh_queue.refresh_thread 保持 BSS 0 init (POSIX 7.2 pthread_create
           fail 不修改 thread 参数); plic_join_pending_refresh_thread 内 pthread_join(0)
           glibc 下返 ESRCH 容错 — 不需要额外 track "是否 spawn 成功"。 */
    }
}

void plic_join_pending_refresh_thread(void) {
    /* 调用前置: main 已 atomic_store(&shutdown_signal, 0); 否则 pthread_join
       永远 block (refresh thread 在 while(SDS) 内永不退)。
       spawn fail case: plic_refresh_queue.refresh_thread = BSS 0, pthread_join 在
       glibc/musl 下返 ESRCH 一行 fprintf 不 fatal (跟 clint_join_timer_thread /
       uart_join_reader_thread 同形态)。 */
    int rc = pthread_join(plic_refresh_queue.refresh_thread, NULL);
    if (rc != 0) {
        fprintf(stderr, "plic_join_pending_refresh_thread: pthread_join failed: %s\n",
                strerror(rc));
    }
}
