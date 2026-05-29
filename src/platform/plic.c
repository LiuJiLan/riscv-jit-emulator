//
// Created by liujilan on 2026/5/23.
// PLIC 实现 — per-source <device_line, claimed, priority> + per-ctx <threshold,
// enable bitmap> + plic_ctx_map + bus 注册 + 完整 claim/complete 仲裁 + hart 侧
// is_plic_*_pending 合成 view + 外设侧 device_set/clear_pending 接口。
//
// 设计 trail (异步刷新方案曾试过, 因 handler 同步 ACK + 异步 CLEAR race 引起
// spurious re-fire 回退同步; hot path 优化的本质 = atomic 字段, 跟异步 queue
// 无关): 详 trade_off_log §T.6.
//
// 设计形态:
//   - 主路径 (hart 主帧 csr_mip_read → is_plic_*_pending) 走 atomic_load
//     plic_ctx_eip[ctx], 零 lock 零 scan. (hot path 优化)
//   - 慢路径 (device_set/clear_pending / priority/threshold/enable 写 / claim /
//     complete) 走 wrlock + 改字段 + recompute_ctx_eip + atomic_store(eip)
//     + recompute_pending_bitmap + unlock. 同步, 跟真硬件 wire 模型对齐.
//   - guest MMIO 读 pending bitmap 区走 atomic_load plic_pending_bitmap_cache,
//     零 lock; cache 由 wrlock 路径同步维护.
//
// 接口形态 + monitor 模型 + rwlock 抽象 + 字段对应 spec + 命名选择见 plic.h 顶段 doc。
// 地址布局见 config.h PLIC_* 宏 (跟 QEMU virt + RV PLIC spec v1.0.0 兼容)。
// 报错风格见 dummy.txt §5; "0=成功 / 非0=cause" 接口约定见 dummy.txt §9。
//

#include "plic.h"

#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config.h"          // PLIC_* / MAX_HARTS
#include "core/wfi.h"        // wfi_kick (ctx_eip 0→1 翻转唤醒 WFI hart)
#include "platform/bus.h"    // mmio_dev_t / bus_register_mmio
#include "riscv.h"           // CAUSE_LOAD/STORE_ACCESS_FAULT / PRIV_M / PRIV_S


// ----------------------------------------------------------------------------
// 锁协议 (避免 deadlock)
// ----------------------------------------------------------------------------
//
// 模块内只一把锁: plic.lock (rwlock). 所有路径都遵循 "进锁 → 改 +
// recompute_ctx_eip atomic_store + recompute_pending_bitmap atomic_store →
// 出锁" 模式. 跨线程 (worker / reader thread → device_set/clear_pending)
// 也走同一把 wrlock 短临界区.
//
//   hart is_plic_*_pending:           不持锁; atomic_load(plic_ctx_eip[ctx_id])
//   hart claim/complete:              wrlock (read 但改 claimed 字段)
//   hart MMIO pending bitmap 读:      不持锁; atomic_load(plic_pending_bitmap_cache)
//   hart MMIO priority/threshold/enable 读: rdlock
//   hart MMIO priority/threshold/enable 写: wrlock
//   设备 device_set/clear_pending:    wrlock (跨线程同步)
//   plic_reset:                       wrlock
//
// 单锁形态, 无 deadlock 风险.


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
//   device_line — 设备拉线状态 (level 模型镜像); device_set/clear_pending 持
//                 wrlock 直接写 + recompute_ctx_eip atomic_store + recompute_
//                 pending_bitmap atomic_store
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
    int8_t            plic_ctx_to_hartid[PLIC_N_CONTEXTS]; /* 反查: ctx_id → hartid; -1 = 未绑定;
                                                              015 加, 服务 wfi_kick on eip 0→1 */
    pthread_rwlock_t  lock;
} plic;


// ----------------------------------------------------------------------------
// plic_ctx_eip — per-ctx external interrupt pending atomic 子集
// ----------------------------------------------------------------------------
//
// 独立 file-static, NOT 进 plic struct; 跟 CLINT 全 atomic monitor 体例对偶
// (CLINT 字段直 _Atomic file-static, 没 rwlock); PLIC 的"atomic 子集"提取出来
// 跟 CLINT 一个味道 — atomic 自带 ordering, 不持 lock。
//
// 语义: per-ctx visible-pending boolean. consumer = is_plic_meip/seip_pending
// (dispatcher 主帧 csr_mip_read 调) 直接 atomic_load_explicit(acquire), 零 lock,
// 是 hot path 优化的核心 (跟 refresh queue 是否异步无关 — atomic 字段才是
// hot path 优化的本质).
//
// producer 路径 (所有写都持 plic.lock wrlock, atomic_store_explicit(release)):
//   - device_set/clear_pending 改 sources[].device_line 后重算
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
// plic_pending_bitmap_cache — MMIO pending bitmap 区 atomic cache
// ----------------------------------------------------------------------------
//
// 独立 file-static, NOT 进 plic struct; 跟 plic_ctx_eip 同性质 (PLIC "atomic 子集"
// — atomic 自带 ordering, 不持 lock)。guest 读 PLIC pending bitmap (off [0x1000,
// 0x2000)) 从原 rdlock + scan 96 source 降到 atomic_load 32-bit 一次。
//
// 语义: 1 bit/source, 32 src/word, 总 (PLIC_N_SOURCES+31)/32 = 3 word
// (PLIC_N_SOURCES=96). 每 word 缓存 32 source 的 (device_line && !claimed) view.
// source 0 (word 0 bit 0) 永远 0 (保留)。
//
// producer 路径 (所有写都持 plic.lock wrlock; atomic_store_explicit release):
//   - device_set/clear_pending 改 sources[].device_line 后重算
//   - plic_read 命中 claim 副作用 (set claimed 后)
//   - plic_write 命中 complete (清 claimed 后)
//   - plic_reset 清 0
//   - plic_init 清 0
//
// **不在 priority/threshold/enable 写路径调** — 这三个不影响 pending bitmap (pending
// = device_line && !claimed, 不含 priority/threshold/enable). 区分点跟 plic_ctx_eip
// 调用点矩阵不同 (后者 priority/threshold/enable 都影响 — 因为仲裁结果变).
//
// consumer 路径 (plic_read pending 区 lw): atomic_load_explicit(acquire) 直返;
// 不持锁不 scan, 跟 is_plic_*_pending 同 hot path 形态.
//
// 设计要点: hot path 优化的本质是 atomic 字段 (atomic_load 零 lock 直返), 跟
// device_line 是否同步 / 异步无关. wrlock 路径内同步维护 atomic cache, 一次
// wrlock 临界区"改 device_line + recompute_eip atomic_store + recompute_bitmap
// atomic_store + 释放" 原子完成, 跟 ctx_eip 一致性等级相同. 真硬件 wire 模型
// 同步, 我们 wrlock 短临界区 (~几百 ns) 量级一致.
static _Atomic uint32_t plic_pending_bitmap_cache[(PLIC_N_SOURCES + 31u) / 32u];


// ----------------------------------------------------------------------------
// monitor RW helper (file-static)
// ----------------------------------------------------------------------------
//
// 包装 pthread_rwlock_t 的 rdlock / wrlock / unlock; 不检 rc (POSIX rwlock 在 init
// 成功后, lock/unlock 只在 EDEADLK/EINVAL 等编程错误下失败, v1 不处理 — 真撞 ASan
// + sanitizer 抓住)。SMP 单 hart 下 contention=0, glibc 走 fast path 零开销。
//
// 调用方分布: rdlock 仅 plic_read 内 priority/threshold/enable 读 (pending 区
// 走 atomic_load_bitmap_cache, is_plic_*_pending 走 atomic_load_ctx_eip, 都不调
// rdlock); wrlock 调用方多 — 各 MMIO 写 + device_set/clear_pending.

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
// plic_ctx_has_pending_locked — plic_recompute_ctx_eip_locked 内调; 只判
// "有任一可送 source", 不算具体哪个 (短路返 1)。

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
// plic_recompute_all_ctx_eip_locked()   — 所有 ctx 重算 (device_set/clear_pending
// 改 device_line / priority 改 / plic_reset 后调; PLIC_N_CONTEXTS=2 全 scan 成本
// 可忽略, 简单避免"算哪些 ctx 受 source X 影响"的精确性维护)
//
// 写 plic_ctx_eip 用 atomic_store_explicit(release), 跟 is_plic_*_pending 的
// atomic_load_explicit(acquire) 配对建立 happens-before (producer/consumer 范式;
// dummy.txt §7 monitor 模型 memory_order 配对规则)。

static void plic_recompute_ctx_eip_locked(uint32_t ctx_id) {
    int v_new = plic_ctx_has_pending_locked(ctx_id);
    int v_old = atomic_load_explicit(&plic_ctx_eip[ctx_id], memory_order_relaxed);
    atomic_store_explicit(&plic_ctx_eip[ctx_id], v_new, memory_order_release);
    // 015 加: ctx eip 0→1 翻转 → 唤醒该 ctx 对应 hart (可能在 WFI cond_wait)。
    //
    // ctx → hartid 走 plic_ctx_to_hartid[] 反查表 (plic_init 跟 plic_ctx_map 同处填);
    // 未绑定 (-1) 跳过 (合法状态, 例如 U/VS 槽当前都 -1)。
    //
    // ctx_id 顺序: plic_init 体例 ctx 2h = hart h M-mode, ctx 2h+1 = hart h S-mode
    // (即 ctx_id 越小 priv 越高); plic_recompute_all_ctx_eip_locked 按 ctx_id 升序
    // 遍历 → 同一 hart 的 M ctx 必先于 S ctx 被 recompute, 即 M kick 必先于 S kick。
    // 这是安全的:
    //   (a) wfi_kick 幂等, 多次 kick 同 hart 只多醒一次, hart predicate re-check 决定睡或继续
    //   (b) MEIP > SEIP 优先级跟 kick 顺序一致, hart 醒来后 mip 两位都 set 时
    //       trap_check_interrupt 也是先选 MEIP, 跟 kick 顺序自然对齐
    if (v_old == 0 && v_new != 0) {
        int8_t h = plic.plic_ctx_to_hartid[ctx_id];
        if (h >= 0) {
            wfi_kick((uint32_t)h);
        }
    }
}

static void plic_recompute_all_ctx_eip_locked(void) {
    for (uint32_t c = 0; c < PLIC_N_CONTEXTS; c++) {
        plic_recompute_ctx_eip_locked(c);
    }
}


// ----------------------------------------------------------------------------
// plic_pending_bitmap_cache 重算 helper (file-static; 调用方持 wrlock; "_locked")
// ----------------------------------------------------------------------------
//
// 重算所有 word (3 word for PLIC_N_SOURCES=96), 跟 plic_recompute_all_ctx_eip_locked
// 一样全 scan — 单调用成本 ~96 source 比较 + 3 atomic_store, 可忽略。简化避免"算
// 哪些 word 受 source X 影响"的精确性维护。
//
// 写 plic_pending_bitmap_cache 用 atomic_store_explicit(release), 跟 plic_read
// pending 区的 atomic_load_explicit(acquire) 配对建立 happens-before。

static void plic_recompute_pending_bitmap_locked(void) {
    for (uint32_t w = 0; w < (PLIC_N_SOURCES + 31u) / 32u; w++) {
        uint32_t v = 0;
        for (uint32_t i = 0; i < 32u && (w * 32u + i) < PLIC_N_SOURCES; i++) {
            uint32_t src = w * 32u + i;
            if (src == 0u) continue;          /* source 0 保留 */
            if (plic.sources[src].device_line && !plic.sources[src].claimed) {
                v |= (1u << i);
            }
        }
        atomic_store_explicit(&plic_pending_bitmap_cache[w], v, memory_order_release);
    }
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
        /* pending 区: [0x1000, 0x2000); 1 bit/source, 32 src/word.
           atomic_load(plic_pending_bitmap_cache[word_idx], acquire) 直返, 零 lock
           零 scan. cache 由 device_line/claimed 改路径 wrlock 内
           recompute_pending_bitmap_locked 同步刷新. */
        uint32_t word_idx = (off - (uint32_t)PLIC_PENDING_OFF) / 4u;
        if (word_idx < (PLIC_N_SOURCES + 31u) / 32u) {
            value = atomic_load_explicit(&plic_pending_bitmap_cache[word_idx],
                                         memory_order_acquire);
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
                plic_recompute_pending_bitmap_locked();  /* claimed 改 → pending bitmap 也变 */
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
                        plic_recompute_pending_bitmap_locked(); /* claimed 改 → pending bitmap 也变 */
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
// 外部接口: is_plic_*_pending (hart 侧 csr_mip_read 合成)
// ----------------------------------------------------------------------------
//
// atomic_load_explicit(plic_ctx_eip[ctx_id], acquire) 直返, 零 lock + 零 scan.
// producer 在 wrlock 路径 (device_set/clear_pending / MMIO 写 / claim / complete)
// 内同步更新 plic_ctx_eip atomic_store(release), 主帧不感知锁.
//
// hot path 优化的本质 = atomic 字段 (跟此前曾试的异步 refresh queue 无关; 演进
// trail 详 trade_off_log §T.6).
//
// memory_order_acquire 跟 producer 的 atomic_store_explicit(release) 配对, 建立
// happens-before (consumer 看到 producer 的最新更新; dummy.txt §7 monitor 模型
// memory_order 配对规则).
//
// 函数签名透明 (csr_mip_read / trap_check_interrupt 调用点零侵入); hartid 越界
// 防御 + plic_ctx_map[index] < 0 防御 (没连线 hart×priv) 仍保留, 走 atomic_load
// 之前提前 return 0.

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
// 外部接口: device_set/clear_pending (外设侧; 同步 wrlock)
// ----------------------------------------------------------------------------
//
// 外设内部状态机自己决定何时拉高/拉低 device_line, 通过这俩接口通知 PLIC (UART /
// virtio-blk 等)。source_id 0 / 越界 silent ignore (设备树没分配的 source 走这里
// 不该出问题 — 防御性)。
//
// 形态: wrlock + 改 sources[id].device_line + 重算 plic_ctx_eip atomic_store +
// 重算 plic_pending_bitmap_cache atomic_store + unlock. 调用方同帧完成, 不阻塞
// 主路径 (hot path 仍是 atomic_load).
//
// 跨线程调用 (worker / reader thread → device_set_pending) 同步吃 wrlock,
// 短临界区 (~几百 ns recompute 96 source); 跟真硬件 wire 信号传播 (几 cycle
// ns 级) 量级一致, 跟真硬件 PLIC 模型对齐.
//
// 同步形态 vs 曾试过的异步 refresh queue 的取舍 — 见 trade_off_log §T.6 (handler
// 同步 ACK + 异步 CLEAR race 引起 spurious re-fire 是回退原因).
//
// fixture 通过 test_dev MMIO 写 (sw TEST_DEV_SET_OFF / CLEAR_OFF) 触发; 详
// src/device/test_dev.{h,c}.

void device_set_pending(uint32_t source_id) {
    if (source_id == 0u || source_id >= PLIC_N_SOURCES) return;
    plic_wrlock();
    plic.sources[source_id].device_line = 1;
    plic_recompute_all_ctx_eip_locked();
    plic_recompute_pending_bitmap_locked();
    plic_unlock();
}

void device_clear_pending(uint32_t source_id) {
    if (source_id == 0u || source_id >= PLIC_N_SOURCES) return;
    plic_wrlock();
    plic.sources[source_id].device_line = 0;
    plic_recompute_all_ctx_eip_locked();
    plic_recompute_pending_bitmap_locked();
    plic_unlock();
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
    /* 015 加: 反查表 plic_ctx_to_hartid 全 -1 init, 服务 wfi_kick on eip 0→1 */
    for (uint32_t c = 0; c < PLIC_N_CONTEXTS; c++) {
        plic.plic_ctx_to_hartid[c] = -1;
    }
    for (uint32_t h = 0; h < MAX_HARTS; h++) {
        plic.plic_ctx_map[(h << 2) | PRIV_M] = (int8_t)(2u * h);
        plic.plic_ctx_map[(h << 2) | PRIV_S] = (int8_t)(2u * h + 1u);
        /* 反查表同处填: ctx 2h ↔ hart h (M), ctx 2h+1 ↔ hart h (S) */
        plic.plic_ctx_to_hartid[2u * h]      = (int8_t)h;
        plic.plic_ctx_to_hartid[2u * h + 1u] = (int8_t)h;
    }

    /* plic_ctx_eip 全 0 init (BSS 已 0, 显式 atomic_store lifecycle 可读 +
       memory_order_release 确保后续线程看到 0 起点)。 */
    for (uint32_t c = 0; c < PLIC_N_CONTEXTS; c++) {
        atomic_store_explicit(&plic_ctx_eip[c], 0, memory_order_release);
    }

    /* plic_pending_bitmap_cache 全 0 init (跟 plic_ctx_eip 同体例)。 */
    for (uint32_t w = 0; w < (PLIC_N_SOURCES + 31u) / 32u; w++) {
        atomic_store_explicit(&plic_pending_bitmap_cache[w], 0, memory_order_release);
    }

    /* pthread_rwlock_init: 默认 attr; 失败按 dummy.txt §5 fprintf + 返 -1。 */
    int rc = pthread_rwlock_init(&plic.lock, NULL);
    if (rc != 0) {
        fprintf(stderr, "plic_init: pthread_rwlock_init failed: rc=%d" EOL, rc);
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
        fprintf(stderr, "plic_init: bus_register_mmio failed" EOL);
        (void)pthread_rwlock_destroy(&plic.lock);
        return -1;
    }
    return 0;
}

int plic_reset(void) {
    /* system reset 每 iter: sources / contexts 字段清 + plic_ctx_eip + pending
       bitmap cache 清; plic_ctx_map 不动 (硬件 wired 状态不掉电不停, 跟 mtime
       不动同性质); 锁不动 (基础设施)。 */
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
    for (uint32_t w = 0; w < (PLIC_N_SOURCES + 31u) / 32u; w++) {
        atomic_store_explicit(&plic_pending_bitmap_cache[w], 0, memory_order_release);
    }
    plic_unlock();

    return 0;
}

void plic_destroy(void) {
    /* 纯模块 cleanup — 跟 clint_destroy / cpu_destroy 同 lifecycle 对称.
       PLIC 无辅助线程, 只 rwlock 一把锁需清.
       pthread_rwlock_destroy 在 glibc 下基本零开销; 但 valgrind/ASan 在意
       内部 leak, 显式 destroy 干净. */
    (void)pthread_rwlock_destroy(&plic.lock);
}
