//
// Created by liujilan on 2026/5/23.
// PLIC 实现 — per-source <device_line, claimed, priority> + per-ctx <threshold,
// enable bitmap> + plic_ctx_map + bus 注册 + 完整 claim/complete 仲裁 + hart 侧
// is_plic_*_pending 合成 view + 外设侧 device_set/clear_pending 接口。
//
// 接口形态 + monitor 模型 + rwlock 抽象 + 字段对应 spec + 命名选择见 plic.h 顶段 doc。
// 地址布局见 config.h PLIC_* 宏 (跟 QEMU virt + RV PLIC spec v1.0.0 兼容)。
// 报错风格见 dummy.txt §5; "0=成功 / 非0=cause" 接口约定见 dummy.txt §9。
//

#include "plic.h"

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config.h"          // PLIC_* / MAX_HARTS
#include "platform/bus.h"    // mmio_dev_t / bus_register_mmio
#include "riscv.h"           // CAUSE_LOAD/STORE_ACCESS_FAULT / PRIV_M / PRIV_S


// ----------------------------------------------------------------------------
// PLIC 内部状态 (单例 file-static)
// ----------------------------------------------------------------------------
//
// 字段 NOT _Atomic — PLIC 走 pthread_rwlock_t (plic_rdlock / plic_wrlock 包装), 锁本身
// 是 happens-before 边界, _Atomic 在锁内冗余 (plan §1.9 "shared 数据 atomic 起步"
// 针对 clint 那种无锁场景: timer thread 跟主帧并发 mtime; 有锁场景不适用)。SMP 真切
// 锁拆细 / RCU 时才需要补 _Atomic。
//
// sources 数组: per-source 三字段:
//   device_line — 设备拉线状态 (level 模型镜像, device_set/clear_pending 拉)
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
// monitor RW helper (file-static)
// ----------------------------------------------------------------------------
//
// 包装 pthread_rwlock_t 的 rdlock / wrlock / unlock; 不检 rc (POSIX rwlock 在 init
// 成功后, lock/unlock 只在 EDEADLK/EINVAL 等编程错误下失败, v1 不处理 — 真撞 ASan
// + sanitizer 抓住)。SMP 单 hart 下 contention=0, glibc 走 fast path 零开销。

static void plic_rdlock(void) { (void)pthread_rwlock_rdlock(&plic.lock); }
static void plic_wrlock(void) { (void)pthread_rwlock_wrlock(&plic.lock); }
static void plic_unlock(void) { (void)pthread_rwlock_unlock(&plic.lock); }


// ----------------------------------------------------------------------------
// 仲裁 helper (file-static; 调用方已持锁; "_locked" 后缀提醒)
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
// plic_ctx_has_pending_locked — is_plic_*_pending 内调; 只判 "有任一可送 source",
// 不算具体哪个 (短路返 1)。

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
//   - wrlock: priority / threshold / enable / complete 写; claim 读 (改 claimed)
//   - 不 lock: pending 写 (spec RO, silent ignore 不动字段); reserved (silent 返 0);
//              size/对齐 fault (前置 check, 不进 lock)

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
                plic_unlock();
            } else if (off_in_ctx == 4u) {
                /* complete: 校验 sources[value].claimed == ctx+1 后清; 否则 silent
                   (RV PLIC spec §8 "If complete value does not match active claim,
                    write is ignored"). */
                if (value > 0u && value < PLIC_N_SOURCES) {
                    plic_wrlock();
                    if (plic.sources[value].claimed == ctx_id + 1u) {
                        plic.sources[value].claimed = 0;
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
// priv 编码进函数名 (跟 is_clint_msip_pending 同体例); 内部查 plic_ctx_map 拿 ctx_id,
// <0 返 0 防御 (没连线 hart×priv); 持 rdlock 调 plic_ctx_has_pending_locked 仲裁。
//
// 热路径 — csr_mip_read 每次 trap_check_interrupt 调; SMP 多 hart 真起来时是 reader
// 并发的主要场景。pthread_rwlock_t 在 glibc 下读路径 fast path (无 writer 时) 零阻塞。

int is_plic_meip_pending(uint32_t hartid) {
    if (hartid >= MAX_HARTS) return 0;
    int8_t ctx = plic.plic_ctx_map[(hartid << 2) | PRIV_M];
    if (ctx < 0) return 0;
    plic_rdlock();
    int r = plic_ctx_has_pending_locked((uint32_t)ctx);
    plic_unlock();
    return r;
}

int is_plic_seip_pending(uint32_t hartid) {
    if (hartid >= MAX_HARTS) return 0;
    int8_t ctx = plic.plic_ctx_map[(hartid << 2) | PRIV_S];
    if (ctx < 0) return 0;
    plic_rdlock();
    int r = plic_ctx_has_pending_locked((uint32_t)ctx);
    plic_unlock();
    return r;
}


// ----------------------------------------------------------------------------
// 外部接口: device_set/clear_pending (外设侧)
// ----------------------------------------------------------------------------
//
// 外设内部状态机自己决定何时拉高/拉低 device_line, 通过这俩接口通知 PLIC (UART /
// virtio-blk 等)。source_id 0 / 越界 silent ignore (设备树没分配的 source 走这里
// 不该出问题 — 防御性)。
//
// T3 UART 接入前, fixture 通过 csr.c 的 CSR_TOHOST (0x800) 写改造路径触发 (bit31=
// set/clear, 低 31 bits=source_id); T3 接 UART 后改造还原。

void device_set_pending(uint32_t source_id) {
    if (source_id == 0u || source_id >= PLIC_N_SOURCES) return;
    plic_wrlock();
    plic.sources[source_id].device_line = 1;
    plic_unlock();
}

void device_clear_pending(uint32_t source_id) {
    if (source_id == 0u || source_id >= PLIC_N_SOURCES) return;
    plic_wrlock();
    plic.sources[source_id].device_line = 0;
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
    for (uint32_t h = 0; h < MAX_HARTS; h++) {
        plic.plic_ctx_map[(h << 2) | PRIV_M] = (int8_t)(2u * h);
        plic.plic_ctx_map[(h << 2) | PRIV_S] = (int8_t)(2u * h + 1u);
    }

    /* pthread_rwlock_init: 默认 attr; 失败按 dummy.txt §5 fprintf + 返 -1。 */
    int rc = pthread_rwlock_init(&plic.lock, NULL);
    if (rc != 0) {
        fprintf(stderr, "plic_init: pthread_rwlock_init failed: rc=%d\n", rc);
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
        return -1;
    }
    return 0;
}

int plic_reset(void) {
    /* system reset 每 iter: sources / contexts 字段清; plic_ctx_map 不动 (硬件 wired
       状态不掉电不停, 跟 mtime 不动同性质); 锁不动 (基础设施)。
       不持锁 — system reset 时 dispatcher 主帧停, 没有其他线程访问 PLIC. */
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
    }
    return 0;
}

void plic_destroy(void) {
    /* 纯模块 cleanup — 跟 clint_destroy / cpu_destroy 同 lifecycle 对称。
       pthread_rwlock_destroy 释放 rwlock 内部资源 (glibc 下基本零开销; 但 valgrind/
       ASan 在意 rwlock 内部 leak, 显式 destroy 干净)。 */
    (void)pthread_rwlock_destroy(&plic.lock);
}
