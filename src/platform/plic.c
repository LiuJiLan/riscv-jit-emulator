//
// Created by liujilan on 2026/5/23.
// PLIC 实现 — per-source <device_line, claimed> + plic_ctx_map + bus 注册 + MMIO
// read/write 骨架 (T1: 地址解码 + fprintf 标识; 真仲裁 / 外设接通 / hart 侧合成读
// 留 T2)。
//
// 接口形态 + monitor 模型 + 字段对应 spec + 命名选择见 plic.h 顶段 doc。
// 地址布局见 config.h PLIC_* 宏 (跟 QEMU virt + RV PLIC spec v1.0.0 兼容)。
// 报错风格见 dummy.txt §5; "0=成功 / 非0=cause" 接口约定见 dummy.txt §9。
//

#include "plic.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config.h"          // PLIC_* / MAX_HARTS
#include "platform/bus.h"    // mmio_dev_t / bus_register_mmio
#include "riscv.h"           // CAUSE_LOAD/STORE_ACCESS_FAULT


// ----------------------------------------------------------------------------
// PLIC 内部状态 (单例 file-static)
// ----------------------------------------------------------------------------
//
// 字段 NOT _Atomic — PLIC 走全局锁 (T2 真上 pthread_mutex_t), 锁本身是 happens-before
// 边界, _Atomic 在锁内冗余 (plan §1.9 "shared 数据 atomic 起步" 针对的是 clint 那种
// 无锁场景: timer thread 跟主帧并发 mtime; 有锁场景不适用)。SMP 真切锁拆细 / RCU 时
// 才需要补 _Atomic。
//
// sources 数组: array of struct, 每 source 两字段 — device_line (level 模型设备拉线
// 状态) + claimed (gateway forward latch 等价物, 0=未 claim / 非 0 = ctx_id + 1)。
// claim 编码 +1 是 PLIC 内部约定, 不暴露 ABI (hart 读 claim 寄存器拿到的是 source_id,
// 不是 claimed 字段; 详 plic.h 顶段)。Linux 启动遇到 ctx_id=-1 退出 init 的坑也避开。
//
// plic_ctx_map: hart_priv → ctx_id 反向映射表。index = (hartid << 2) + priv (复用
// cpu_t.tlb_table 的 priv encoding: U=0 / S=1 / VS=2 / M=3); 元素 = 0..PLIC_N_CONTEXTS-1;
// -1 = 没连线 (跟 dtb interrupts-extended 没列出来的 hart×priv 对应)。当前 T1 阶段
// 全 -1 init; T4 真做 enable/threshold 时按设备树补两槽 (M + S per hart)。
//
// T1 阶段所有访问只走 plic_read / plic_write fprintf 不动字段, 全局锁暂不需要;
// T2 加 device_set/clear_pending + claim/complete 真实装时再加 pthread_mutex_t。
static struct {
    struct {
        uint8_t   device_line;     /* bool 形态; 设备拉线状态 (level 模型镜像) */
        uint32_t  claimed;         /* 0=未 claim; 非 0 = ctx_id + 1 (PLIC 内部编码) */
    } sources[PLIC_N_SOURCES];     /* source 0 保留, 索引从 1 起用 */
    int8_t        plic_ctx_map[MAX_HARTS * 4];  /* index = hartid<<2+priv; -1 = 没连线 */
} plic;


// ----------------------------------------------------------------------------
// plic_read / plic_write — bus 派发入口 (T1 骨架: 地址解码 + fprintf 标识)
// ----------------------------------------------------------------------------
//
// 接口语义 (dummy.txt §9):
//   返 0    = 成功; read 路径返常量 0 / write 路径丢弃
//   返非 0  = cause (RV spec exception code); bus 透传给 trap_raise_exception
//
// T1 fault 政策 (跟 RV PLIC spec / QEMU sifive_plic 一致):
//   - size != 4 / off 未对齐 → CAUSE_*_ACCESS_FAULT (硬件级硬限制, PLIC 寄存器都
//     32-bit 对齐, 1B/2B/8B 访问 + 非 4 对齐 不是合法访问形态)
//   - PLIC region 内越界 (src_id >= PLIC_N_SOURCES / ctx_id >= PLIC_N_CONTEXTS) →
//     silent ignore, read 返 0 / write 丢弃, **不 fprintf 不 fault**
//   - PLIC region 内 reserved (context 区 off_in_ctx 不是 0/4) → silent ignore (跟
//     越界同处理), **但 fprintf 标识 "reserved"** ("软件不该访问" 的报告; spec 说
//     "Software should not access reserved registers")
//   - PLIC region 外 (off >= PLIC_SIZE) → bus 已经 range check 拦, 进不到 plic_*;
//     plic_* 内部 final else 保留 fault 是防御性, 实际不会走到
//
// 即: "PLIC region 内 silent (除 reserved fprintf 报告)" + "PLIC 区外 bus 自产 fault"
// 是边界 fault 语义切分。fixture 02 走"区内 silent + 区外 fault" 双路径验证。
//
// fprintf 暂时不 gate (骨架阶段, 方便人工观察跑期解码; T4 之前整理时再决定加
// DEBUG_PLIC_ON gate)。

static int plic_read(void *ctx, uint32_t off, void *buf, uint32_t size) {
    (void)ctx;
    if (size != 4u)              return CAUSE_LOAD_ACCESS_FAULT;
    if ((off & 0x3u) != 0u)      return CAUSE_LOAD_ACCESS_FAULT;

    uint32_t value = 0;

    if (off < (uint32_t)PLIC_PRIORITY_OFF + 0x1000u) {
        /* priority 区: [0, 0x1000); per-source 4B */
        uint32_t source_id = off / 4u;
        if (source_id < PLIC_N_SOURCES) {
            fprintf(stderr, "[plic] priority src=%u R\n", source_id);
        }
        /* src_id >= PLIC_N_SOURCES: silent ignore */

    } else if (off < (uint32_t)PLIC_ENABLE_OFF) {
        /* pending 区: [0x1000, 0x2000); 1 bit/source, 32 src/word */
        uint32_t word_idx  = (off - (uint32_t)PLIC_PENDING_OFF) / 4u;
        uint32_t src_base  = word_idx * 32u;
        if (src_base < PLIC_N_SOURCES) {
            fprintf(stderr, "[plic] pending word=%u src=[%u..%u] R\n",
                    word_idx, src_base, src_base + 31u);
        }
        /* word_idx 越界: silent ignore */

    } else if (off < (uint32_t)PLIC_CONTEXT_OFF) {
        /* enable 区: [0x2000, 0x200000); per-ctx, 0x80 B/ctx, 1 bit/source */
        uint32_t bo        = off - (uint32_t)PLIC_ENABLE_OFF;
        uint32_t ctx_id    = bo / (uint32_t)PLIC_ENABLE_STRIDE;
        uint32_t word_idx  = (bo % (uint32_t)PLIC_ENABLE_STRIDE) / 4u;
        uint32_t src_base  = word_idx * 32u;
        if (ctx_id < PLIC_N_CONTEXTS) {
            fprintf(stderr, "[plic] enable ctx=%u src=[%u..%u] R\n",
                    ctx_id, src_base, src_base + 31u);
        }
        /* ctx_id 越界: silent ignore */

    } else if (off < (uint32_t)PLIC_SIZE) {
        /* context 区: [0x200000, 0x600000); per-ctx, 4 KB/ctx; +0 threshold, +4 claim */
        uint32_t bo        = off - (uint32_t)PLIC_CONTEXT_OFF;
        uint32_t ctx_id    = bo / (uint32_t)PLIC_CONTEXT_STRIDE;
        uint32_t off_in_ctx = bo % (uint32_t)PLIC_CONTEXT_STRIDE;
        if (ctx_id < PLIC_N_CONTEXTS) {
            const char *region = (off_in_ctx == 0u) ? "threshold"
                               : (off_in_ctx == 4u) ? "claim"
                               :                      "reserved";
            /* reserved 仍 fprintf 报告 ("软件不该访问" 标识; spec §xx);
               越界 silent; 命中 (threshold / claim) 也 fprintf. */
            fprintf(stderr, "[plic] ctx=%u %s R\n", ctx_id, region);
        }
        /* ctx_id 越界: silent ignore */

    } else {
        /* off >= PLIC_SIZE: 实际 bus range check 已拦, 这里防御性 fault. */
        return CAUSE_LOAD_ACCESS_FAULT;
    }

    memcpy(buf, &value, 4);
    return 0;
}

static int plic_write(void *ctx, uint32_t off, const void *buf, uint32_t size) {
    (void)ctx;
    (void)buf;                   /* T1 骨架: write value 暂丢弃, 真用 T2 加 */
    if (size != 4u)              return CAUSE_STORE_ACCESS_FAULT;
    if ((off & 0x3u) != 0u)      return CAUSE_STORE_ACCESS_FAULT;

    if (off < (uint32_t)PLIC_PRIORITY_OFF + 0x1000u) {
        uint32_t source_id = off / 4u;
        if (source_id < PLIC_N_SOURCES) {
            fprintf(stderr, "[plic] priority src=%u W\n", source_id);
        }
        /* src_id 越界: silent ignore */

    } else if (off < (uint32_t)PLIC_ENABLE_OFF) {
        uint32_t word_idx  = (off - (uint32_t)PLIC_PENDING_OFF) / 4u;
        uint32_t src_base  = word_idx * 32u;
        if (src_base < PLIC_N_SOURCES) {
            fprintf(stderr, "[plic] pending word=%u src=[%u..%u] W\n",
                    word_idx, src_base, src_base + 31u);
        }
        /* word_idx 越界: silent ignore. 注: pending 区 spec 是 RO, 软件写应 ignore;
           这里 fprintf 提示软件写过 pending, 但不拦. */

    } else if (off < (uint32_t)PLIC_CONTEXT_OFF) {
        uint32_t bo        = off - (uint32_t)PLIC_ENABLE_OFF;
        uint32_t ctx_id    = bo / (uint32_t)PLIC_ENABLE_STRIDE;
        uint32_t word_idx  = (bo % (uint32_t)PLIC_ENABLE_STRIDE) / 4u;
        uint32_t src_base  = word_idx * 32u;
        if (ctx_id < PLIC_N_CONTEXTS) {
            fprintf(stderr, "[plic] enable ctx=%u src=[%u..%u] W\n",
                    ctx_id, src_base, src_base + 31u);
        }
        /* ctx_id 越界: silent ignore */

    } else if (off < (uint32_t)PLIC_SIZE) {
        uint32_t bo        = off - (uint32_t)PLIC_CONTEXT_OFF;
        uint32_t ctx_id    = bo / (uint32_t)PLIC_CONTEXT_STRIDE;
        uint32_t off_in_ctx = bo % (uint32_t)PLIC_CONTEXT_STRIDE;
        if (ctx_id < PLIC_N_CONTEXTS) {
            const char *region = (off_in_ctx == 0u) ? "threshold"
                               : (off_in_ctx == 4u) ? "complete"
                               :                      "reserved";
            fprintf(stderr, "[plic] ctx=%u %s W\n", ctx_id, region);
        }
        /* ctx_id 越界: silent ignore */

    } else {
        return CAUSE_STORE_ACCESS_FAULT;
    }
    return 0;
}


// ----------------------------------------------------------------------------
// lifecycle: plic_init / plic_reset / plic_destroy
// ----------------------------------------------------------------------------

int plic_init(void) {
    /* 字段 0 init — BSS 已经 0, 显式 memset 仅 lifecycle 可读 (跟 clint_init 同体例)。 */
    memset(&plic, 0, sizeof(plic));

    /* plic_ctx_map 全 -1 init: 未来设备树 init 阶段填两槽 (M + S per hart);
       T1 阶段无外部填表, T2 真接通 is_plic_*_pending 时按需补 init 路径。 */
    for (uint32_t i = 0; i < MAX_HARTS * 4u; i++) {
        plic.plic_ctx_map[i] = -1;
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
        return -1;
    }
    return 0;
}

int plic_reset(void) {
    /* system reset 每 iter: device_line / claimed 清 (跟 clint_reset mtimecmp/msip
       清同体例); plic_ctx_map 不动 (未来设备树 init 阶段填一次后跨 reset 持续,
       跟 mtime 不动同性质 — 硬件 wired 状态不掉电不停)。 */
    for (uint32_t i = 0; i < PLIC_N_SOURCES; i++) {
        plic.sources[i].device_line = 0;
        plic.sources[i].claimed     = 0;
    }
    return 0;
}

void plic_destroy(void) {
    /* 纯模块 cleanup — 跟 clint_destroy / cpu_destroy 同, 函数留作 lifecycle 对称。
       bus 未来加 unregister 时这里调; 当前 atomic / 字段值在进程退出时不影响别处。 */
}
