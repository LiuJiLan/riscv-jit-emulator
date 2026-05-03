//
// Created by liujilan on 2026/4/28.
// a01_2 cpu 模块实现 (cpu_create / cpu_destroy)。
//
// tlb_table[4] 设计 (D23 后):
//   - REGIME_BARE 走 identity bypass TLB → [PRIV_M] 槽永远 NULL, 不分配
//   - REGIME_SV32 走 [priv][asid] 二级索引 → [PRIV_S] 容器 eager 分配, entries 懒 (walker 填)
//   - [PRIV_U] alias [PRIV_S] (MSU 默认; U 共享 S 的 ASID 命名空间)
//   - [PRIV_H] VS 占位, NULL (未来 H 扩展)
//
// 报错风格见 src/dummy.txt §5。
//

#include "cpu.h"

#include "config.h"
#include "riscv.h"
#include "tlb.h"

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

cpu_t *cpu_create(uint32_t misa) {
    (void)misa;   // a_01 不读; 为未来 misa 驱动初始化预留 (MU-only / F/D/V 扩展 等)。

    cpu_t *hart = aligned_alloc(64, sizeof(cpu_t));
    if (hart == NULL) {
        fprintf(stderr, "cpu_create: aligned_alloc(64, %zu) failed: %s\n",
                sizeof(cpu_t), strerror(errno));
        return NULL;
    }
    memset(hart, 0, sizeof(*hart));

    hart->priv = PRIV_M;    // a_01: M 模式常量
    hart->satp = 0;         // a_01: bare 模式 (MODE=0, ASID=0, PPN=0 全 0)

    // ------------------------------------------------------------------------
    // tlb_table[4] 装载 (MSU 默认 misa; D23 后: Trust regime bypass TLB)
    //
    // [PRIV_M] M/bare 槽: D23 后**永远 NULL**。
    //                     Trust regime (M-mode 或 任何 priv 带 bare satp) 直接走 identity
    //                     + IS_GPA_RAM 检查, 不需要 TLB。real CPU 在 bare 下也 bypass MMU/TLB,
    //                     我们对齐。
    //                     memset 0 已经置 NULL, 不需显式赋值。
    //
    // [PRIV_S] S 槽: ASID 容器, entries 由 walker 在 Sv32 路径懒分配 (a_05+ 接入)。
    //
    // [PRIV_U] U 槽: alias [PRIV_S] (MSU 默认; U 与 S 共享 ASID 命名空间)。
    //                "U 是某个的副本"语义不变, 只是不再有 MU-only 时副本 [PRIV_M] 的需求
    //                (因为 [PRIV_M] 已 NULL, MU-only 时 U 直接走 BARE regime, 不查 TLB)。
    //                未来 mstatus / hstatus 写 helper 维护 mirror 切换。
    //
    // [PRIV_H] VS 占位: NULL (memset 0 已置), 未来 H 扩展激活时 calloc。
    //
    // 未来 misa 驱动派发 (占位注释):
    //   - MU-only ISA (类比 SiFive U74 的 M-only 不对称核): 没 S-mode, U-mode 即使
    //     satp.MODE = Sv32 也无意义 (没 PTE 设施); dispatcher 应识别 misa 把 U 路由到
    //     BARE regime。当前 a_01 默认 MSU 三态, 不实现这条分支, 但保留扩展点。
    //
    // memset 0 已经把 tlb_table[0..3] 置 NULL, 失败回滚路径上 free(NULL) 无害。
    // ------------------------------------------------------------------------

    // [PRIV_S] S: ASID 容器, entries 由 walker 在 Sv32 路径懒分配 (a_01 不触发)。
    hart->tlb_table[PRIV_S] = calloc(ASID_MAX, sizeof(tlb_t *));
    if (hart->tlb_table[PRIV_S] == NULL) {
        fprintf(stderr, "cpu_create: calloc tlb_table[PRIV_S] failed: %s\n", strerror(errno));
        cpu_destroy(hart);
        return NULL;
    }

    // [PRIV_U] U: alias [PRIV_S] (MSU 默认; U 共享 S 的 ASID 命名空间)。
    hart->tlb_table[PRIV_U] = hart->tlb_table[PRIV_S];

    // [PRIV_M] / [PRIV_H]: 不分配, memset 0 保证 NULL。

    return hart;
}

void cpu_destroy(cpu_t *hart) {
    if (hart == NULL) return;

    // [PRIV_S] S: 递归 free 非 NULL entries (walker 懒分配的叶 TLB), 然后 free 容器。
    if (hart->tlb_table[PRIV_S] != NULL) {
        for (uint32_t i = 0; i < ASID_MAX; i++) {
            free(hart->tlb_table[PRIV_S][i]);    // free(NULL) 无害, 未懒分配的槽位天然 no-op
        }
        free(hart->tlb_table[PRIV_S]);
    }

    // [PRIV_H] V 占位: 同 [PRIV_S] 模式; a_01 始终 NULL, 循环天然 no-op, 写出来未来 H 扩展不漏。
    if (hart->tlb_table[PRIV_H] != NULL) {
        for (uint32_t i = 0; i < ASID_MAX; i++) {
            free(hart->tlb_table[PRIV_H][i]);
        }
        free(hart->tlb_table[PRIV_H]);
    }

    // [PRIV_U] U: alias [PRIV_S], 不再 free。
    // [PRIV_M] M: D23 后不分配 (Trust regime 不走 TLB), 不再 free。

    free(hart);
}
