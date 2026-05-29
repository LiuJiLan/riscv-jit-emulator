//
// Created by liujilan on 2026/4/29.
// tlb 模块实现 (tlb_alloc / tlb_clear / tlb_table_reset)。
// 顶部模块文档见 tlb.h。报错风格见 src/dummy.txt §5。
//

#include "tlb.h"
#include "cpu.h"        // cpu_t 完整定义 (tlb_table_reset 需要访问 tlb_table[4])
#include "config.h"     // EOL (项目级 stderr 输出体例)
#include "riscv.h"      // PRIV_S / PRIV_H

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

tlb_t *tlb_alloc(void) {
    // aligned_alloc 要求 size 是 alignment 的倍数。
    // sizeof(tlb_t) = sizeof(tlb_e_t) * TLB_NUM_ENTRIES = 16 * 64 = 1024,
    // 1024 % 64 == 0, 满足。
    tlb_t *tlb = aligned_alloc(64, sizeof(tlb_t));
    if (tlb == NULL) {
        fprintf(stderr,
                "tlb_alloc: aligned_alloc(64, %zu) failed: %s" EOL,
                sizeof(tlb_t), strerror(errno));
        return NULL;
    }
    memset(tlb, 0, sizeof(tlb_t));   // V 位全 0 → 全 invalid
    return tlb;
}

void tlb_clear(tlb_t *tlb) {
    if (tlb == NULL) return;          // sfence helper 友好: 未分配槽位 do nothing
    memset(tlb, 0, sizeof(tlb_t));
}

void tlb_table_reset(cpu_t *hart) {
    if (hart == NULL) return;

    // [PRIV_S] S 槽 ASID 容器: 遍历每个 ASID 调 tlb_clear (entries 清; tlb_clear(NULL)
    // 是 no-op, 未懒分配的 entry 天然跳过)。
    if (hart->tlb_table[PRIV_S] != NULL) {
        for (uint32_t i = 0; i < ASID_MAX; i++) {
            tlb_clear(hart->tlb_table[PRIV_S][i]);
        }
    }

    // [PRIV_H] V 占位: 同 S 形态; 当前始终 NULL (无 H 扩展), 循环天然 no-op,
    // 写出来未来 H 扩展不漏。
    if (hart->tlb_table[PRIV_H] != NULL) {
        for (uint32_t i = 0; i < ASID_MAX; i++) {
            tlb_clear(hart->tlb_table[PRIV_H][i]);
        }
    }

    // [PRIV_M] / [PRIV_U] 不动: M 永远 NULL (Trust regime 不查 TLB); U 副本
    // 语义共享 S 的 entries, 不重复清。
}
