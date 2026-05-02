//
// Created by liujilan on 2026/4/29.
// a01_2 tlb 模块实现 (tlb_alloc / tlb_clear)。
// 顶部模块文档见 tlb.h。报错风格见 src/dummy.txt §5。
//

#include "tlb.h"

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
                "tlb_alloc: aligned_alloc(64, %zu) failed: %s\n",
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
