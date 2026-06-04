//
// Created by liujilan on 2026/6/4.
// isa/fence —— fence_helper / fence_i_helper 实现 (NOP + lrsc_clear_self).
//
// 顶部接口 doc + 双重 cover 论证 见 fence.h.
//

#include "fence.h"

#include "lrsc.h"            // lrsc_clear_self (七类清除时机 #4 fence.i)


void fence_helper(cpu_t *hart) {
    /* memory ordering NOP — 见 fence.h 双重 cover 论证 (host x86 TSO + helper atomic). */
    (void)hart;
}

void fence_i_helper(cpu_t *hart) {
    /* i-cache flush 副作用: 清当前 hart reservation. RV spec 七类清除 #4.
     * 未来 SMC chain (a_05+): 这里加 JIT cache page invalidate / page_dirty bitmap 重置. */
    lrsc_clear_self(hart);
}
