//
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
     * 不主动调 jit_cache_invalidate_page — 详 fence.h 顶段 "fence_i_helper 不调 ... 为什么" 段. */
    lrsc_clear_self(hart);
}
