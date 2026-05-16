//
// Created by liujilan on 2026/5/5.
// isa/lsu —— store_helper 实现 (extern, slow path)。
//
// 顶部接口 doc + 不对称设计背景 + trap 协议见 lsu.h。跨文件协议见 src/dummy.txt §1 末段。
//
// load_helper 在 lsu.h 是 static inline (fast path, 调用方内联进 case); store_helper 在本文件
// 是 extern 函数 (slow path, helper call)。两者签名风格对齐, 实现风格不对称。
//
// 当前形态:
//   - misalign → trap_raise(6)
//   - BARE 路径: identity + RAM 检查 + host store
//     - 不在 RAM → bus_dispatch_write (_Noreturn-on-failure, MMIO 派发; dummy.txt §8 / §9)
//   - SV32 路径: TLB lookup fast path + miss/D=0 fall back walker_helper_store
//   - reservation 清除 注释占位 (LR/SC 真做时填; A 扩展)
//   - SMC page_dirty 检测 注释占位 (jit/smc.c 真做时填; JIT 阶段)
//

#include "lsu.h"

#include <stdint.h>
#include <string.h>     // memcpy: 防 strict-aliasing / unaligned 风险

#include "config.h"          // TLB_NUM_ENTRIES (SV32 fast path 用)
#include "core/mmu.h"        // mmu_walker_helper_store (SV32 miss / D=0 / perm 不齐 fall back)
#include "platform/bus.h"    // bus_dispatch_write (BARE MMIO 派发, _Noreturn-on-failure)


void store_helper(cpu_t *hart, tlb_t *current_tlb,
                  uint32_t gva, uint32_t value, uint32_t size) {
    // Step 1: misalign (Spike 风格严格对齐)。size=1 时 mask=0, SB 永远过。
    if ((gva & (size - 1u)) != 0u) {
        trap_raise_exception(hart, CAUSE_STORE_ADDR_MISALIGNED, /*tval*/gva);  // _Noreturn longjmp
    }

    // Step 2: REGIME_BARE (current_tlb == NULL) — identity + RAM 检查 + host store
    if (current_tlb == NULL) {
        uint32_t pa = gva;  // identity
        // RAM 区检查 (无符号下溢比较)。
        if ((uint32_t)(pa - GUEST_RAM_START) >= GUEST_RAM_SIZE) {
            // PA 不在 RAM 区 → MMIO 派发 (不入 TLB; plan §1.4 + dummy.txt §8 / §9)。
            // bus_dispatch_write 内部 _Noreturn-on-failure (未命中 / device 拒绝
            // 都 longjmp, 不返回 caller)。
            // 注: 未来 ROM 真接时, "写 ROM = access fault" 由 ROM device write_fn
            // 返非 0 cause 表达, bus 透传给 trap_raise — 仍走这条路径。
            bus_dispatch_write(hart, pa, /*gva for tval*/gva, value, size);
            return;
        }

        uint8_t *host_ptr = gpa_to_hva_offset + pa;
        // host store: memcpy size 字节 (低 size 字节进 host_ptr; SB 写 1 字节 / SH 写 2 字节 /
        // SW 写 4 字节)。memcpy 防 strict-aliasing / unaligned 风险。
        memcpy(host_ptr, &value, size);

        // ----------------------------------------------------------------------
        // reservation 清除 (LR/SC 语义) —— 占位
        //
        // RV A 扩展: 任何 store (普通 SW 或 AMO) 都可能让某 hart 的 LR-reserved 地址失效。
        // 真做 isa/amo.c 时:
        //   - 清当前 hart 的 reservation (如果存在)
        //   - SMP 时 (本项目预留, 不实现): 跨 hart 同步 reservation table (atomic 字段)
        // 当前没有 LR/SC, reservation_t struct 也未定; 占位等真做。
        // ----------------------------------------------------------------------

        // ----------------------------------------------------------------------
        // SMC 检测 (page_dirty bitmap) —— 占位
        //
        // JIT 接入后: store 写到含 JIT 翻译过的 page 时, 配合 jit/smc.c 的 page_dirty bitmap
        // 检测, 让 dispatcher 在下次进 block 前 invalidate 该 page 上所有 jit_cache 条目
        // (整页失效 — 不是精细; plan §1.17 + §3 #13 决策)。
        // 当前没 JIT 也没 jit_cache, 占位等真做。
        //
        // 历史路径备注: 替代设计是 "SIGSEGV write-protect → handler 设 dirty"; 但 store_helper
        // 内主动设 dirty 也是合法路径 (handler 仅服务真正 inline 的 store fast path; helper 路径
        // 自己设更直接, 不需要 SIGSEGV 介入)。
        // ----------------------------------------------------------------------

        return;
    }

    // Step 3: REGIME_SV32 (current_tlb 非 NULL) — TLB lookup fast path + miss/D=0 fall back
    //
    // 注意: store_helper 整体是 slow path (extern 函数, 已付 helper call 开销), 这段 "TLB
    // 命中 fast path" 是 helper 内的快速路径 — 跟 load_helper (lsu.h static inline 真 fast
    // path) 性质不同。命中段调 check_perm 是 inline 还是 extern 都可以, 当前用 mmu.h 的
    // static inline 形态跟 load_helper / mmu_translate_pc 共用一份。命名澄清见 dummy.txt §1
    // F1 段 (store_helper inline 化 ≠ store 变 fast path)。
    //
    // 命中条件: V + tag + D + check_perm(MMU_PERM_W)
    //   D 位必查 — load 路径 walker 不 set D, store 时 D=0 必 fall back walker 重 set
    //   (hw-managed D 关键路径时序场景: walker 第一次 set D 写回 PT + 重 fill TLB; 之后
    //   fast path 直接 host store)
    //   check_perm 内查 priv/PTE_U/SUM + W 位; W=1+R=0 reserved 不查 (跟 spike 同)
    //
    // 简化命中 (V + tag + W + D) 不够 — corner case (S+PTE.U=1+SUM=0): S 模式访问 user
    //   page 没 SUM 时, 看 W=1 + D=1 就 host store 通过, 但 spec 规定应 store page fault。
    //   check_perm inline 形式让命中段跟 walker 同源, 不漏 corner case。
    //
    // 不命中 (miss / V=0 / D=0 / perm 不齐) → fall back mmu_walker_helper_store —
    // walker 内重做 walk + check_perm + set A+D 写回 PT + fill TLB; 失败 trap_raise
    // (cause 15 page fault / cause 7 access fault) 长跳, 不返回 caller。
    {
        const uint32_t vpn   = gva >> 12;
        const uint32_t index = vpn & (TLB_NUM_ENTRIES - 1);
        tlb_e_t *entry = &current_tlb->e[index];

        /* 命中: V + tag + D + check_perm(W) (A 位 walker 进 TLB 时永远 set, check_perm 内
         * 不查 A; D 位必查 — load 路径 walker 不 set D, store 时 D=0 需 fall back walker 重
         * set; check_perm 完整覆盖 priv/PTE_U/SUM + W 位, 跟 walker 同源) */
        if ((entry->pte_flags & PTE_V)
            && entry->gva_tag == vpn
            && (entry->pte_flags & PTE_D)
            && check_perm(hart, (uint32_t)entry->pte_flags, MMU_PERM_W)) {
            uint8_t *host_ptr = entry->host_ptr + (gva & 0xFFFu);
            memcpy(host_ptr, &value, size);

            /* reservation 清除 (LR/SC) + SMC page_dirty 占位 — 跟 BARE 路径同形态;
             * 当前未接 LR/SC, jit/smc.c 也未接, 占位等 A 扩展 / JIT 真做。 */
            return;
        }
    }
    /* fall back to walker (含完整 perm 检查 + set A+D 写回 PT + fill TLB; 失败 trap_raise
     * 长跳; reservation/SMC 副作用一并由 walker_helper_store 内做) */
    mmu_walker_helper_store(hart, current_tlb, gva, value, size);
}
