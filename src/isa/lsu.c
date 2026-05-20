//
// Created by liujilan on 2026/5/5.
// isa/lsu —— store_helper 实现 (extern, HVA-based, RAM 写 + 副作用)。
//
// 顶部接口 doc + 调用拓扑 + 不对称真机理 + misalign 隐式契约 见 lsu.h。
//
// store_helper 形态:
//   - HVA-based 接口, 不做 BARE/SV32 / IS_GPA_RAM 分流; 只做 RAM 写 + 副作用
//     (LR/SC + SMC 占位)
//   - 调用方 (lsu_store_helper BARE/SV32 hit / mmu_walker_helper_store RAM) 已确认
//     PA 落 RAM, 算好 hva 传入
//   - MMIO 路径 caller 直接调 mmio_write_helper, 不经 store_helper (跳过 LR/SC +
//     SMC 副作用; 跟 §8 三层模型一致, 因 MMIO 不参与 reservation 也非可执行
//     不参与 SMC)
//

#include "lsu.h"

#include <stdint.h>
#include <string.h>     // memcpy: 防 strict-aliasing / unaligned 风险


void store_helper(cpu_t *hart, uint8_t *hva, uxlen_t gva_for_tval,
                  uxlen_t value, uint32_t size) {
    (void)hart;          /* 当前 reservation/SMC 占位未真做, hart 未消费 */
    (void)gva_for_tval;  /* 同上, gva 仅供未来副作用 trap_raise 当 tval 透传 */

    // host store: memcpy size 字节 (低 size 字节进 hva; SB 写 1 字节 / SH 写 2 字节 /
    // SW 写 4 字节)。memcpy 防 strict-aliasing / unaligned 风险。
    memcpy(hva, &value, size);

    // ----------------------------------------------------------------------
    // reservation 清除 (LR/SC 语义) —— 占位
    //
    // RV A 扩展: 任何 store (普通 SW 或 AMO) 都可能让某 hart 的 LR-reserved 地址失效。
    // 真做 isa/amo.c 时:
    //   - 清当前 hart 的 reservation (如果存在)
    //   - SMP 时 (本项目预留, 不实现): 跨 hart 同步 reservation table (atomic 字段)
    // 当前没有 LR/SC, reservation_t struct 也未定; 占位等真做。
    //
    // 注: 本 helper 只服务 RAM 写 — MMIO 路径不调本 helper (mmio_write_helper 直接,
    // 不经此处), 因 LR/SC 对 MMIO RV spec implementation-defined 一般 access fault,
    // 不会建 reservation, 也就不需要清。
    // ----------------------------------------------------------------------

    // ----------------------------------------------------------------------
    // SMC 检测 (page_dirty bitmap) —— 占位
    //
    // JIT 接入后: store 写到含 JIT 翻译过的 page 时, 配合 jit/smc.c 的 page_dirty
    // bitmap 检测, 让 dispatcher 在下次进 block 前 invalidate 该 page 上所有
    // jit_cache 条目 (整页失效 — 不是精细; plan §1.17 + §3 #13 决策)。
    // 当前没 JIT 也没 jit_cache, 占位等真做。
    //
    // 注: MMIO 非可执行 page, 不参与 SMC (跟 reservation 同理由), 所以本 helper
    // 不服务 MMIO 路径。
    // ----------------------------------------------------------------------
}
