//
// Created by liujilan on 2026/5/5.
// a_01_6 isa/lsu —— store_helper 实现 (extern, slow path)。
//
// 顶部接口 doc + 不对称设计背景 + trap 协议见 lsu.h。跨文件协议见 src/dummy.txt §1 末段。
//
// load_helper 在 lsu.h 是 static inline (fast path, 调用方内联进 case); store_helper 在本文件
// 是 extern 函数 (slow path, helper call)。两者签名风格对齐, 实现风格不对称, 跟 file_plan
// §8.interpreter D 区设计一致。
//
// 当前 a_01_6 形态:
//   - misalign → trap_raise(6)
//   - BARE 路径: identity + RAM 检查 + host store
//     - 不在 RAM → fprintf "MMIO bus_dispatch not implemented" + trap_raise(7)
//   - SV32 路径占位: priv 恒 PRIV_M, current_tlb 永远 NULL, 不可达; fprintf + trap_raise(15)
//   - reservation 清除 注释占位 (LR/SC 真做时填; a_03+ A 扩展)
//   - SMC page_dirty 检测 注释占位 (jit/smc.c 真做时填; B 阶段 JIT)
//

#include "lsu.h"

#include <stdint.h>
#include <stdio.h>      // fprintf
#include <string.h>     // memcpy: 防 strict-aliasing / unaligned 风险


void store_helper(cpu_t *hart, tlb_t *current_tlb,
                  uint32_t gva, uint32_t value, uint32_t size) {
    // Step 1: misalign (Spike 风格严格对齐)。size=1 时 mask=0, SB 永远过。
    if ((gva & (size - 1u)) != 0u) {
        trap_raise_exception(hart, /*cause*/6u, /*tval*/gva);  // _Noreturn longjmp
    }

    // Step 2: REGIME_BARE (current_tlb == NULL) — identity + RAM 检查 + host store
    if (current_tlb == NULL) {
        uint32_t pa = gva;  // identity
        // RAM 区检查 (无符号下溢比较)。
        if ((uint32_t)(pa - GUEST_RAM_START) >= GUEST_RAM_SIZE) {
            // PA 不在 RAM 区。a_01_6 bus_dispatch 未实现, fprintf 提示 + trap access fault。
            // 未来 a_01_7+ 加 platform/bus.c 后, 这里改为 bus_dispatch 路径 (MMIO store 走 bus +
            // 也要看是不是 ROM, 写 ROM 是 access fault); 现在一律 cause 7 (Store/AMO Access Fault)。
            fprintf(stderr,
                    "[lsu] store: PA 0x%08x not in RAM (MMIO bus_dispatch not implemented in a_01_6)\n",
                    pa);
            trap_raise_exception(hart, /*cause*/7u, /*tval*/gva);  // _Noreturn longjmp
        }

        uint8_t *host_ptr = gpa_to_hva_offset + pa;
        // host store: memcpy size 字节 (低 size 字节进 host_ptr; SB 写 1 字节 / SH 写 2 字节 /
        // SW 写 4 字节)。memcpy 防 strict-aliasing / unaligned 风险。
        memcpy(host_ptr, &value, size);

        // ----------------------------------------------------------------------
        // reservation 清除 (LR/SC 语义) —— 占位
        //
        // RV A 扩展: 任何 store (普通 SW 或 AMO) 都可能让某 hart 的 LR-reserved 地址失效。
        // a_03+ 真做 isa/amo.c 时:
        //   - 清当前 hart 的 reservation (如果存在)
        //   - SMP 时 (本项目预留, 不实现): 跨 hart 同步 reservation table (atomic 字段)
        // 现在 a_01 没有 LR/SC, reservation_t struct 也未定; 占位等真做。
        // ----------------------------------------------------------------------

        // ----------------------------------------------------------------------
        // SMC 检测 (page_dirty bitmap) —— 占位
        //
        // B 阶段 JIT 接入后: store 写到含 JIT 翻译过的 page 时, 配合 jit/smc.c 的 page_dirty
        // bitmap 检测, 让 dispatcher 在下次进 block 前 invalidate 该 page 上所有 jit_cache
        // 条目 (整页失效 — 不是精细; plan §1.17 + §3 #13 决策)。
        // 当前 a_01 没 JIT 也没 jit_cache, 占位等真做。
        //
        // 历史路径备注: 替代设计是 "SIGSEGV write-protect → handler 设 dirty"; 但 store_helper
        // 内主动设 dirty 也是合法路径 (handler 仅服务真正 inline 的 store fast path; helper 路径
        // 自己设更直接, 不需要 SIGSEGV 介入)。
        // ----------------------------------------------------------------------

        return;
    }

    // Step 3: REGIME_SV32 (current_tlb 非 NULL) — TLB lookup + walker_helper_store miss
    //
    // a_01_7+ 真做时:
    //   - TLB hit (V + tag + W 位 + priv/SUM 检查) → host store + reservation 清除 + SMC 设 dirty
    //   - TLB miss → walker_helper_store(hart, gva, value, size) (mmu.c 加, 走页表 + fill TLB +
    //                  host store + reservation/SMC 副作用; 接口设计 a_01_7 真做时定)
    //   - TLB hit 权限错 → trap_raise_exception(15 store page fault, gva)
    //
    // a_01_6 占位: 不可达 (priv 恒 PRIV_M); fprintf + trap_raise(15) 防御。
    fprintf(stderr,
            "[lsu] store: SV32 path not implemented in a_01_6 (gva=0x%08x value=0x%08x size=%u)\n",
            gva, value, size);
    trap_raise_exception(hart, /*cause*/15u, /*tval*/gva);  // _Noreturn longjmp
}
