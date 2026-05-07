//
// Created by liujilan on 2026/5/7.
// a_01_8 isa/sfence —— sfence_vma_helper 实现 (extern, slow path)。
//
// 顶部接口 doc + RV spec 四组合 vs 简化方案 4.a 对照 + 涉及哪些 tlb_table[] 槽 见 sfence.h。
//

#include "sfence.h"

#include <stddef.h>          // NULL (a_01_8 v01 加; sfence.c 内 if (tlb_table[priv] != NULL) 用)
#include <stdint.h>

#include "config.h"          // ASID_MAX, ASID_MASK
#include "core/tlb.h"        // tlb_clear
#include "riscv.h"           // PRIV_S, PRIV_H


void sfence_vma_helper(cpu_t *hart,
                       uint32_t vaddr_val, uint32_t asid_val,
                       uint32_t rs1,       uint32_t rs2) {
    /* 4.a 简化下不读 vaddr_val (因为 (b)/(d) 都退化为全清, 不需要 vaddr 维度);
     * 接口里保留 vaddr_val 参数是为未来精确实现 (b)/(d) 时启用, 不需要改签名。
     * 抑制 -Wunused-parameter (CMakeLists.txt 已加 -Wno-unused-parameter, 但显式 (void)
     * cast 表达"故意不读"语义)。 */
    (void)vaddr_val;

    /* 路由 RV spec 4 组合到简化方案 (a) 全清 / (c) 单 ASID 清:
     *   (a) rs1=x0, rs2=x0  → 全清
     *   (b) rs1!=x0, rs2=x0 → 全清 (过度刷新)
     *   (c) rs1=x0, rs2!=x0 → 单 ASID 清
     *   (d) rs1!=x0, rs2!=x0 → 全清 (过度刷新)
     * 即只有 (rs1=x0, rs2!=x0) 走单 ASID 清, 其他三组合都全清。
     *
     * 关键: 这里比的是寄存器**编号** (rs1==0 即 RV spec "rs1=x0" 编码), 不是寄存器
     * 值 vaddr_val/asid_val。RV spec sfence.vma 用 "rs1=x0/rs2=x0" 这个 magic 编码标
     * 记忽略对应维度; vaddr=0/asid=0 是合法的实值不算忽略。详见 sfence.h 接口 doc。 */
    int single_asid_mode = (rs1 == 0u) && (rs2 != 0u);

    /* tlb_table[priv] 是 ASID 容器指针; 容器自身可能 NULL (该 priv 没 eager-allocate 容器).
     * a_01_8 现状:
     *   [PRIV_S] 容器 eager 分配 (cpu_create), 永远非 NULL
     *   [PRIV_H] 容器 a_01_8 不分配, 永远 NULL (留 H 扩展激活时分配)
     *   [PRIV_M] / [PRIV_U] 见 D13 (此处不动)
     * 索引 tlb_table[priv][asid] 前必须 check 容器非 NULL — 否则 NULL[asid] 是 host 段错
     * (a_01_8 v01 fixture (g) 跑 sfence.vma 时 ASan 抓到本 bug). */

    if (single_asid_mode) {
        /* (c) 单 ASID 清: WARL 截 asid 到 ASID_MASK 位 (跟 csr_satp_write 同, host 内存安全) */
        uint32_t asid = asid_val & ASID_MASK;
        if (hart->tlb_table[PRIV_S] != NULL) {
            tlb_clear(hart->tlb_table[PRIV_S][asid]);     /* 主目标 */
        }
        if (hart->tlb_table[PRIV_H] != NULL) {
            tlb_clear(hart->tlb_table[PRIV_H][asid]);     /* a_01_8 不到这, H 扩展时激活 */
        }
    } else {
        /* (a)/(b)/(d) 全清: 遍历所有 ASID 槽位调 tlb_clear (tlb_clear(NULL) no-op) */
        if (hart->tlb_table[PRIV_S] != NULL) {
            for (uint32_t asid = 0; asid < ASID_MAX; asid++) {
                tlb_clear(hart->tlb_table[PRIV_S][asid]);
            }
        }
        if (hart->tlb_table[PRIV_H] != NULL) {
            for (uint32_t asid = 0; asid < ASID_MAX; asid++) {
                tlb_clear(hart->tlb_table[PRIV_H][asid]);
            }
        }
    }
}
