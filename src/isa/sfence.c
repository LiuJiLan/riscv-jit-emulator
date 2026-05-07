//
// Created by liujilan on 2026/5/7.
// a_01_8 isa/sfence —— sfence_vma_helper 实现 (extern, slow path)。
//
// 顶部接口 doc + RV spec 四组合 vs 简化方案 4.a 对照 + 涉及哪些 tlb_table[] 槽 见 sfence.h。
//

#include "sfence.h"

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

    if (single_asid_mode) {
        /* (c) 单 ASID 清:
         *   - asid 来自 caller 传的 asid_val (= READ_REG(rs2)); rs2!=0 时是真值, rs2==0
         *     时 caller 传 0 (但本分支已判 rs2!=0, 不可达) — 接口契约让代码自洽。
         *   - WARL 截断到 ASID_MASK 位 (跟 csr.c csr_satp_write 路径同); host 内存安全防线
         *     (满足 dummy.txt §3 satp 合法性契约的精神 — 无论 satp 还是 sfence 路径, 进
         *     tlb_table[][asid] 索引前 asid 必须合法)。
         */
        uint32_t asid = asid_val & ASID_MASK;
        tlb_clear(hart->tlb_table[PRIV_S][asid]);     /* 主目标 (ASID 容器内对应槽) */
        tlb_clear(hart->tlb_table[PRIV_H][asid]);     /* a_01_8 永远 NULL → no-op;
                                                         留写法方便未来 H 扩展激活 */
    } else {
        /* (a)/(b)/(d) 全清: 遍历所有 ASID 槽位调 tlb_clear。
         *   - tlb_clear(NULL) 是 no-op (tlb.h 设计), 没懒分配的槽位天然跳过。
         *   - [PRIV_U] 是 [PRIV_S] 别名 (cpu.c eager 设), 不重复清。
         *   - [PRIV_M] D23 路线永远 NULL, sfence.vma 按 RV spec 也不影响 M-mode TLB, 不动。
         */
        for (uint32_t asid = 0; asid < ASID_MAX; asid++) {
            tlb_clear(hart->tlb_table[PRIV_S][asid]);
            tlb_clear(hart->tlb_table[PRIV_H][asid]);
        }
    }
}
