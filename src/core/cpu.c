//
// Created by liujilan on 2026/4/28.
// a01_2 cpu 模块实现 (cpu_create / cpu_destroy)。
//
// tlb_table[4] 完全对称设计 (v3): 所有 4 槽都是 ASID 数组容器 (tlb_t **)。
// M 槽 [3] 的"无 ASID"语义靠"16 个槽全部别名同一份 leaf"实现。
// 详见 tlb.h 顶部 + notes/context/a_01_session_005.md "A4 第二问"。
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

    cpu_t *cpu = aligned_alloc(64, sizeof(cpu_t));
    if (cpu == NULL) {
        fprintf(stderr, "cpu_create: aligned_alloc(64, %zu) failed: %s\n",
                sizeof(cpu_t), strerror(errno));
        return NULL;
    }
    memset(cpu, 0, sizeof(*cpu));

    cpu->priv = PRIV_M;    // a_01: M 模式常量
    cpu->satp = 0;         // a_01: bare 模式 (MODE=0, ASID=0, PPN=0 全 0)

    // ------------------------------------------------------------------------
    // tlb_table[4] 装载 (MSU 默认 misa)。
    //
    // 未来 misa 驱动: 这里会按 misa 分支:
    //   - MU-only: [3] 同下; [0] = [3] (类型已统一, 赋值无障碍); [1] 不分配 (= NULL)
    //   - 含 H 扩展: [2] 也按 [1] 形态 calloc
    // dispatcher 派发逻辑不需感知 (靠 tlb_table[select][asid] 通用路径)。
    //
    // memset 0 已经把 tlb_table[0..3] 置 NULL, 失败回滚路径上 free(NULL) 无害。
    // ------------------------------------------------------------------------

    // [PRIV_M] M/bare: ASID 容器 + 一份共享 leaf, 16 槽全部别名 (M 模式无 ASID)。
    cpu->tlb_table[PRIV_M] = calloc(ASID_MAX, sizeof(tlb_t *));
    if (cpu->tlb_table[PRIV_M] == NULL) {
        fprintf(stderr, "cpu_create: calloc tlb_table[PRIV_M] failed: %s\n", strerror(errno));
        cpu_destroy(cpu);
        return NULL;
    }
    tlb_t *m_leaf = tlb_alloc();           // tlb_alloc 内部失败已 fprintf
    if (m_leaf == NULL) {
        cpu_destroy(cpu);
        return NULL;
    }
    for (uint32_t i = 0; i < ASID_MAX; i++) {
        cpu->tlb_table[PRIV_M][i] = m_leaf;
    }

    // [PRIV_S] S: ASID 容器, entries 由 dispatcher 在首次访问该 ASID 时懒分配 (a_01 #if 0 不触发)。
    // 注释: 假设 misa 含 S 扩展; 未来 MU-only 时本字段应为 NULL, [PRIV_U] 改为别名 [PRIV_M]。
    cpu->tlb_table[PRIV_S] = calloc(ASID_MAX, sizeof(tlb_t *));
    if (cpu->tlb_table[PRIV_S] == NULL) {
        fprintf(stderr, "cpu_create: calloc tlb_table[PRIV_S] failed: %s\n", strerror(errno));
        cpu_destroy(cpu);
        return NULL;
    }

    // [PRIV_U] U: MSU 默认下 mirror S 的 ASID 容器 (同一指针)。
    // 真正的"当前 ASID 的 mirror 切换"由 mstatus / hstatus 写 helper 维护, 这里只是初值。
    cpu->tlb_table[PRIV_U] = cpu->tlb_table[PRIV_S];

    // [PRIV_H] VS 占位: NULL (memset 0 已经保证), 未来 H 扩展激活时 calloc。
    //          (PRIV_H 是项目内部占位宏, 见 riscv.h 的警告注释)

    return cpu;
}

void cpu_destroy(cpu_t *cpu) {
    if (cpu == NULL) return;

    // [PRIV_M] M: 16 路别名同一 leaf, 只 free [PRIV_M][0] 一次, 然后 free 容器。
    if (cpu->tlb_table[PRIV_M] != NULL) {
        free(cpu->tlb_table[PRIV_M][0]);
        free(cpu->tlb_table[PRIV_M]);
    }

    // [PRIV_S] S: 递归 free 非 NULL entries, 然后 free 容器。
    if (cpu->tlb_table[PRIV_S] != NULL) {
        for (uint32_t i = 0; i < ASID_MAX; i++) {
            free(cpu->tlb_table[PRIV_S][i]);    // free(NULL) 无害, 未懒分配的槽位天然 no-op
        }
        free(cpu->tlb_table[PRIV_S]);
    }

    // [PRIV_H] V 占位: 同 [PRIV_S] 模式; a_01 始终 NULL, 循环天然 no-op, 但写出来未来 H 扩展不漏。
    if (cpu->tlb_table[PRIV_H] != NULL) {
        for (uint32_t i = 0; i < ASID_MAX; i++) {
            free(cpu->tlb_table[PRIV_H][i]);
        }
        free(cpu->tlb_table[PRIV_H]);
    }

    // [PRIV_U] U: alias [PRIV_S] (MSU) 或 alias [PRIV_M] (未来 MU-only), 均不再 free。

    free(cpu);
}
