//
// Created by liujilan on 2026/4/28.
// cpu 模块实现 (cpu_create / cpu_destroy)。
//
// tlb_table[4] 设计:
//   - REGIME_BARE 走 identity bypass TLB → [PRIV_M] 槽永远 NULL, 不分配
//   - REGIME_SV32 走 [priv][asid] 二级索引 → [PRIV_S] 容器 eager 分配, entries 懒 (walker 填)
//   - [PRIV_U] alias [PRIV_S] (MSU 默认; U 共享 S 的 ASID 命名空间)
//   - [PRIV_H] VS 占位, NULL (未来 H 扩展)
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

// cpu_info_shared_default — 多 hart 共享出场信息 RO CSR (cpu_t::shared_info 指向)
//
// 制造商信息 (mvendorid/marchid/mimpid) 是机器整体属性, 不区分 hart; 当前 open-source
// 项目全 0 (RV spec: 0 = no JEDEC vendor / no architecture ID / no impl ID, 合法值)。
// 异构 SMP 时这里加多份 cpu_info_shared_default_a/b 给不同 hart 组; 项目当前同构, 单份。
//
// 注: misa 不在这里 (per-hart 私有, 进 cpu_info_per_hart_t — 异构 SMP 时不同 hart 的扩展
// 字段不同; mhartid 同理)。
static const cpu_info_shared_t cpu_info_shared_default = {
    .mvendorid = 0,
    .marchid   = 0,
    .mimpid    = 0,
};

cpu_t *cpu_create(uint32_t misa, uint32_t mhartid) {
    // init 依赖 misa (future) — 当前只把 misa 值存入 per_hart_info.misa 给
    // csr_misa_read 用, 不按 misa 真做派发 (F/D 扩展按 misa.fdv 决定 fcsr alloc /
    // H 扩展按 misa.h 决定 [PRIV_H] tlb 容器 alloc 等都属于未来)。
    cpu_t *hart = aligned_alloc(64, sizeof(cpu_t));
    if (hart == NULL) {
        fprintf(stderr, "cpu_create: aligned_alloc(64, %zu) failed: %s\n",
                sizeof(cpu_t), strerror(errno));
        return NULL;
    }
    memset(hart, 0, sizeof(*hart));

    // per-hart 私有 RO CSR 数据 (mhartid + misa); cpu_create 入参直接写入
    // cpu_t.per_hart_info 字段。misa 入参当前仅作为 csr_misa_read 返回值 (不按 misa 派发
    // lifecycle, 例如 F/D 扩展按 misa.fdv 决定 fcsr alloc — 那是未来)。
    hart->per_hart_info.mhartid = mhartid;
    hart->per_hart_info.misa    = misa;

    // 多 hart 共享 RO CSR 数据 (mvendorid/marchid/mimpid); 指针指向全局 static const default。
    hart->shared_info = &cpu_info_shared_default;

    // ------------------------------------------------------------------------
    // RV "硬件 reset 后默认状态" 写入 (regs / priv / satp; 原 main.c hart 字段
    // 初始化段 L314~L321 挪入)。跟 cpu_reset 内的写入序列一致 — POR 一次性 init
    // 等同于 "刚加电 + 第一次 reset" 后的 hart 状态。详 cpu_reset doc 段。
    //
    // 启动状态参考 https://docs.kernel.org/arch/riscv/boot.html
    // ------------------------------------------------------------------------
    hart->regs[0]   = GUEST_RAM_START;      // pc = 程序起点 (reset vector)
    hart->regs[10]  = mhartid;              // a0 = hartid (Linux RV boot 协议)
    // regs[11] = 0 (a1 = dtb 占位, 未来; memset 已置 0)
    hart->priv      = PRIV_M;               // M 模式 (启动)
    hart->satp      = 0;                    // bare 模式 (MODE=0, ASID=0, PPN=0 全 0)

    // ------------------------------------------------------------------------
    // tlb_table[4] 装载 (Trust regime bypass TLB; 当前默认 MSU)
    //
    // [PRIV_M] M/bare 槽: 永远 NULL。Trust regime (M-mode 或任何 priv 带 bare satp) 直接
    //                     走 identity + IS_GPA_RAM 检查, 不需要 TLB; real CPU bare 下也
    //                     bypass MMU/TLB, 我们对齐。memset 0 已置 NULL, 不需显式赋值。
    //
    // [PRIV_S] S 槽: ASID 容器, entries 由 walker 在 Sv32 路径懒分配。
    //
    // [PRIV_U] U 槽: 始终是副本语义 (U 是 S 或 M 的副本, 取决于 misa 实际配置)。即使
    //                MU-only CPU 中 U 副本 M 态、槽内 NULL (M 走 bare 不查 TLB), "副本"
    //                语义本身不变。副本分配两路:
    //                  - 初始化: cpu_create 负责 (未来根据 misa 派发: MSU 副本 S; MU-only
    //                            副本 M, 即 NULL)
    //                  - 运行时: H 扩展激活 (VS / VU 等切换) 由对应 csr_helper 维护 mirror
    //                当前默认 MSU, 直接赋值 S 槽 (U 与 S 共享 ASID 命名空间)。
    //
    // [PRIV_H] VS 占位: NULL (memset 0 已置), 未来 H 扩展激活时 calloc。
    //
    // memset 0 已经把 tlb_table[0..3] 置 NULL, 失败回滚路径上 free(NULL) 无害。
    // ------------------------------------------------------------------------

    // [PRIV_S] S: ASID 容器, entries 由 walker 在 Sv32 路径懒分配。
    hart->tlb_table[PRIV_S] = calloc(ASID_MAX, sizeof(tlb_t *));
    if (hart->tlb_table[PRIV_S] == NULL) {
        fprintf(stderr, "cpu_create: calloc tlb_table[PRIV_S] failed: %s\n", strerror(errno));
        cpu_destroy(hart);
        return NULL;
    }

    // [PRIV_U] U: alias [PRIV_S] (MSU 默认; U 共享 S 的 ASID 命名空间)。
    hart->tlb_table[PRIV_U] = hart->tlb_table[PRIV_S];

    // [PRIV_M] / [PRIV_H]: 不分配, memset 0 保证 NULL。

    return hart;
}

void cpu_reset(cpu_t *hart) {
    // reset 依赖 misa (future) — 真硬件 reset 后哪些字段清 / 哪些保留按 misa 决定
    // (例 misa.F=0 时 fcsr 不存在不需清; misa.H=0 时 [PRIV_H] tlb 容器不存在不需
    // tlb_table_reset 内遍历)。当前不实装运行时 misa 切换, 全 hart 同 reset 序。
    if (hart == NULL) return;

    // regs: 全清 0, 然后写 RV-spec reset 后的启动协议字段。
    // (顺序: 先 memset, 再写需要非 0 的 — regs[0]=pc / regs[10]=a0)
    memset(hart->regs, 0, sizeof(hart->regs));
    hart->regs[0]  = GUEST_RAM_START;            // pc = reset vector
    hart->regs[10] = hart->per_hart_info.mhartid; // a0 = hartid (Linux RV boot 协议)
    // regs[11] = 0  (a1 = dtb 占位, memset 已 0)

    // 控制状态: priv = M / satp = 0 (bare)
    hart->priv = PRIV_M;
    hart->satp = 0;

    // trap_csrs_t 全 memset 0 — xcause/xtval/xepc/xtvec/xscratch 各 [4] +
    // _mstatus/_medeleg/mideleg/_mie/_mip_sw/in_trap 等所有字段一次清。
    // jmp_buf_ptr / per_hart_info / shared_info 不在 trap 内, 不影响。
    memset(&hart->trap, 0, sizeof(hart->trap));

    // tlb_table 容器不动 (跟 sfence 形态一致); entries 全清 (跨地址空间防 PTE
    // 残留撞 hash 命中)。详 tlb.h tlb_table_reset doc 段。
    tlb_table_reset(hart);

    // 保留 (硬件 ID 类, 真硬件 reset 后不变):
    //   per_hart_info (mhartid / misa); shared_info 指针 (mvendorid 等)
    //   jmp_buf_ptr (dispatcher 进入时重设永久落点, reset 时不动也无害)
}

void cpu_destroy(cpu_t *hart) {
    if (hart == NULL) return;

    // [PRIV_S] S: 递归 free 非 NULL entries (walker 懒分配的叶 TLB), 然后 free 容器。
    if (hart->tlb_table[PRIV_S] != NULL) {
        for (uint32_t i = 0; i < ASID_MAX; i++) {
            free(hart->tlb_table[PRIV_S][i]);    // free(NULL) 无害, 未懒分配的槽位天然 no-op
        }
        free(hart->tlb_table[PRIV_S]);
    }

    // [PRIV_H] V 占位: 同 [PRIV_S] 模式; 当前始终 NULL, 循环天然 no-op, 写出来未来 H 扩展不漏。
    if (hart->tlb_table[PRIV_H] != NULL) {
        for (uint32_t i = 0; i < ASID_MAX; i++) {
            free(hart->tlb_table[PRIV_H][i]);
        }
        free(hart->tlb_table[PRIV_H]);
    }

    // [PRIV_U] U: 副本语义, 跟所副本的槽 (S 或 M) 共享内存, 不在这里 free。
    // [PRIV_M] M: 不分配 (Trust regime 不走 TLB), 不再 free。

    free(hart);
}
