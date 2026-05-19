//
// Created by liujilan on 2026/4/29.
// tlb 模块对外接口。
//
// ============================================================================
// tlb_table[4] 语义约定 (cpu_t 持有, 见 cpu.h)
// ============================================================================
//
// cpu_t 内的 `tlb_t **tlb_table[4]` 是 4 槽派发数组, index 按 RV privilege encoding:
//
//   [0] = U  : 始终是副本语义 (副本于 [1] S 或 [3] M, 取决于 misa)。副本分配两路:
//                - 初始化: cpu_create 按 misa 派发 (MSU 副本 [1]; MU-only 副本 [3])
//                - 运行时: H 扩展 (VS / VU 切换) 由对应 csr_helper 维护 mirror
//              当前默认 MSU, 副本 [1] (U 与 S 共享 ASID 命名空间)。
//
//   [1] = S  : ASID 数组容器 (tlb_t **)。容器由 cpu_create eager 分配, entries 由 walker
//              在首次访问该 ASID 时懒分配 (调 tlb_alloc 写回)。
//
//   [2] = VS : 初版 NULL (无 H 扩展); 未来 H 扩展激活时同 [1] 形态。
//
//   [3] = M  : 永远 NULL。Trust regime (M-mode 或任何 priv 带 bare satp) 直接走 identity
//              + IS_GPA_RAM 检查, 不需要 TLB; real CPU bare 下也 bypass MMU/TLB, 我们对齐。
//
// 物理元素类型在 cpu_t 中是 tlb_t ** (4 槽对称, 类型统一; 不需要 cast)。S 实际指向 ASID
// 容器, U 副本同样指针; M 永远 NULL (Trust regime 不查 TLB); V 当前 NULL, 是 H 扩展接口
// (激活时同 [1] S 形态)。
//
// dispatcher 派发逻辑 (见 dispatcher.c block 1):
//
//   if (priv == M || xatp.mode == bare) {
//       regime = REGIME_BARE;  current_tlb = NULL;        // Trust 不查 TLB
//   } else {
//       regime = REGIME_SV32;
//       current_tlb = cpu->tlb_table[priv][xatp.ASID];
//       if (current_tlb == NULL) {                        // 仅 S/V 槽会发生
//           current_tlb = tlb_alloc();
//           cpu->tlb_table[priv][xatp.ASID] = current_tlb;
//       }
//   }
//   /* 把 current_tlb 通过固定 host 寄存器 (JIT) / 函数参数 (interpreter) 传给 block */
//
// 懒分配只在 S/V 的 ASID entries 上发生。
//
// 详见 dummy.txt §4 (TLB 作为 block 入口的统一分发机制) + dummy.txt §3 (satp ASID 合法性契约)。
//
// ============================================================================
// fast path 查询序列 (JIT inline / interpreter 自己写, 不调函数)
// ============================================================================
//
//   index = (gva >> 12) & (TLB_NUM_ENTRIES - 1);
//   if (tlb->e[index].gva_tag == (gva >> 12)) {
//       if ((tlb->e[index].pte_flags & required_perm) == required_perm) {
//           /* fast path 通过 → 直接 host_ptr 访问 */
//       } else {
//           /* 走 helper (权限错) */
//       }
//   } else {
//       /* 走 helper (miss) */
//   }
//
// V 位 (bit 0 of pte_flags) = 0 即 invalid; tlb_alloc 内部 calloc 全 0 → 全 invalid,
// 第一次 fast path 查询自然 miss 走 walker 路径 fill。
//
// 不暴露 tlb_lookup / tlb_insert / tlb_free:
//   - lookup: 命中由调用方 inline (上方 fast path 伪码), 不调函数
//   - insert: 由 walker helper 直接写 TLB 数组, 不暴露独立函数
//   - free:   调用方直接用标准 free()。两个原因合一:
//             (a) tlb_alloc 内部用 aligned_alloc, 返回的内存按 C11 / POSIX 规定
//                 可直接用 free 释放
//             (b) free(NULL) 是 no-op (C 标准保证), 未懒分配的槽位天然无操作
//             因此 tlb_free 与 free() 等价, 不带额外语义, 不另立。
//             当前唯一调用点是 cpu_destroy (释放 ASID 数组容器中各 entries 时);
//             运行时 TLB 失效走 tlb_clear (memset 保留分配), 不走 free。
//

#ifndef CORE_TLB_H
#define CORE_TLB_H

#include <stdint.h>
#include "config.h"

// forward typedef cpu_t — tlb_table_reset 签名要 cpu_t*, 但 cpu.h 反过来
// #include "tlb.h", 不能反向 include 形成循环。跟 trap.h 同形态; 见 trap.h
// 顶段 doc。tlb.c 内 #include "cpu.h" 走完整定义。
typedef struct cpu_s cpu_t;

// 单条 TLB entry, 16 字节, 布局照 RV PTE 排位。
//
//   gva_tag  : 命中比较用 (gva >> 12), 低 20 位有效。
//   pte_flags: bit0=V bit1=R bit2=W bit3=X bit4=U bit5=G bit6=A bit7=D bit8-9=RSW
//              位置完全对齐 RV PTE, walker 填表时直接拷过来。V=0 即 invalid。
//   _pad     : 16B 对齐天然填充, 保留给未来扩展。
//   host_ptr : 指向该 guest page 起点的 host 地址, walker miss 路径填表时一次算好,
//              fast path 命中后无地址算术。
typedef struct {
    uint32_t  gva_tag;
    uint16_t  pte_flags;
    uint16_t  _pad;
    uint8_t  *host_ptr;
} tlb_e_t;

// 一套叶 TLB = TLB_NUM_ENTRIES 个 entry 的连续数组 (direct-mapped)。
// wrapper struct 是纯命名 —— 让 `tlb_t *current_tlb` 不被误读为"指向单条 entry"。
typedef struct {
    tlb_e_t e[TLB_NUM_ENTRIES];
} tlb_t;

// 分配一套 cache-line 对齐的叶 TLB。内容初始化为全 0 (V 位全 0 → 全 invalid)。
// 失败返回 NULL (内部已 fprintf), 调用方按 dummy.txt §5 错误风格处理。
//
// 调用方:
//   - dispatcher:   懒分配 S 槽 [priv][asid] entries (Sv32 路径首次访问该 ASID 时)
//   - 未来 H 扩展激活时, [VS] 槽走同样的懒分配路径
tlb_t *tlb_alloc(void);

// 清空一套叶 TLB (memset 全 0, 保留分配)。
// NULL 入参 do nothing —— 简化 sfence helper, 不需写 if (tlb_table[PRIV_S][i]) ...
//
// 调用方:
//   - sfence.vma helper: 清 [1][asid] 的 entries (具体 ASID 选择见 sfence.c 4 组合分流)
//   - tlb_table_reset:    system reset 路径全清各 ASID entries (本头下方)
void tlb_clear(tlb_t *tlb);

// 整 hart 级 TLB 全清 (system reset 时由 cpu_reset 调)。遍历 [PRIV_S] 容器
// 的全部 ASID entries 调 tlb_clear; [PRIV_H] 容器也走同形态遍历 (当前永远
// NULL, 循环天然 no-op, 写出来未来 H 扩展不漏)。
//
// 不走的槽:
//   [PRIV_M] 永远 NULL (Trust regime 不查 TLB)
//   [PRIV_U] alias [PRIV_S] (副本语义, 跟 S 共享 entries, 不重复清)
//
// 容器本身 (tlb_t ** ASID 数组) 不释放, 保留给后续 dispatcher 懒分配复用 —
// 跟 sfence 形态一致 (entries 内容清 / 容器不动)。
void tlb_table_reset(cpu_t *hart);

#endif //CORE_TLB_H
