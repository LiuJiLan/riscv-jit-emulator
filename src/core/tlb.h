//
// Created by liujilan on 2026/4/29.
// a01_2 tlb 模块对外接口。
//
// ============================================================================
// tlb_table[4] 语义约定 (cpu_t 持有, 见 cpu.h)
// ============================================================================
//
// cpu_t 内的 `tlb_t **tlb_table[4]` 是 4 槽派发数组, index 按 RV privilege encoding:
//
//   [0] = U  : ASID 数组容器 (tlb_t **). MSU 默认下别名 [1] (U 共享 S 的 ASID 命名空间);
//              MU-only 不对称核 (类比 SiFive U74 M-only) 下应别名 [3]。由 cpu_create
//              按 misa 派发, 由 mstatus / hstatus 写 helper 在 xstatus 变动时维护切换。
//
//   [1] = S  : ASID 数组容器 (tlb_t **), entries 由 dispatcher 在首次访问该 ASID 时
//              懒分配 (调 tlb_alloc 写回)。
//
//   [2] = VS : 初版 NULL (无 H 扩展); 未来 H 扩展激活时同 [1] 形态。
//
//   [3] = M  : ASID 数组容器 (tlb_t **), 16 槽全部别名同一份共享 leaf (M 模式无 ASID
//              概念, 用 16 路别名让 dispatcher 用统一的 [select][asid] 索引)。
//
// 物理元素类型在 cpu_t 中是 tlb_t **(v3 完全对称设计后, 4 槽都是 ASID 数组容器,
// 类型统一)。早期版本曾因"M 槽是单 leaf, S/V 是 ASID 数组"的类型不对称用 void *
// 存储, v3 收敛后不再需要 cast。
//
// dispatcher 派发逻辑 (未来; a_01 仍 #if 0):
//
//   if (xatp.mode == bare)  select = PRIV_M;   /* M 槽 (16 路别名同一 leaf); 实际位运算 */
//   else                    select = priv;
//   leaf = cpu->tlb_table[select][xatp.ASID];
//   if (leaf == NULL) {                  /* 仅 S/V 槽会发生 */
//       leaf = tlb_alloc();
//       cpu->tlb_table[select][xatp.ASID] = leaf;
//   }
//   /* 把 leaf 通过固定 host 寄存器 (JIT) / 函数参数 (interpreter) 传给 block */
//
// M 槽永不 NULL (cpu_create eager 分配 + 16 路别名), 懒分配只在 S/V 的 ASID entries 上发生。
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
//             a_01 唯一调用点是 cpu_destroy (释放 ASID 数组容器中各 entries 时);
//             运行时 TLB 失效走 tlb_clear (memset 保留分配), 不走 free。
//

#ifndef CORE_TLB_H
#define CORE_TLB_H

#include <stdint.h>
#include "config.h"

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
//   - cpu_create:   eager 分配 M 槽 [3] 的共享 leaf
//   - dispatcher:   懒分配 S/V 槽 [priv][asid] entries (a_01 仍 #if 0)
tlb_t *tlb_alloc(void);

// 清空一套叶 TLB (memset 全 0, 保留分配)。
// NULL 入参 do nothing —— 简化 sfence helper, 不需写 if (tlb_table[PRIV_S][i]) ...
//
// 调用方:
//   - sfence.vma helper (未来): 清 [1][asid] 的 entries, 见 file_plan.md §2.tlb E 区
void tlb_clear(tlb_t *tlb);

#endif //CORE_TLB_H
