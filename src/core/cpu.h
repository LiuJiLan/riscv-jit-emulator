//
// Created by liujilan on 2026/4/28.
// a01_2 cpu 模块对外接口。
//
// cpu_t = 单 hart 的 guest CPU 状态镜像 (per-hart, 无并发, 普通读写)。
//
// 字段布局规约 (与 dummy.txt §2 协议一致):
//   regs[32], 单一连续数组
//     - regs[0] 物理上占 x0 的位置, 但实际存 pc
//     - regs[1..31] = x1..x31
//     - x0 由 translator / interpreter 特殊路径处理 (读 = 字面量 0, 写 = 丢弃),
//       永远不真碰 regs[0]
//   priv / satp / jmp_buf_ptr / tlb_table[4]
//
// 这种 regs[32] 而非 pc + regs[31] 的布局是有意设计:
//   - decoded rd 是 5-bit 数字, 可以直接 regs[rd] 索引, 不需要 rd-1 减法
//   - JIT emit 出的 host 代码计算 cpu_base + N*4 直接 load/store, 翻译规则统一
//   - dummy.txt §2 文字描述的"pc 在偏移 0"在物理上就是 regs[0]
// 伪代码 / 注释中的 hart->pc / cpu->pc 是表意, 实际访问点请用 regs[0]。
//
// jmp_buf_ptr 实体在 dispatcher 栈, cpu_t 只持指针, 见 dummy.txt §1。
// tlb_table[4] 4 槽语义见 tlb.h 顶部注释 (v3 对称设计下元素类型统一为 tlb_t **,
//   ASID 数组容器, 直接索引 [asid] 拿 tlb_t *, 不需要 cast)。
//
// 未来扩展 (a01_2 不含):
//   - isa/fpu 进来时加 fcsr 指针 (POD 外挂, 不嵌入主结构)
//   - monitor / 调试需要时加 perf_counters 指针
//   - trap / CSR 状态字段 (mstatus / mip / mcause / mtval / mepc / mtvec 等)
//
// 报错风格见 src/dummy.txt §5。
//

#ifndef CORE_CPU_H
#define CORE_CPU_H

#include <setjmp.h>
#include <stdint.h>

#include "tlb.h"   // tlb_t * 类型 (tlb_table 元素是 tlb_t **)

typedef struct {
    // _Alignas(64) 在第一字段, 强制整个 struct 以 64B (cache line) 对齐。
    // cpu_create 用 aligned_alloc(64, sizeof(cpu_t)) 分配, 与之配套。
    //
    // regs[0] 实际是 pc (物理占 x0 位置, x0 走特殊路径不碰 regs[0])
    // regs[1..31] = x1..x31, offset(reg N) = N * 4
    _Alignas(64) uint32_t regs[32];
    uint8_t               priv;             // RV privilege encoding (riscv.h PRIV_*); a_01 = PRIV_M
    uint32_t              satp;             // Sv32 satp; a_01 = 0 (bare; MODE=0, ASID=0, PPN=0)
    sigjmp_buf           *jmp_buf_ptr;      // 实体在 dispatcher 栈, 见 dummy.txt §1
    tlb_t               **tlb_table[4];     // 4 槽派发数组, 语义见 tlb.h 顶部
} cpu_t;

// 工厂: 分配 (cache-line 对齐) + 初始化 cpu_t。
// 失败返回 NULL, 内部已 fprintf。
//
// misa 参数 a_01 内部不读 —— 仅作未来 misa 驱动初始化预留。
// 未来用例:
//   - MU-only 不对称核 (类比 SiFive U74 M-only): tlb_table[PRIV_U] 改为别名 [PRIV_M], [PRIV_S] 不分配
//   - F/D/V 扩展进来时按 misa.fdv 决定是否分配 fcsr / vcsr 子结构
cpu_t *cpu_create(uint32_t misa);

// 释放 cpu_create 分配的 cpu_t 及其下所有子结构 (tlb 容器 + 共享 leaf + 已懒分配的 entries)。
// NULL 入参 do nothing。
void cpu_destroy(cpu_t *hart);

#endif //CORE_CPU_H
