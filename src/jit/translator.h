//
// Created by liujilan on 2026/6/19.
// jit/translator.h —— RV → IR 翻译器 (T1 c+ 范围: 真翻译 ADD/ADDI 子集 + 截断走
// interpret).
//
// 三层架构 (plan §1.21 Dispatcher / Translator / JitBackend):
//   Translator 知道 RISC-V, 不知道 host  → 产出 IR (后端无关)
//   JitBackend 知道 host, 不知道 RISC-V  → 消费 IR
//
// 接口 (T1 c+; T2-T4 真翻译扩时签名按需调):
//   translator_translate(pa, regime, ir_buf, n_insts) — 输入块入口 (pa, regime),
//     输出 ir_buf[N + 1] 包含 N 条 IR_OP_ADD / IR_OP_ADDI 前缀 + 1 条
//     IR_OP_DISPATCH_EXIT 收尾. *n_insts 写真实长度 (N + 1).
//
// 调用方:
//   src/jit/jit_entry.cc jit_compile_block 内部, 调 translator_translate 拿 IR 流,
//   再 forward 给 backend.compile_block. translator 不调 backend (跟 plan §1.23.17
//   一致: Translator 不直接调 backend).
//
// caller 保证 ir_buf 大小 ≥ BLOCK_INST_LIMIT (jit_compile_block 内部 stack 分配
// BLOCK_INST_LIMIT 大小 ir_buf).
//
// 跨页边界处理 (跟 interpreter 同体例): 块前缀推进后 cur_pc 进新 page → 截断.
// 软边界 BLOCK_INST_LIMIT - 1 (留 1 slot 给 DISPATCH_EXIT 收尾).
//
// 块前缀空 (N = 0, 第一条就是非 ADD/ADDI): translator 仍 emit DISPATCH_EXIT 一条
// (target_pc = 块入口 pa), *n_insts = 1; backend.compile_block 检测后返
// NOT_IMPLEMENTED, jit_entry.cc set_blacklist + dispatcher 兜底走 interpret.
//

#ifndef JIT_TRANSLATOR_H
#define JIT_TRANSLATOR_H

#include <stddef.h>
#include "core/mmu.h"   // regime_t (caller 传, T1 c+ 不用; T3 翻译 LOAD/STORE 时接 SUM/MXR check 形态)
#include "ir.h"
#include "riscv.h"      // uxlen_t (pa 参数; dummy.txt §13 typedef family)

#ifdef __cplusplus
extern "C" {
#endif

// translator_translate —— 翻译块入口 (pa, regime) 处的 RV 指令流为 IR 流.
//
// 参数:
//   pa       - 块入口 PA (uxlen_t)
//   regime   - baked priv 视角 (REGIME_BARE / REGIME_SV32_S / REGIME_SV32_U);
//              T1 c+ 不用 (T3 翻译 LOAD/STORE 时接 SUM/MXR check 形态)
//   ir_buf   - 出参 IR buffer (大小 ≥ BLOCK_INST_LIMIT)
//   n_insts  - 出参 IR 流真实长度 (含末尾 DISPATCH_EXIT)
//
// 前置: pa 在 RAM 区 (IS_GPA_RAM(pa) == 1); caller (jit_compile_block) 保证.
//
// 返 void; 块前缀空时 *n_insts = 1 (仅 DISPATCH_EXIT), backend 端兜底返
// NOT_IMPLEMENTED.
void translator_translate(uxlen_t pa, regime_t regime,
                          ir_inst_t *ir_buf, size_t *n_insts);

#ifdef __cplusplus
}
#endif

#endif //JIT_TRANSLATOR_H
