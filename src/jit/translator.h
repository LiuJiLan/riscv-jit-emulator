//
// jit/translator.h —— RV → IR 翻译器
//
// 三层架构 (plan §1.21 Dispatcher / Translator / JitBackend):
//   Translator 知道 RISC-V, 不知道 host  → 产出 IR (后端无关)
//   JitBackend 知道 host, 不知道 RISC-V  → 消费 IR
//
// 真翻译范围 (RV32 IMC 全集; 详 ir.h 顶段 op_kind 清单):
//   - RV32I 算术 21 op (R-type / I-type / U-type)
//   - LOAD/STORE 8 op (helper call; BARE/SV32 fast path 详 backend)
//   - CSR 6 op + FENCE/FENCE.I 2 op (硬边界; helper call)
//   - AMO 9 op + LR/SC 2 op (helper call; 详 isa/amo.h / isa/lrsc.h)
//   - 控制流 BRANCH 6 + JAL/JALR 2 + SYSTEM 6 (硬边界; 双出口/runtime target)
//   - RV32M 8 op (MUL 4 + DIV/REM 4; 详 backend emit_ir_muldiv)
//   - RVC: decode_rvc 折叠到等价 32-bit op_kind, d.pc_step 透传 (translator
//     零特殊处理; cur_pc += d.pc_step)
//
// 接口:
//   translator_translate(pa, regime, ir_buf, n_insts) — 输入块入口 (pa, regime),
//     输出 ir_buf 含 N 条真翻译 IR + 1 条出口模板 (DISPATCH_EXIT /
//     DISPATCH_EXIT_RUNTIME) 收尾. *n_insts 写真实长度 (N + 1).
//
// 调用方:
//   src/jit/jit_entry.cc jit_compile_block 内部, 调 translator_translate 拿 IR 流,
//   再 forward 给 backend.compile_block. translator 不调 backend (跟 plan §1.23.17
//   一致: Translator 不直接调 backend).
//
// caller 保证 ir_buf 大小 ≥ BLOCK_INST_LIMIT + 1 (jit_compile_block 内部 stack
// 分配 BLOCK_INST_LIMIT + 1 大小 ir_buf): 前 BLOCK_INST_LIMIT 槽给真翻译 RV inst
// (跟 interpreter.c:189 软边界字面对偶), 末槽给 DISPATCH_EXIT/_RUNTIME 哨兵.
//
// 跨页边界处理 (跟 interpreter 同体例): 块前缀推进后 cur_pc 进新 page → 截断.
// 软边界 BLOCK_INST_LIMIT 跟解释器字面一致 (b_04_session_004 对齐).
//
// 块前缀空 (N = 0, 第一条就是 unsupported op): translator 仍 emit 出口模板一条,
// *n_insts = 1; backend.compile_block 检测后返 NOT_IMPLEMENTED, jit_entry.cc
// set_blacklist + dispatcher 兜底走 interpret.
//

#ifndef JIT_TRANSLATOR_H
#define JIT_TRANSLATOR_H

#include <stddef.h>
#include "core/mmu.h"   // regime_t (caller 传; backend SV32 fast path 用)
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
//              translator 不消费 regime, 透传给 backend (LOAD/STORE fast path 用)
//   ir_buf   - 出参 IR buffer (大小 ≥ BLOCK_INST_LIMIT + 1; 末槽给出口模板)
//   n_insts  - 出参 IR 流真实长度 (含末尾 DISPATCH_EXIT)
//
// 前置: pa 在 RAM 区 (IS_GPA_RAM(pa) == 1); caller (jit_compile_block) 保证.
//
// 返 void; 块前缀空时 *n_insts = 1 (仅出口模板), backend 端兜底返
// NOT_IMPLEMENTED.
void translator_translate(uxlen_t pa, regime_t regime,
                          ir_inst_t *ir_buf, size_t *n_insts);

#ifdef __cplusplus
}
#endif

#endif //JIT_TRANSLATOR_H
