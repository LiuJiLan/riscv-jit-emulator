//
// Created by liujilan on 2026/6/5.
// api/helpers.h —— slow path helper 集中声明 (C 实现, 给 C++ backend 调).
//
// ============================================================================
// 跨语言边界 + namespace 角色
// ============================================================================
//
// 本头是 src/api/ 跨语言边界的一边 (跟 api/jit_api.h 对偶):
//   jit_api.h     C++ 实现 (jit/backend_asmjit.cc 等), 给 C 调 (dispatcher 等)
//   helpers.h     C 实现, 给 C++ 调 (backend emit slow path call 时用)
//
// 设计约束 (CLAUDE.md "Module boundaries"):
//   - api/ 头必须 C-compilable: 不用 class / template / namespace / std:: 类型
//   - C/C++ 边界 = extern "C" 包整段 + POD struct + 函数指针
//
// ============================================================================
// 集中声明的 helper 集合 (re-export, 不增新逻辑)
// ============================================================================
//
// 本头不真重写 helper 声明, 而是 include 既有头转发 (避免维护两份签名漂移).
// 转发集 = JIT translator emit `call <helper>` 时所有可能用到的 slow path
// helper 入口 (dummy.txt §10 "helper 颗粒度 by design"):
//
//   core/mmu.h    — mmu_translate_pc / mmu_walk / mmu_walker_helper_load /
//                    store / amo_xxx (9) / lr_w / sc_w
//   isa/lsu.h     — lsu_load_helper / lsu_store_helper / store_helper
//   isa/lrsc.h    — lrsc_lr_helper / lrsc_sc_helper / lrsc_lr_w / lrsc_sc_w /
//                    lrsc_on_store / lrsc_on_device_write / lrsc_clear_self
//   isa/amo.h     — amo_xxx_helper (9) / amo_xxx_apply (9)
//   core/csr.h    — csr_op
//   isa/sfence.h  — sfence_vma_helper
//   isa/fence.h   — fence_helper / fence_i_helper
//   core/trap.h   — trap_raise_exception (helper longjmp 入口; dummy.txt §1)
//   core/wfi.h    — wfi_wait (可能 b_03+ JIT 翻译 WFI 时调; 先纳入)
//
// 当前 re-export 全套. backend 真做 emit (b_02) 时若需 namespace 精细控制
// (只 export 部分) 再拆 — 当前转发简单 + 维护友好.
//
// ============================================================================
// 命名
// ============================================================================
//
// 本头不加 c_ prefix — api/ 目录天然就是 C-compilable 跨语言头 (文件位置已表达),
// 函数 declaration 用 extern "C" 已显式声明 ABI; 不需要 prefix 再强调.
// 跟 jit_api.h 平行命名 (都不带额外 prefix; 一边 jit_* 函数 prefix, 一边 re-export
// 既有 helper 沿用各自模块 prefix mmu_walker_helper_* / lrsc_* / amo_* / ...).
//

#ifndef API_HELPERS_H
#define API_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

// 既有 helper 头转发 (re-export 全套).
// 顺序: core → isa (跟 src/ 目录组织一致).
#include "core/mmu.h"
#include "core/csr.h"
#include "core/trap.h"
#include "core/wfi.h"
#include "isa/lsu.h"
#include "isa/lrsc.h"
#include "isa/amo.h"
#include "isa/sfence.h"
#include "isa/fence.h"

#ifdef __cplusplus
}
#endif

#endif //API_HELPERS_H
