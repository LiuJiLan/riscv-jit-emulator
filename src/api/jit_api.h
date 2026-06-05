//
// Created by liujilan on 2026/6/5.
// api/jit_api.h —— JIT 子系统对外入口 (C++ 实现, 给 C 调).
//
// ============================================================================
// 跨语言边界
// ============================================================================
//
// 本头是 src/api/ 跨语言边界的一边 (跟 api/helpers.h 是对偶):
//   jit_api.h     C++ 实现 (jit/backend_asmjit.cc 等), 给 C 调 (dispatcher 等)
//   helpers.h     C 实现, 给 C++ 调 (backend emit slow path call 时用)
//
// 设计约束 (CLAUDE.md "Module boundaries"):
//   - api/ 头必须 C-compilable: 不用 class / template / namespace / std:: 类型
//   - C/C++ 边界用 extern "C" + POD struct + 函数指针
//   - api/ 不 include jit/ (单向; jit/ 可以 include api/)
//
// ============================================================================
// JIT 入口 (b_01 T1 阶段接口骨架, T2-T4 真做时填实现)
// ============================================================================
//
// T1 阶段函数仅声明, 不带实现 — backend / translator / code_cache / jit_cache
// 全 stub 状态 (T2 起 jit_cache 真做, T3 起 backend / translator 真做 stub
// 形态, T4 起 dispatcher fork 点真接).
//
// 错误码 jit_status_t (T1 阶段 enum 定义, 具体语义 T2/T3 真做时填):
//   JIT_OK                  — 编译成功 / 接口调用成功
//   JIT_CODE_CACHE_FULL     — code_cache mmap 区满, 调用方触发整体 Flush
//                              (plan §1.23.9)
//   JIT_IR_ERROR            — IR 内部不一致 / translator 产物错; 调用方进
//                              编译黑名单 (plan §1.23.8)
//   JIT_BACKEND_INTERNAL    — backend (asmjit) 内部错; 调用方进黑名单 / 退
//                              interpreter
//   JIT_ERR_NOT_IMPLEMENTED — backend stub 阶段返这个; T3 stub backend 总
//                              返这个, dispatcher fork 点 fall back interpreter
//
// 命名 (session_002 拍): jit_* prefix, snake_case (CLAUDE.md "Naming and style").
//

#ifndef API_JIT_API_H
#define API_JIT_API_H

#include "core/mmu.h"    // regime_t (jit_compile_block / jit_invalidate_block 参数)
#include "riscv.h"       // uxlen_t (PA 参数; dummy.txt §13 typedef family)

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------------
// jit_status_t —— JIT 接口错误码 enum
// ----------------------------------------------------------------------------
typedef enum {
    JIT_OK                  = 0,
    JIT_CODE_CACHE_FULL     = 1,
    JIT_IR_ERROR            = 2,
    JIT_BACKEND_INTERNAL    = 3,
    JIT_ERR_NOT_IMPLEMENTED = 4,
} jit_status_t;


// ----------------------------------------------------------------------------
// jit lifecycle
// ----------------------------------------------------------------------------

// jit_init: 初始化 JIT 子系统 (backend.init / translator 内部状态 / code_cache
//   mmap 区分配 / jit_cache hash 表). main 在 cpu/clint/plic 等子系统 init 之后
//   调一次; SMP 多 hart 仍只调一次 (jit_cache / code_cache 是 shared).
void jit_init(void);

// jit_shutdown: 释放 JIT 子系统资源. main 在所有 hart join 之后调一次, 跟
//   jit_init 配对. 不含 pthread_join (dummy.txt §12 — destroy 不 join).
void jit_shutdown(void);


// ----------------------------------------------------------------------------
// jit block 操作 (dispatcher fork 点 T4 真做时调)
// ----------------------------------------------------------------------------

// jit_compile_block: 触发翻译 + backend 编译一个块, 入口 = (pa, regime).
//   成功时内部把 host_code_ptr install 进 jit_cache (key = (pa, regime));
//   失败时按返码处理 (CODE_CACHE_FULL → Flush; IR_ERROR / BACKEND_INTERNAL →
//   黑名单).
//
//   T1 阶段 stub: 总返 JIT_ERR_NOT_IMPLEMENTED (T3 backend 真实装前).
//
//   签名细节 (T3 真做时可调整): 当前不带 hart 参数 — translator 编译期 baked
//   regime 已涵盖 priv 视角, 不需要具体 hart 上下文 (mstatus.SUM/MXR 等 runtime
//   inline 读, 不 baked).
jit_status_t jit_compile_block(uxlen_t pa, regime_t regime);

// jit_invalidate_block: 失效 jit_cache 内 (pa, regime) 对应的块.
//   SMC handler 延迟 invalidate / fence.i / 单块失效路径调.
//   T1 阶段 stub: nop.
void jit_invalidate_block(uxlen_t pa, regime_t regime);

// jit_flush_all: 整体 Flush jit_cache + code_cache (CODE_CACHE_FULL 时调).
//   T1 阶段 stub: nop.
void jit_flush_all(void);


#ifdef __cplusplus
}
#endif

#endif //API_JIT_API_H
