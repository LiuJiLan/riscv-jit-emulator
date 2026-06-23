//
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
// JIT 入口
// ============================================================================
//
// 5 个对外入口实装在 jit/jit_entry.cc (backend-agnostic 组合层, 通过
// backend_get_default() 拿 backend_t* 调 vtable). dispatcher fork 点 miss 路径
// 调 jit_compile_block; lifecycle (jit_init / jit_shutdown) main POR / teardown
// 配对一次; jit_flush_all 作 jit_compile_block 内部 Q11 a 组合用;
// jit_invalidate_page caller = dispatcher 顶扫 page_dirty bitmap (SMC chain
// 隐式触发); jit_invalidate_block 主要 caller 是测试 fixture, 给 future block
// chaining 等按 (pa, regime) 单块失效场景留用.
//
// 错误码 jit_status_t:
//   JIT_OK                  — 编译成功 / 接口调用成功
//   JIT_CODE_CACHE_FULL     — JitRuntime allocator 满, 调用方触发整体 Flush
//                              (plan §1.23.9)
//   JIT_IR_ERROR            — IR 内部不一致 / translator 产物错; 调用方进
//                              编译黑名单 (plan §1.23.8)
//   JIT_BACKEND_INTERNAL    — backend (asmjit) 内部错; 调用方进黑名单 / 退
//                              interpreter
//   JIT_ERR_NOT_IMPLEMENTED — IR 流空 (块前缀无真翻译 op) / unsupported op;
//                              dispatcher fork 点 fall back interpreter
//
// 命名: jit_* prefix, snake_case (CLAUDE.md "Naming and style").
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

// jit_init: 初始化 JIT 子系统 (backend.init alloc asmjit JitRuntime / jit_cache
//   hash 表). main 在 cpu/clint/plic 等子系统 init 之后调一次; SMP 多 hart 仍
//   只调一次 (jit_cache / JitRuntime 是 shared).
void jit_init(void);

// jit_shutdown: 释放 JIT 子系统资源. main 在所有 hart join 之后调一次, 跟
//   jit_init 配对. 不含 pthread_join (dummy.txt §12 — destroy 不 join).
void jit_shutdown(void);


// ----------------------------------------------------------------------------
// jit block 操作 (dispatcher fork 点 miss 路径调)
// ----------------------------------------------------------------------------

// jit_compile_block: 触发翻译 + backend 编译一个块, 入口 = (pa, regime).
//   成功时内部把 host_code_ptr install 进 jit_cache (key = (pa, regime));
//   失败时按返码处理 (CODE_CACHE_FULL → Flush; IR_ERROR / BACKEND_INTERNAL →
//   黑名单).
//
//   签名不带 hart 参数 — translator 编译期 baked regime 已涵盖 priv 视角,
//   不需要具体 hart 上下文 (mstatus.SUM/MXR 等 runtime inline 读, 不 baked).
jit_status_t jit_compile_block(uxlen_t pa, regime_t regime);

// jit_invalidate_block: 失效 jit_cache 内 (pa, regime) 对应的块, release host
//   code RX 段. 主要 caller 是测试 fixture (协议路径 verify); SMC chain 走
//   jit_invalidate_page (按 page 颗粒度更高效), 本接口给 future block chaining
//   等单块失效场景留用. 协议 4 步详 jit_entry.cc 实装.
void jit_invalidate_block(uxlen_t pa, regime_t regime);

// jit_invalidate_page: 失效 jit_cache 内 page_idx 对应 page 的所有块, release
//   各 host code RX 段. caller = dispatcher 顶扫 page_dirty bitmap (SMC chain
//   隐式触发路径). fence.i 不直接调 — audit Q4.2.1+ 拍 fence_i_helper 不主动
//   invalidate, 走块边界 + SMC chain bitmap 间接触发 (详 isa/fence.h 顶段 doc).
//   协议 4 步内部对偶 jit_invalidate_block, step 1+2 合并 (collect 跟 invalidate
//   原子打包, 详 jit_cache.h jit_cache_invalidate_page 顶段 race 分析):
//     1+2. jit_cache_invalidate_page (atomic_exchange head + 遍历 collect +
//          清 status + RCU sync; 单 atomic 边界保 race-free)
//     3. wait: 循环 cpu_wait_all_harts_exit_host_code (per host_code)
//     4. release: 循环 backend.invalidate_block (per host_code)
//   详 jit_entry.cc 实装.
void jit_invalidate_page(uint32_t page_idx);

// jit_flush_all: 整体 Flush jit_cache + code_cache (CODE_CACHE_FULL 时调).
//   实装 4 步 (jit_entry.cc; 整套裹 backend_lock wrlock 防跟 compile/invalidate
//   的 rdlock race): 1+2. jit_cache_flush_all (全表 status=EMPTY + RCU sync) →
//   3. cpu_wait_all_harts_idle_jit (等所有 hart 退出任何 host_code) →
//   4. backend.flush_all (asmjit JitRuntime::reset 整片回收 RX 池).
void jit_flush_all(void);


#ifdef __cplusplus
}
#endif

#endif //API_JIT_API_H
