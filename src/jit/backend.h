//
// Created by liujilan on 2026/6/5.
// jit/backend.h —— JitBackend 抽象接口 (C ABI vtable, 跨 C/C++).
//
// ============================================================================
// 三层架构里的位置 (plan §1.21)
// ============================================================================
//
// Dispatcher → Translator (RV → IR) → JitBackend (IR → host 机器码)
//
//   JitBackend 知道 host, 不知道 RISC-V; 只消费 IR (ir_inst_t 流) + 编译成 host
//   可执行 code (mmap 区 host_code_ptr); 暴露 C ABI vtable, 实现可换 (当前只
//   asmjit, plan §1.21 末段也论证过 LLVM 可行性).
//
// 接口形态选择 (plan §1.21):
//   - 自制 IR + C-style vtable (本头)  ✓ 选用
//   - 直接 RV → host (无 IR)            ✗ 否决 (backend 被迫 RV 化)
//   - LLVM IR                            ✗ 否决 (锁死 LLVM)
//
// ============================================================================
// 命名 (session_002 拍)
// ============================================================================
//
// vtable 函数指针 prefix `backend_*` (不加 jit_ 前缀, jit/ 目录内无歧义):
//   backend_init / backend_compile_block / backend_invalidate_block /
//   backend_flush_all / backend_destroy
// struct 类型 `backend_t`
//
// 跟 api/jit_api.h 的 jit_* 接口区分:
//   jit_*           = 子系统对外入口 (dispatcher 调; jit_init / jit_compile_block
//                      内部按 jit_cache lookup + 找 backend.compile_block)
//   backend.*       = 后端实现细节 (jit_api 内部组合用, dispatcher 不直接见)
//
// ============================================================================
// b_01 T1 阶段范围 (本文件)
// ============================================================================
//
// T1 阶段只放 vtable struct + 5 fn pointer 签名占位; T3 backend / translator
// 全 stub 时 backend_asmjit.cc 暴露一个 default backend 实例 (5 个 fn pointer
// 都填 stub 函数 — compile_block 总返 JIT_ERR_NOT_IMPLEMENTED, 其他 nop).
//
// 签名细节 (T3 真做时可调整 — 当前是 best guess 占位, 真做时按 backend 真需要
// 的形态拍):
//   - backend_compile_block 当前签名带 (pa, regime, ir 流, n_insts, host_code 出参);
//     真做时可能拆 host_code 出参为返值 + status 拆出, 或加 hart 参数 baked
//     mstatus snapshot 等
//   - backend_invalidate_block 当前签名带 (host_code), 真做时可能改 (pa, regime)
//     按 jit_cache key 失效
//
// 调用约束 (plan §1.23.17):
//   - SMC handler 不直接调 backend (只动 page_dirty bitmap; dispatcher 延迟调
//     backend_invalidate_block)
//   - Translator 不直接调 backend (Translator 只 RV → IR; Dispatcher 拿 IR 后
//     调 backend_compile_block)
//

#ifndef JIT_BACKEND_H
#define JIT_BACKEND_H

#include <stddef.h>      // size_t (ir 流长度)

#include "api/jit_api.h" // jit_status_t (compile_block 返值)
#include "core/mmu.h"    // regime_t (compile_block 参数)
#include "ir.h"          // ir_inst_t (compile_block 输入)
#include "riscv.h"       // uxlen_t (pa 参数; dummy.txt §13 typedef family)

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------------
// backend_t —— JitBackend 抽象接口的 vtable
//
// 实例由 backend_asmjit.cc (C++ 实现) 暴露 (extern "C" wrapper); jit_api.cc /
// jit_api 内部组合代码持有 backend_t * 指针调度.
//
// T1 阶段签名占位 (T3 真做时拍最终形态).
// ----------------------------------------------------------------------------
typedef struct {
    // backend_init: 后端 init (asmjit Runtime 创建 / code_cache mmap 区分配 / ...).
    //   返非 0 = init 失败 (host 资源问题; jit_init 调用方报错退出).
    int (*backend_init)(void);

    // backend_compile_block: 消费 ir_inst_t 流, 编译成 host code, 输出 host_code
    //   入口指针 (mmap 区内的 RX 段地址).
    //
    //   参数:
    //     pa             — 块入口 PA (cosmetic; 给 backend 内部 trace / debug 用,
    //                       真编译时不依赖 PA 值本身)
    //     regime         — baked priv 视角 (REGIME_BARE/SV32_S/SV32_U); backend
    //                       根据 regime 决定 fast path baked 的 PTE_U / SUM/MXR
    //                       check 形态
    //     insts          — IR 流 (translator 产出)
    //     n_insts        — IR 流长度
    //     host_code_out  — 出参; 成功时填 host_code 入口指针 (RX 段)
    //
    //   返:
    //     JIT_OK                  — 成功, host_code_out 已填; 调用方装 jit_cache
    //     JIT_CODE_CACHE_FULL     — code_cache mmap 区满, 调用方触发整体 Flush
    //     JIT_IR_ERROR            — IR 内部不一致 (translator bug); 调用方进黑名单
    //     JIT_BACKEND_INTERNAL    — asmjit 内部错; 调用方进黑名单 / 退 interpreter
    //     JIT_ERR_NOT_IMPLEMENTED — T3 stub 阶段总返这个
    //
    //   T1 stub: 返 JIT_ERR_NOT_IMPLEMENTED, *host_code_out = NULL.
    jit_status_t (*backend_compile_block)(uxlen_t pa, regime_t regime,
                                          const ir_inst_t *insts, size_t n_insts,
                                          void **host_code_out);

    // backend_invalidate_block: 失效 host_code (从 code_cache mmap 区抹掉 / 标
    //   dead 等). 当前签名按 host_code 索引; T3 真做时可能改按 (pa, regime).
    //   T1 stub: nop.
    void (*backend_invalidate_block)(void *host_code);

    // backend_flush_all: 整体 Flush code_cache (CODE_CACHE_FULL 时调).
    //   T1 stub: nop.
    void (*backend_flush_all)(void);

    // backend_destroy: 释放 backend 资源 (asmjit Runtime / code_cache mmap unmap).
    //   不含 pthread_join (dummy.txt §12 — destroy 不 join).
    //   T1 stub: nop.
    void (*backend_destroy)(void);
} backend_t;


// ----------------------------------------------------------------------------
// backend 实例获取 (T3 backend_asmjit.cc extern "C" 暴露)
//
// T1 阶段函数仅声明; T3 真做时 backend_asmjit.cc 实装返一个 file-static backend_t
// 实例的指针 (单例; SMP 多 hart 共享一个 backend, 跟 jit_cache 同步)
// ----------------------------------------------------------------------------
const backend_t *backend_get_default(void);


#ifdef __cplusplus
}
#endif

#endif //JIT_BACKEND_H
