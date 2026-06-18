//
// Created by liujilan on 2026/6/18.
// jit/backend_asmjit.cc —— JitBackend asmjit 实装 (跟 jit_entry.cc 平行).
//
// ============================================================================
// 跟 jit_entry.cc 的概念分工
// ============================================================================
//
// 本文件 = backend 实装细节; jit_entry.cc = jit_api.h 入口 (backend-agnostic).
//   - backend_asmjit.cc 暴露一个 file-static backend_t 实例 + backend_get_default()
//     extern "C" 返指针; 未来加 backend_llvm.cc 时是另一份 .cc, 不动本文件
//   - jit_entry.cc 通过 backend_get_default() 拿当前 backend_t* 调 vtable; 不感知
//     具体后端
//
// ============================================================================
// stub 阶段范围
// ============================================================================
//
// vtable fn 全 stub:
//   asmjit_backend_init             返 0 (无资源 alloc)
//   asmjit_backend_compile_block    永返 JIT_ERR_NOT_IMPLEMENTED + *host_code_out
//                                    = NULL; caller (jit_compile_block Q11 a)
//                                    走非 OK 路径 → set_blacklist + 透传错码
//   asmjit_backend_invalidate_block nop (b_03 SMC 真做时 unmap host_code 区)
//   asmjit_backend_flush_all        nop (b_03 真做时 reset code_cache mmap)
//   asmjit_backend_destroy          nop (b_02+ 真做时 asmjit JitRuntime delete)
//
// stub 阶段不 include asmjit header — backend.compile_block 永返 NOT_IMPLEMENTED
// 跑不通真编译路径, 没必要引 symbol. CMakeLists 已 target_link_libraries asmjit,
// cmake reconfigure 期 FetchContent 拉源 + 编译 verify link 路径就位.
//
// b_02 真做 emit 时:
//   #include <asmjit/asmjit.h>
//   file-static asmjit::JitRuntime g_runtime;
//   compile_block 内部 asmjit::CodeHolder + assembler emit RV→host 翻译;
//   destroy 时清理 (asmjit::JitRuntime 内部 dtor 自管, ~JitRuntime 调即可)
//
// ============================================================================
// 命名 (项目体例)
// ============================================================================
//
// 5 个 file-static fn prefix `asmjit_backend_*` (不是简单的 backend_*, 因 backend_*
// 是 vtable struct 字段名, 跟 file-static fn 名冲突会引导读者错以为 vtable fn 直接
// extern 暴露; asmjit_backend_* 表达"这是 asmjit 后端的 backend_* vtable fn 实装",
// 平行 backend_llvm_*).
//
// 不引 class / 不继承 / 不多态 — 项目 C-style vtable, .cc 内部代码也保持 C-style
// (file-static fn + file-static struct + extern "C"); 见 plan §1.5 + §1.12 实状.
//

#include "backend.h"

extern "C" {

// ----------------------------------------------------------------------------
// 5 vtable fn stub (file-static; backend_t 实例填这 5 个 fn pointer)
// ----------------------------------------------------------------------------

static int asmjit_backend_init(void) {
    return 0;
}

static jit_status_t asmjit_backend_compile_block(uxlen_t pa, regime_t regime,
                                                 const ir_inst_t *insts,
                                                 size_t n_insts,
                                                 void **host_code_out) {
    (void)pa; (void)regime; (void)insts; (void)n_insts;
    *host_code_out = nullptr;
    return JIT_ERR_NOT_IMPLEMENTED;
}

static void asmjit_backend_invalidate_block(void *host_code) {
    (void)host_code;
}

static void asmjit_backend_flush_all(void) {
}

static void asmjit_backend_destroy(void) {
}


// ----------------------------------------------------------------------------
// file-static backend_t 单例 + extern "C" backend_get_default() 暴露
//
// designated initializer (C++20 标准; gnu++17 作 GCC extension 接受) 跟 backend.h
// struct 字段一一对应, 顺序无关; 跟 C 端项目体例对偶 (lrsc.c / clint.c 等大量
// 用 designated initializer).
// ----------------------------------------------------------------------------
static const backend_t asmjit_backend = {
    .backend_init             = asmjit_backend_init,
    .backend_compile_block    = asmjit_backend_compile_block,
    .backend_invalidate_block = asmjit_backend_invalidate_block,
    .backend_flush_all        = asmjit_backend_flush_all,
    .backend_destroy          = asmjit_backend_destroy,
};

const backend_t *backend_get_default(void) {
    return &asmjit_backend;
}

}  // extern "C"
