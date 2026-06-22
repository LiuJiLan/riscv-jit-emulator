//
// jit/jit_entry.cc —— JIT 子系统对外入口实装 (jit_api.h 5 函数的 .cc 实装).
//
// ============================================================================
// 概念分工
// ============================================================================
//
// 本文件 = jit_api.h 入口实装 (backend-agnostic; 不绑具体后端).
// 跟 backend_asmjit.cc 平行 — backend_asmjit.cc 是 backend 实装细节, jit_entry.cc
// 通过 backend_get_default() 拿当前 backend_t* 调 vtable. 未来加 backend_llvm.cc
// 时本文件零改动, backend_get_default() 内部按 build-time flag 切返哪个 backend.
//
// plan §1.12 行 553 "未来切换 LLVM 后端, 只是 jit_init 内部 new 不同 class, C 侧零
// 改动" 的实状形态 — 实状走 C-style vtable + fn pointer (不是 C++ virtual class),
// 但 "jit_api.h 入口 backend-agnostic" 这一层语义对偶.
//
// ============================================================================
// 5 个 extern "C" 入口 (jit_api.h 声明)
// ============================================================================
//
// jit_init / jit_shutdown — lifecycle (main POR / teardown 一次):
//   jit_init    顺序: backend.init → jit_cache_init (backend 先 alloc 资源,
//                                                    cache 再初始化 hash 表)
//   jit_shutdown 反序: jit_cache_destroy → backend.destroy
//
// jit_flush_all (jit_compile_block Q11 a 内部组合也调; dispatcher 不直接调):
//   backend_lock wrlock (排他, 等所有 rdlock 持有方退) → jit_cache_flush_all
//   (mark EMPTY + RCU sync) → cpu_wait_all_harts_idle_jit (等所有 hart 退出
//   任何 host_code) → backend.flush_all (asmjit JitRuntime::reset, 现在物理
//   安全) → wrunlock. 协议 4 步对偶 jit_invalidate_block 但 wait 升级到全
//   hart idle (reset 释放整片 RX 池). b_04_session_002 加 SMP 保护, 详 backend_
//   lock 段顶 doc 跟 jit_flush_all 函数顶 doc.
//
// jit_invalidate_block (协议骨架接通; 按 (pa, regime) 单块失效):
//   take 路径: jit_cache_take_compiled_host_code(pa, regime) 拿 host_code +
//              CAS COMPILED→EMPTY (后续 lookup 见 EMPTY 视 miss);
//   RCU sync : jit_rcu_synchronize 等所有 in-flight dispatcher read critical
//              出, 确保已经 hit COMPILED 的 dispatcher 都已经写完状态灯;
//   状态灯扫 : cpu_wait_all_harts_exit_host_code 扫所有 hart 等退出该 host_code
//              (起步 busy-wait + PAUSE);
//   release : backend.invalidate_block(host_code) 真 release host code RX 段.
//
//   caller: SMC chain 走 jit_invalidate_page (顺 per-page list 整页清; SIGSEGV
//   handler 标 page_dirty bitmap → dispatcher 主循环顶扫 → 调 page 版组合层);
//   本接口按 (pa, regime) 单块失效语义, 给 future block chaining / 单点
//   invalidate 真撞场景留用, 主要 caller 是测试 fixture 验协议路径.
//   注意 sfence.vma 不调本接口 — sfence 只清 TLB, JIT 块 key=(PA, regime) 在
//   sfence 后不过期 (PA 不变 + perm runtime 读 TLB; 详 sfence.h 顶段).
//   注意 fence.i 也不调本接口 — audit Q4.2.1+ 拍 fence_i_helper 不主动
//   invalidate, 走块边界 + SMC chain bitmap 间接 (详 fence.h 顶段 doc).
//
// jit_compile_block — Q11 a 组合层 (dispatcher fork 点 miss 路径调):
//   step 1: backend.compile_block → JIT_OK 时 jit_cache_install + 返 JIT_OK
//   step 2: JIT_CODE_CACHE_FULL 时 Flush + retry 1 次
//     - jit_flush_all → backend.compile_block 二次
//     - 二次 OK → install + 返 OK
//     - 二次 FULL → set_blacklist + 返 JIT_BACKEND_INTERNAL
//     - 二次撞别的错码 → 透传 step 3
//   step 3: 非 FULL 错码 (IR_ERROR / BACKEND_INTERNAL / NOT_IMPLEMENTED):
//     set_blacklist + 透传错码; dispatcher 看非 OK 退 interpreter
//
// ============================================================================
// translator 调用
// ============================================================================
//
// jit_api.h doc 写 "触发翻译 + backend 编译一个块" — 完整路径:
//   ir_inst_t ir_buf[BLOCK_INST_LIMIT];
//   size_t n_insts = 0;
//   translator_translate(pa, regime, ir_buf, &n_insts);
//   backend.compile_block(pa, regime, ir_buf, n_insts, &host_code);
//
// translator_translate 翻译 RV32 IMC 全集 (详 ir.h 顶段 op_kind 清单), 截断
// emit 出口模板收尾; 块前缀空时 *n_insts = 1 (仅出口模板), backend 端兜底返
// NOT_IMPLEMENTED → set_blacklist → dispatcher 兜底走 interpret 真执行.
//

#include <pthread.h>         // backend_lock rwlock (b_04_session_002 SMP)
#include <stdio.h>           // fprintf (Flush / BLACK 异常路径 stderr 提示)

#include "api/jit_api.h"
#include "config.h"          // BLOCK_INST_LIMIT (ir_buf 大小) / MAX_BLOCKS_PER_PAGE / EOL
#include "core/cpu.h"        // cpu_wait_all_harts_exit_host_code / idle_jit (invalidate / flush 协议)
#include "jit/backend.h"
#include "jit/jit_cache.h"
#include "jit/translator.h"  // translator_translate (RV → IR)

extern "C" {

// ----------------------------------------------------------------------------
// backend_lock —— rwlock 串行化 g_rt 访问跟 reset 之间的关系 (b_04_session_002)
//
// 背景: asmjit JitAllocator::reset() 上游 doc 明字 "not thread-safe ... when
// nobody else is using the JitAllocator" — alloc / release 内部 LockGuard 真
// SMP 安全 (跨 hart 同时 compile/invalidate 没 race), 但 reset() 跟其他任何
// op 并发就会撞 freelist + freed-while-executing. 解法 = 加 rwlock:
//
//   rdlock 持有方: jit_compile_block (backend.compile_block + install 整段) /
//                   jit_invalidate_block / jit_invalidate_page (backend.invalidate_
//                   block 调用) — 真并发互不阻塞 (asmjit 内部 LockGuard 已防 race)
//   wrlock 持有方: jit_flush_all (cache_flush_all + wait_idle + backend.flush_all
//                   整段) — 排他, 等所有 rdlock release 才能 acquire
//
// 协议跟 jit_invalidate_block 4-step 同精神, 但 reset 涉及全 host_code 段, 所以
// step 3 wait 从单 host_code 升级到 "全 hart 退出任何 host_code"
// (cpu_wait_all_harts_idle_jit).
//
// 锁顺序 invariant: rdlock 持有时不嵌套调任何走 wrlock 的接口 (否则 deadlock).
// jit_compile_block step 2 的 jit_flush_all 必须在 rdunlock **之后** 调.
//
// PTHREAD_RWLOCK_INITIALIZER 静态初始化, 不需 init/destroy 配对 (跟 uart_lock /
// other file-static pthread_mutex_t 对偶).
// ----------------------------------------------------------------------------
static pthread_rwlock_t backend_lock = PTHREAD_RWLOCK_INITIALIZER;

// ----------------------------------------------------------------------------
// lifecycle (main POR / teardown 一次配对; jit_cache + backend 都是单例 + 跨
// system reset 一直跑, 跟 wfi / lrsc 体例同, system reset (while 每 iter) 不调).
// ----------------------------------------------------------------------------

void jit_init(void) {
    const backend_t *be = backend_get_default();
    /* backend.init 跟 jit_cache_init 当前实装都返 0 无失败路径; backend.init
     * asmjit JitRuntime new 成功返 0; jit_cache_init 内部纯 atomic store 也不失败.
     * 真失败语义 (asmjit Runtime alloc OOM 等) 按 wfi_init / lrsc_init 体例
     * fprintf + 不传播; 那时调整本处签名为接 int 返码 + main 接 fail 退. */
    (void)be->backend_init();
    (void)jit_cache_init();
}

void jit_shutdown(void) {
    jit_cache_destroy();
    const backend_t *be = backend_get_default();
    be->backend_destroy();
}


// ----------------------------------------------------------------------------
// jit_flush_all — Q11 a 组合层内部用; dispatcher 不直接调 (Q11 a 拍 "dispatcher
// 不感知 Flush"). 4-step 协议跟 jit_invalidate_block 对偶, 但 reset 全 RX 池,
// step 3 wait 升级到全 hart idle (b_04_session_002).
//
// 协议 4 步:
//   step 1+2: jit_cache_flush_all (mark 全表 EMPTY + 内部 jit_rcu_synchronize
//              等 grace period; 新 lookup 全 miss)
//   step 3:   cpu_wait_all_harts_idle_jit (等所有 hart 退出任何 host_code,
//              避免 reset 释放 RX 段时别 hart 跳进垃圾内存)
//   step 4:   backend.flush_all (asmjit JitRuntime::reset 整片回收)
//
// SMP 保护: 整套裹在 backend_lock wrlock 内, 跟 jit_compile_block /
// jit_invalidate_block / jit_invalidate_page 的 rdlock 互斥 (rwlock writer 等
// readers 全 release). asmjit JitAllocator::reset() doc 明字 "not thread-safe",
// 加 wrlock 才物理安全.
//
// 推翻 trail (b_04_session_002): 早期版本只 jit_cache_flush_all → backend.flush_all,
// 缺 SMP 保护 + 缺 step 3 wait. 当时 jit_flush_all 是 dead code (backend 不返
// JIT_CODE_CACHE_FULL, jit_compile_block 走不到 step 2 Flush 路径) 没暴露. 本
// session 顺手把 backend OOM 细化 + flush_all 4-step 补全, 真撞 cache 满时安全.
// ----------------------------------------------------------------------------
void jit_flush_all(void) {
    pthread_rwlock_wrlock(&backend_lock);

    /* step 1+2: mark 全表 EMPTY + RCU sync */
    jit_cache_flush_all();

    /* step 3: 等所有 hart 退出任何 host_code (reset 释放整片 RX 池, 不能像
     * invalidate_block 那样按单 host_code wait) */
    cpu_wait_all_harts_idle_jit();

    /* step 4: backend release 整 RX 池 (asmjit JitRuntime::reset, 现在物理安全) */
    const backend_t *be = backend_get_default();
    be->backend_flush_all();

    pthread_rwlock_unlock(&backend_lock);
}


// ----------------------------------------------------------------------------
// jit_invalidate_block — 按 (pa, regime) 单块失效 (协议骨架接通)
//
// 协议 4 步 (详本文件顶段 §"5 个 extern C 入口" jit_invalidate_block 段):
//   1. take: jit_cache_take_compiled_host_code 拿 host_code + CAS COMPILED→EMPTY
//   2. RCU sync: 等 in-flight dispatcher read critical 出 (确保状态灯写完 visible)
//   3. 状态灯扫: cpu_wait_all_harts_exit_host_code (busy-wait + PAUSE)
//   4. release: backend.invalidate_block release host code RX 段
//
// race-safe: take CAS 拿失败 → host_code = NULL → 后续 step skip (别 hart 已
// invalidate / 还没 promote / BLACK).
//
// 调用者:
//   - 测试 fixture 直调 (协议路径 verify)
//   - SMC chain 不走本接口, 走 jit_invalidate_page (按 page 整页清, 顺链表).
//     本接口给 future block chaining 等按 (pa, regime) 单块失效场景预留.
//
// 不接 sfence_vma_helper — sfence 只清 TLB, JIT 块 key=(PA, regime) 不过期.
// 不接 fence_i_helper — fence.i 走块边界 + SMC chain bitmap 间接 (详 fence.h).
// ----------------------------------------------------------------------------
void jit_invalidate_block(uxlen_t pa, regime_t regime) {
    /* step 1: take + CAS COMPILED→EMPTY */
    void *host_code = jit_cache_take_compiled_host_code(pa, regime);
    if (host_code == nullptr) {
        return;   /* 无 entry / 非 COMPILED / 已被别 hart take 走 — 不需 invalidate */
    }

    /* step 2: RCU sync (等所有 in-flight read critical 出; 此时所有"已 lookup hit
     * 的"dispatcher 都已经写完状态灯 release, step 3 扫描能看到 visible 值) */
    jit_rcu_synchronize();

    /* step 3: 状态灯扫所有 hart 等退出 host_code (busy-wait + PAUSE) */
    cpu_wait_all_harts_exit_host_code(host_code);

    /* step 4: backend release host code RX 段 (asmjit JitRuntime::release).
     * 裹 backend_lock rdlock 防跟 jit_flush_all wrlock 内的 backend.flush_all
     * (reset 整片 RX 池) race; rdlock 允许多 hart 并发 invalidate_block, asmjit
     * release() 内部 LockGuard 已防 alloc/release 同 freelist race. */
    pthread_rwlock_rdlock(&backend_lock);
    const backend_t *be = backend_get_default();
    be->backend_invalidate_block(host_code);
    pthread_rwlock_unlock(&backend_lock);
}


// ----------------------------------------------------------------------------
// jit_invalidate_page — 按 page_idx 整 page 失效 (SMC chain caller 用)
//
// 协议 4 步内部对偶 jit_invalidate_block (单块版), 差异在 step 1+2 合并 (collect
// 跟 invalidate 必须原子打包; 详 jit_cache.h jit_cache_invalidate_page 顶段
// "为什么 collect 必须嵌入本函数" race 分析):
//   1+2. jit_cache_invalidate_page (内部 atomic_exchange head + 遍历 collect
//        host_code 进 out 数组 + 清 status + RCU sync); 替代了 block 版的
//        take CAS COMPILED→EMPTY + RCU sync 两步
//   3. 状态灯扫: 循环每 host_code 调 cpu_wait_all_harts_exit_host_code
//   4. release: 循环每 host_code 调 backend.invalidate_block
//
// caller (b_03 T1.a 接通):
//   - dispatcher 主循环顶扫 page_dirty bitmap 拿到 dirty page_idx 调本函数
//   - T2 fence.i 不直接调本函数 (audit Q4.2.1+ 拍 fence_i_helper 不主动
//     invalidate, 走块边界 + 已 SMC chain 路径自动)
//
// out 数组用栈 (MAX_BLOCKS_PER_PAGE, 起步 16; jit_cache.h 顶段说明上限算法).
// NULL 元素 (BLACK / COUNTING entry) skip.
// ----------------------------------------------------------------------------
void jit_invalidate_page(uint32_t page_idx) {
    /* step 1+2: collect + invalidate + RCU sync (jit_cache 内原子打包) */
    void *host_codes[MAX_BLOCKS_PER_PAGE];
    size_t n_codes = 0;
    jit_cache_invalidate_page(page_idx, host_codes,
                              (size_t)MAX_BLOCKS_PER_PAGE, &n_codes);
    if (n_codes == 0u) {
        return;   /* page 空 (无 JIT 块) / page_idx 越界 — 不需 release */
    }

    /* step 3 + 4: 循环每 host_code 等 hart 退出 + backend release. 整 loop 裹
     * 一次 backend_lock rdlock — flush_all wrlock 不能在 loop 中途插入 (插入
     * 会导致后续 backend.invalidate_block 撞已 release 的 host_code, double-free
     * 或撞 asmjit freelist). 单次 rdlock 包整 loop 是 minimal scope, 跟 b_03_
     * session_009 backend_lock 段顶 doc 协议一致. */
    pthread_rwlock_rdlock(&backend_lock);
    const backend_t *be = backend_get_default();
    for (size_t i = 0; i < n_codes; i++) {
        if (host_codes[i] == nullptr) { continue; }   /* BLACK / COUNTING 无 host_code */
        cpu_wait_all_harts_exit_host_code(host_codes[i]);
        be->backend_invalidate_block(host_codes[i]);
    }
    pthread_rwlock_unlock(&backend_lock);
}


// ----------------------------------------------------------------------------
// jit_compile_block — Q11 a 组合层 (dispatcher fork 点 miss 路径调)
//
// 返码分流见本文件顶段 §"5 个 extern C 入口" jit_compile_block 三 step.
// ----------------------------------------------------------------------------
jit_status_t jit_compile_block(uxlen_t pa, regime_t regime) {
    const backend_t *be = backend_get_default();

    /* RV → IR 翻译. ir_buf stack 分配 BLOCK_INST_LIMIT slot, translator 写真实
     * 长度到 n_insts; 块前缀空 (i==0) 时 n_insts = 1 (仅出口模板),
     * backend.compile_block 检测后返 NOT_IMPLEMENTED, step 3 set_blacklist 走
     * interpret 兜底. */
    ir_inst_t ir_buf[BLOCK_INST_LIMIT];
    size_t n_insts = 0;
    translator_translate(pa, regime, ir_buf, &n_insts);

    /* step 1: 首次 compile (rdlock 包 compile + install, 防 flush_all wrlock
     * 在 compile 返 OK 跟 install 之间插入释放 host_code RX 段 → 别 hart 后续
     * lookup 拿到悬空指针). 详 backend_lock 段顶 doc. */
    void *host_code = nullptr;
    pthread_rwlock_rdlock(&backend_lock);
    jit_status_t s = be->backend_compile_block(pa, regime,
                                               ir_buf, n_insts,
                                               &host_code);
    if (s == JIT_OK) {
        (void)jit_cache_install(pa, regime, host_code);
        pthread_rwlock_unlock(&backend_lock);
        return JIT_OK;
    }
    pthread_rwlock_unlock(&backend_lock);

    /* step 2: JIT_CODE_CACHE_FULL → Flush + retry 1 次 (异常路径, 一次性 stderr
     * 提示 — 信号价值高, 不 gate; 真撞 = JIT_CACHE_SIZE 触底).
     * 关键: jit_flush_all 内部走 wrlock, 必须在上方 rdunlock 之后调 (否则
     * deadlock). retry 重新 rdlock 包. */
    if (s == JIT_CODE_CACHE_FULL) {
        fprintf(stderr, "[jit] FULL → Flush + retry: pa=0x%08x regime=%u" EOL,
                (uint32_t)pa, (uint32_t)regime);
        jit_flush_all();   /* 内部 wrlock; rdunlock 完成才能调, 见上方 invariant */

        host_code = nullptr;
        pthread_rwlock_rdlock(&backend_lock);
        s = be->backend_compile_block(pa, regime,
                                      ir_buf, n_insts,
                                      &host_code);
        if (s == JIT_OK) {
            (void)jit_cache_install(pa, regime, host_code);
            pthread_rwlock_unlock(&backend_lock);
            return JIT_OK;
        }
        pthread_rwlock_unlock(&backend_lock);

        if (s == JIT_CODE_CACHE_FULL) {
            /* 二次 FULL: 单块 host code > 整个 code_cache, 几乎不可能 (RV block
             * ≤ 64 inst × ~16 byte/inst < 1KB; 1MB+ code_cache) — 真撞是 bug.
             * 进 BLACK + 返 BACKEND_INTERNAL (跟 IR_ERROR / 真 BACKEND_INTERNAL
             * 同处理, dispatcher 退 interpreter). */
            fprintf(stderr, "[jit] 二次 FULL → BLACK: pa=0x%08x regime=%u (单块"
                    " host code > code_cache; 检查 backend / config)" EOL,
                    (uint32_t)pa, (uint32_t)regime);
            (void)jit_cache_set_blacklist(pa, regime);
            return JIT_BACKEND_INTERNAL;
        }
        /* 二次撞别的错码 (IR_ERROR / BACKEND_INTERNAL / NOT_IMPLEMENTED) 透传 step 3 */
    }

    /* step 3: 非 FULL 错码透传 — set_blacklist + 透传; dispatcher 退 interpreter.
     * NOT_IMPLEMENTED (块前缀空) 是 expected 常见路径不打; 其他错码异常路径打提示. */
    if (s != JIT_ERR_NOT_IMPLEMENTED) {
        fprintf(stderr, "[jit] compile error → BLACK: pa=0x%08x regime=%u status=%d" EOL,
                (uint32_t)pa, (uint32_t)regime, (int)s);
    }
    (void)jit_cache_set_blacklist(pa, regime);
    return s;
}

}  // extern "C"
