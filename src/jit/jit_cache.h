//
// jit/jit_cache.h —— JIT 块缓存 (PA, regime) → host_code_ptr 的 hash table.
//
// ============================================================================
// 设计依据 (audit Q9/Q10/Q11/Q12; 详 trade_off_log §T JIT cache)
// ============================================================================
//
// 三个核心拍法:
//   Q9 方案 2  — per-page block list: hash entry 加 next_in_page 链表字段 +
//                全局 page_block_head[GUEST_RAM_NPAGES] 链表头数组; install
//                CAS 挂头, invalidate_page 顺链表清 (跟 hash table 全扫的方案 1
//                比 invalidate O(块/页) vs O(全 cache))
//   Q10 c RCU — EBR (epoch-based reclamation) 自实现, lookup read-side 无锁;
//                install / invalidate_page / flush_all 走 modify-side, 等
//                grace period 后才允许 backend 真 release host_code RX 段
//   Q11 a    — jit_api 层 Flush + retry 1 次, dispatcher 不感知 Flush;
//                jit_cache_install 返 -1 (FULL) 时 caller (jit_entry.cc) 触发
//                jit_flush_all + 再 install 重试 1 次 (retry 完整路径见
//                jit/jit_entry.cc)
//   Q12 c    — invalidate_reason_t 入参不带; 现役两条路径 (SMC invalidate /
//                Flush_all) 都清 BLACK; 未来加 block chaining / SMC 立即重编时
//                ABI append-only 扩 reason 入参
//
// SMP 一次性到位原则 (plan §1.9 末段 + dummy.txt §15 末段):
//   jit_cache 出生即 lock-free + RCU, 不走"先单 hart 后补 SMP".
//
// cap-vs-n_harts (dummy.txt §15): 数组定义 / lifecycle 配对 / phantom 防御
//   按 cap (MAX_HARTS / JIT_CACHE_SIZE); RCU synchronize 遍历按 n_harts (存在性).
//
// ============================================================================
// 数据布局 (.c file-static; 跟 lrsc.c bucket_lock 同体例, 内部细节不暴露)
// ============================================================================
//
//   hash_table[JIT_CACHE_SIZE]              jit_cache_entry_t × 65536, ~2MB
//   page_block_head[GUEST_RAM_NPAGES]       _Atomic uint16_t × 32768, ~64KB
//   jit_rcu_slots[MAX_HARTS]                per-hart epoch 数组, _Alignas(64) 防假共享
//   jit_rcu_global_epoch                    _Atomic uint32_t global epoch counter
//
// hash 函数: Fibonacci (跟 lrsc.c bucket_lock 同体例; 常数 2654435761u XOR 混入
//   regime 第二常数 40503u; 详 .c hash 函数实装).
//
// ============================================================================
// 接口分两组
// ============================================================================
//
// jit_cache_*    — hash table + per-page list 操作; caller = jit_entry.cc 组合层
//                   (jit_init / jit_flush_all forward)
//   jit_cache_init / destroy           — lifecycle (main POR / 退出段调)
//   jit_cache_lookup                   — RCU read-side (caller 已调 jit_rcu_read_lock)
//   jit_cache_install                  — RCU modify-side (atomic CAS slot status)
//   jit_cache_invalidate_page          — RCU modify-side (sync grace period)
//   jit_cache_flush_all                — RCU modify-side (sync grace period)
//
// jit_rcu_*      — EBR critical section; caller = dispatcher fork 点 +
//                   modify-side 函数自身 (jit_rcu_synchronize)
//   jit_rcu_read_lock / read_unlock    — read critical section enter / exit
//                                         (1 cycle 退化为 store 一个 epoch)
//   jit_rcu_synchronize                — modify-side 等所有 hart 出 critical
//
// ============================================================================
// 命名
// ============================================================================
//
// 类型 jit_cache_*_t (jit_cache_status_t / jit_cache_entry_t); enum 值
// JIT_CACHE_* (JIT_CACHE_EMPTY / JIT_CACHE_COUNTING / JIT_CACHE_COMPILED /
// JIT_CACHE_BLACK); 函数 jit_cache_* / jit_rcu_* (snake_case, module_verb_object).
//

#ifndef JIT_JIT_CACHE_H
#define JIT_JIT_CACHE_H

#include <stdint.h>
#ifndef __cplusplus
#include <stdatomic.h>     // C++ 端 GCC 11 g++ 不接受 _Atomic extension, 整 stdatomic.h
                           // 在 C++ 编译撞 fail; C++ caller (jit_entry.cc) 不实例化
                           // entry struct 也不调 jit_rcu_*, 不需要 atomic 类型可见.
#endif

#include "core/mmu.h"   // regime_t (key 第二维; mmu.h 不重复 include riscv.h)
#include "riscv.h"      // uxlen_t (key_pa; dummy.txt §13 typedef family)

#ifdef __cplusplus
extern "C" {
#endif


#ifndef __cplusplus
// ----------------------------------------------------------------------------
// jit_cache_status_t —— hash entry 状态机 (4 状态; C 端可见, C++ 端不暴露)
//
//   EMPTY    — 空槽; lookup 终止线性探测 (开放寻址 sentinel)
//   COUNTING — install 进行中 (CAS EMPTY→COUNTING 后填字段中); lookup 视为 miss
//   COMPILED — install 完成, host_code_ptr 已 visible; lookup hit 返 entry
//   BLACK    — backend 编译失败黑名单 (Q11 a 二次 FULL 路径 + Q12 c invalidate
//              来源区分); lookup 返 NULL (跳过当前 PC); invalidate_page /
//              flush_all 都清 BLACK → EMPTY (Q12 c 现役两条路径)
//
// 状态转换主路径:
//   EMPTY → COUNTING (install CAS) → COMPILED (install store, release)
//   EMPTY → BLACK    (backend fail 路径; jit_entry.cc set_blacklist)
//   COMPILED → EMPTY (invalidate_page / flush_all RCU grace period 后)
//   BLACK    → EMPTY (invalidate_page / flush_all; Q12 c)
//
// 字段 _Atomic uint32_t (不是 _Atomic jit_cache_status_t — atomic 加 enum 在 C11
// 上 ok 但部分编译器 warning; 项目既有体例 cpu->priv 等 enum 用 uint32_t/uint8_t).
// ----------------------------------------------------------------------------
typedef enum {
    JIT_CACHE_EMPTY    = 0,
    JIT_CACHE_COUNTING = 1,
    JIT_CACHE_COMPILED = 2,
    JIT_CACHE_BLACK    = 3,
} jit_cache_status_t;


// ----------------------------------------------------------------------------
// jit_cache_entry_t —— 单 hash slot (~32 byte; 64K slot × 32 = 2MB)
//
// 字段:
//   key_pa         — 块入口 PA (uxlen_t; dummy.txt §13)
//   key_regime     — baked priv 视角 (BARE / SV32_S / SV32_U; core/mmu.h)
//   status         — 状态机 (atomic; load acquire / store release)
//   counter        — 解释器执行次数计数 (热度阈值; plan §1.23.8; _Atomic 多 hart
//                     并发 atomic_fetch_add). 达 config.h COMPILE_THRESHOLD 触发
//                     jit_compile_block + promote COUNTING → COMPILED.
//   host_code_ptr  — backend 编译产物入口 (类型 jit_block_func_t = void (*)
//                     (cpu_t*, tlb_t*, uint64_t*); Q1 a; 指向 asmjit JitRuntime
//                     mmap RX 段)
//   next_in_page   — per-page block list 链表 next 索引 (uint16_t; JIT_CACHE_SIZE
//                     = 65536 索引正好用满 uint16_t; 0xFFFF = 链尾哨兵 = JIT_PAGE_HEAD_END)
//
// 字段非 atomic (host_code_ptr / next_in_page):
//   install CAS status EMPTY→COUNTING 后 owner 独占写这些字段; release store
//   status = COMPILED 配 lookup acquire load 形成 happens-before, lookup 看到
//   COMPILED 时其他字段已 visible.
//
// key_pa / key_regime / counter 是 _Atomic:
//   - counter: dispatcher fork 点 multi-hart 并发 atomic_fetch_add
//   - key_pa / key_regime: lookup_or_init 在 status=COUNTING 状态也读 key 比较
//     (跟 install owner CAS 后填 key 期间是 in-flight 窗口, "COUNTING 不读 key"
//     会导致 lookup_or_init 误退 → 同 PA 多 entry 蔓延). _Atomic 让读写都
//     atomic_relaxed (relaxed 即可 — status 的 acquire/release 已建立 happens-
//     before, key 只需要不撕裂值; TSan 验证 race 无).
//
// key_regime 类型 _Atomic uint32_t (跟 status 同体例; atomic 加 enum C11 OK 但部
// 分编译器 warning, 项目既有体例 cpu->priv 等 enum 用 uint32_t/uint8_t. 比较时
// caller 调 entry_key_matches helper cast regime_t → uint32_t).
// ----------------------------------------------------------------------------
typedef struct jit_cache_entry_s {
    _Atomic uxlen_t   key_pa;
    _Atomic uint32_t  key_regime;      // 实际 regime_t 值, atomic 字段用 uint32_t 体例
    _Atomic uint32_t  status;          // jit_cache_status_t
    _Atomic uint32_t  counter;         // heat counter
    void             *host_code_ptr;
    uint16_t          next_in_page;
} jit_cache_entry_t;

#else
// ----------------------------------------------------------------------------
// C++ 端: forward decl entry struct (opaque type)
//
// C++ caller (jit_entry.cc) 只调 lifecycle / install / flush / set_blacklist,
// 不实例化 entry struct 也不读字段, opaque pointer 够用. jit_cache_lookup 返
// jit_cache_entry_t * 在 C++ 端是 opaque 指针 (访问字段会撞 incomplete type fail);
// dispatcher fork 点调 lookup 是 C 文件不撞此问题.
//
// 这条避绕是因 GCC 11 g++ 不接受 _Atomic keyword extension (升 GCC 13+ 后可重审).
// ----------------------------------------------------------------------------
typedef struct jit_cache_entry_s jit_cache_entry_t;

#endif // ifndef __cplusplus


// ----------------------------------------------------------------------------
// lifecycle (main POR / 退出段调; 对偶 wfi_init/destroy)
//
// jit_cache_init: 失败返 -1 (跟 wfi_init / clint_init 同体例 — 半 init 不回滚,
//   main 不跑 destroy chain 进程退出 OS 回收). 当前实装无失败路径 (纯 atomic
//   store 初值), 始终返 0; 签名仍 int 留扩展余地.
//
// jit_cache_destroy: 纯无锁数据结构 (无 mutex / cond / pthread); 此函数主要写
//   atomic clear 防 valgrind 噪声 (跟 lrsc_destroy 体例).
// ----------------------------------------------------------------------------
int  jit_cache_init(void);
void jit_cache_destroy(void);


// ----------------------------------------------------------------------------
// jit_cache_lookup —— RCU read-side, 无锁
//
// 前置: caller 已调 jit_rcu_read_lock(hart_id) 进入 read critical section.
//
// 流程:
//   1. idx = jit_cache_hash(pa, regime)
//   2. 线性探测 ≤ JIT_LOOKUP_MAX_PROBES 步:
//      - status acquire = COMPILED + key 匹配 → return &entry
//      - status = EMPTY → 终止 (开放寻址 sentinel, miss)
//      - status = COUNTING / BLACK → 继续探测下一槽
//   3. miss → return NULL
//
// caller 后续:
//   - hit 路径: 调 host_code(entry->host_code_ptr) 后再调 jit_rcu_read_unlock
//     (host_code 执行过程 RX 段不能被 backend 真 release;
//      invalidate 路径 jit_rcu_synchronize 等出 critical 才允许 backend release)
//   - miss 路径: 调 jit_rcu_read_unlock 后走 interpreter fallback
// ----------------------------------------------------------------------------
jit_cache_entry_t *jit_cache_lookup(uxlen_t pa, regime_t regime);


// ----------------------------------------------------------------------------
// jit_cache_lookup_or_init —— RCU read-side + lazy install COUNTING
//
// caller: dispatcher fork 点 (C-only, dispatcher.c). C++ 端 (jit_entry.cc) 不
// 调用此接口 — 它访问 entry 字段 (status/counter/host_code_ptr) 而 C++ 端
// entry struct 是 opaque (GCC 11 _Atomic 兼容性).
//
// 前置: caller 已调 jit_rcu_read_lock(hart_id) 进入 read critical section.
//
// 流程 (跟 lookup 同 hash + 线性探测, miss EMPTY 时多一步 install COUNTING):
//   1. idx = jit_cache_hash(pa, regime)
//   2. 线性探测 ≤ JIT_LOOKUP_MAX_PROBES 步:
//      - key 匹配 + status ∈ {COMPILED, COUNTING, BLACK} → return &entry
//        (跟 lookup 不同: lookup 只返 COMPILED, lookup_or_init 返任意非 EMPTY)
//      - status = EMPTY → CAS EMPTY → COUNTING:
//          - 成功 → 填 key + 挂 page_block_head + counter=0; status 留 COUNTING
//                  (不切 COMPILED); return &entry
//          - 失败 → race, 另一 hart 抢先 install, 继续探测重判
//   3. 探测 N 步耗尽 → return NULL (hash table 满, dispatcher 退 interpret)
//
// caller 拿到 entry 后据 status 分流:
//   - COMPILED → host_code 跑 (写 cpu->jit_executing_host_code 状态灯; 跑完清)
//   - COUNTING → atomic_fetch_add(&entry->counter, 1); 达 COMPILE_THRESHOLD
//                调 jit_compile_block (本次仍走 interpret, 编译后下次 lookup hit)
//   - BLACK    → interpret 兜底, 不增 counter (永不编)
//
// 跟 lookup 关系: lookup 体例不变 (其他 caller 兼容), lookup_or_init 是
// dispatcher 专用. 共享 hash + 探测核心 (jit_cache.c 内 file-static helper).
// ----------------------------------------------------------------------------
#ifndef __cplusplus
jit_cache_entry_t *jit_cache_lookup_or_init(uxlen_t pa, regime_t regime);
#endif


// ----------------------------------------------------------------------------
// jit_cache_install —— RCU modify-side, atomic CAS 路径无锁
//
// 流程:
//   1. idx = jit_cache_hash(pa, regime)
//   2. 线性探测 ≤ JIT_INSTALL_MAX_PROBES 步:
//      - 撞 EMPTY slot: status atomic CAS EMPTY → COUNTING; 成功后填 key /
//        host_code_ptr / counter=0 / 挂 page_block_head; 走 step 3 store COMPILED
//      - 撞 key 匹配 + status = COUNTING (dispatcher 已 lookup_or_init 建好
//        COUNTING entry): 写 host_code_ptr; 走 step 3 store COMPILED
//        (promote 路径; 此时 key / next_in_page 已 visible, 不重填)
//      - 撞 key 匹配 + status = COMPILED / BLACK: idempotent return 0 (race 时
//        另一 hart 已完成)
//      - 撞别的 key + status ∈ {COUNTING/COMPILED/BLACK} → 继续探测下一槽
//   3. atomic store status = COMPILED (release; 让 lookup 看到 COMPILED 时
//      其他字段已 visible)
//
// 返:
//   0  — install 成功 (status = COMPILED) 或 idempotent
//   -1 — install FULL (探测 N 步都没合适 slot → hash table 满 → caller Flush)
//
// caller (jit_api 层) 处理 -1 (Q11 a): 调 jit_flush_all + 再调 install 重试 1 次;
// 二次 -1 调 jit_cache_set_blacklist (本头下方接口) 让 (PA, regime) 进 BLACK.
//
// 兼容两路径: dispatcher fork 点 lookup_or_init 已建 COUNTING entry, 走 promote;
// jit_compile_block 二次 Flush 后 entry 被全清回 EMPTY, 走 EMPTY→COMPILED
// 直通路径.
// ----------------------------------------------------------------------------
int  jit_cache_install(uxlen_t pa, regime_t regime, void *host_code_ptr);


// ----------------------------------------------------------------------------
// jit_cache_take_compiled_host_code —— take + CAS COMPILED→EMPTY
//
// caller: jit_entry.cc 的 jit_invalidate_block (C/C++ 共用接口, 返 void* 不返
// entry struct, C++ 端友好).
//
// 流程:
//   1. lookup_for_key (hash + 线性探测, 跟 lookup 共享 helper)
//   2. 找到 status = COMPILED + key 匹配 → atomic CAS COMPILED → EMPTY (release);
//      成功 → load host_code_ptr → return
//   3. 找不到 / 非 COMPILED → return NULL (caller 不需 invalidate)
//
// CAS COMPILED→EMPTY 后, 后续 lookup 见 EMPTY 终止探测视 miss; dispatcher 不会
// 再拿到此 host_code. 但已经 hit COMPILED 的 in-flight dispatcher 还在写状态灯
// 跑 host_code — caller 必须:
//   1. jit_rcu_synchronize (等所有 in-flight read critical 出 / 状态灯 visible)
//   2. 扫所有 hart cpu->jit_executing_host_code == host_code, PAUSE 等退出
//   3. backend.invalidate_block(host_code) 真 unmap
//
// idempotent: 撞到 EMPTY/COUNTING/BLACK 返 NULL (entry 已被 invalidate 过 / 还没
// promote / 已黑名单), caller 直接 skip 后续 step 2/3.
// ----------------------------------------------------------------------------
void *jit_cache_take_compiled_host_code(uxlen_t pa, regime_t regime);


// ----------------------------------------------------------------------------
// jit_cache_set_blacklist —— set (PA, regime) 为 BLACK (Q11 a 二次 FULL 路径)
//
// 跟 install 同探测体例 (CAS EMPTY→COUNTING → 填字段 → 挂 page_block_head →
// store BLACK release); 差异:
//   - host_code_ptr = NULL (BLACK 无编译产物)
//   - terminal status = JIT_CACHE_BLACK (lookup 看 BLACK 视 miss + 跳过此 PC)
//
// 调用方 (jit_api 层 Q11 a):
//   s = backend.compile_block(...);  // 二次 FULL
//   if (s == JIT_CODE_CACHE_FULL) {
//       jit_cache_set_blacklist(pa, regime);
//       return JIT_BACKEND_INTERNAL;
//   }
//
// idempotent: 撞到已有 COMPILED 同 key (别 hart 已成功 install) 直接返 0, 不
//   强制覆盖 — BLACK 本意"该 block 永远编不出", 但其他 hart 编出说明能编, race
//   情况下保留 COMPILED 更合理; 撞 BLACK 同 key 也返 0 (idempotent).
//
// BLACK 也挂 page_block_head 链表 (Q12 c: SMC invalidate_page 顺链清 BLACK →
//   EMPTY, 给代码改后新机会编译; flush_all 也清 BLACK).
//
// 返:
//   0  — set BLACK 成功 (或 idempotent 撞同 key COMPILED/BLACK)
//   -1 — 探测 N 步耗尽 (hash table 仍满, 跟 install 一致; caller 通常忽略 — 二次
//        FULL 已经是兜底, 探测 FULL 也只能退 interpreter 跑这个 PC)
// ----------------------------------------------------------------------------
int  jit_cache_set_blacklist(uxlen_t pa, regime_t regime);


// ----------------------------------------------------------------------------
// jit_cache_invalidate_page —— RCU modify-side, sync grace period
//
// 流程:
//   1. atomic_exchange page_block_head[page_idx] = JIT_PAGE_HEAD_END (acq_rel)
//      拿 old_head
//   2. 顺 old_head 链表清: 每 slot atomic store status = EMPTY (release;
//      包括 BLACK → EMPTY, Q12 c)
//   3. jit_rcu_synchronize() — 等所有 hart 出 read critical section
//   4. backend.invalidate_block release host_code RX 段 (asmjit
//      JitRuntime::release)
//
// caller: a_05+ SMC handler 触发 page_dirty bitmap → dispatcher 主循环顶扫
// bitmap → 调 jit_cache_invalidate_page(page_idx); 真端到端 verify 推 a_05+.
// ----------------------------------------------------------------------------
void jit_cache_invalidate_page(uint32_t page_idx);


// ----------------------------------------------------------------------------
// jit_cache_flush_all —— RCU modify-side, sync grace period (整表清)
//
// 流程:
//   1. 扫整张 hash_table[0..JIT_CACHE_SIZE): atomic store status = EMPTY
//      (release; 包括 BLACK)
//   2. 扫 page_block_head[0..GUEST_RAM_NPAGES) 全部 = JIT_PAGE_HEAD_END
//   3. jit_rcu_synchronize()
//   4. backend.flush_all reset code_cache RX 段 (asmjit JitRuntime::reset)
//
// caller: jit_api_flush_all (Q11 a CODE_CACHE_FULL 路径).
// ----------------------------------------------------------------------------
void jit_cache_flush_all(void);


// ----------------------------------------------------------------------------
// EBR (epoch-based reclamation) — read-side 1 cycle 退化为 store 一个 epoch
//
// 设计:
//   - 全局 _Atomic uint32_t jit_rcu_global_epoch (modify-side fetch_add)
//   - per-hart _Atomic uint32_t jit_rcu_slots[MAX_HARTS].epoch (read-side store)
//   - JIT_RCU_INACTIVE = UINT32_MAX 编码 "出 critical section"
//
// read_lock(hart_id):
//   load global_epoch (acquire) → store local_epoch (release)
//   语义: "我进 critical, 看的是全局 g 这个版本"
//
// read_unlock(hart_id):
//   store local_epoch = JIT_RCU_INACTIVE (release)
//   语义: "我出 critical 了"
//
// synchronize:
//   target = fetch_add(&global_epoch, 1) + 1     // 推进 global 一格
//   for (i in 0..n_harts):
//     while (1) {
//       local = load(&slots[i].epoch, acquire)
//       if (local == INACTIVE)  break    // hart 已出 critical
//       if (local >= target)    break    // hart 已重进, 用新 epoch (旧引用已释)
//       sched_yield()
//     }
//   语义: 等所有真 hart 完成"以 target 之前的 epoch 持有的旧引用释放"
//
// caller 协议:
//   - read_lock / read_unlock 配对必须在同一 hart 上下文 (hart_id 是调用方的 hartid)
//   - jit_cache_lookup 路径: caller 进 read_lock → lookup → hit 时 host_code
//     执行完才 read_unlock; miss 时直接 read_unlock 走 fallback
//   - jit_cache_install 路径: install 走 CAS 不进 read_lock (CAS race 不需 RCU
//     保护 — slot 本身永远不 free; RCU 真保护的是 host_code mmap 区)
//   - jit_cache_invalidate_page / flush_all: 内部已调 jit_rcu_synchronize, caller
//     不需进 read_lock
//
// cap-vs-n_harts: 数组 jit_rcu_slots[MAX_HARTS] 用 cap; synchronize 遍历用 n_harts
//   (phantom slot 永远 JIT_RCU_INACTIVE, 走 cap 也不会卡, 但按 §15 判据存在性
//    遍历用 n_harts).
// ----------------------------------------------------------------------------
void jit_rcu_read_lock  (uint32_t hart_id);
void jit_rcu_read_unlock(uint32_t hart_id);
void jit_rcu_synchronize(void);

#ifdef __cplusplus
}
#endif

#endif //JIT_JIT_CACHE_H
