//
// Created by liujilan on 2026/6/7.
// jit/jit_cache.h —— JIT 块缓存 (PA, regime) → host_code_ptr 的 hash table.
//
// ============================================================================
// 设计依据 (b_01 session_003 audit 拍法; jit_plan_audit_decision.md Q9/Q10/Q11/Q12)
// ============================================================================
//
// 三个核心拍法 (b_01 T2 一步到位):
//   Q9 方案 2  — per-page block list: hash entry 加 next_in_page 链表字段 +
//                全局 page_block_head[GUEST_RAM_NPAGES] 链表头数组; install
//                CAS 挂头, invalidate_page 顺链表清 (跟 hash table 全扫的方案 1
//                比 invalidate O(块/页) vs O(全 cache))
//   Q10 c RCU — EBR (epoch-based reclamation) 自实现, lookup read-side 无锁;
//                install / invalidate_page / flush_all 走 modify-side, 等
//                grace period 后才允许 backend 真 unmap host_code mmap 区
//                (b_03 SMC 真触发时 backend.invalidate_block 才真做)
//   Q11 a    — jit_api 层 Flush + retry 1 次, dispatcher 不感知 Flush;
//                jit_cache_install 返 -1 (FULL) 时 caller (jit_api) 触发
//                jit_flush_all + 再 install 重试 1 次 (T2 阶段 backend stub
//                返 NOT_IMPLEMENTED 跑不通完整 retry 形态, 留 T3 真做时拍)
//   Q12 c    — invalidate_reason_t 入参 b_01 T2 不带 (audit 拍"不强求"); 现役
//                两条路径 (SMC invalidate / Flush_all) 都清 BLACK; 未来加
//                block chaining / SMC 立即重编时 ABI append-only 扩 reason 入参
//
// SMP 一次性到位原则 (S6; plan §1.9 末段 + dummy.txt §15 末段):
//   SMP 已落地 (a_03 #205-#209), 后续 SMP 相关设计必须一次性到位; jit_cache
//   出生即 lock-free + RCU, 不走"先单 hart 后补 SMP".
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
// jit_cache_*    — hash table + per-page list 操作; caller = jit_api.c 组合层
//                   (jit_init / jit_flush_all forward) + main 临时 self-check
//                   (T2 阶段直接 #include 进 main; 完事段删)
//   jit_cache_init / destroy           — lifecycle (main POR / 退出段调)
//   jit_cache_lookup                   — RCU read-side (caller 已调 jit_rcu_read_lock)
//   jit_cache_install                  — RCU modify-side (atomic CAS slot status)
//   jit_cache_invalidate_page          — RCU modify-side (sync grace period)
//   jit_cache_flush_all                — RCU modify-side (sync grace period)
//
// jit_rcu_*      — EBR critical section; caller = dispatcher (T4 fork 点真接) +
//                   modify-side 函数自身 (jit_rcu_synchronize)
//   jit_rcu_read_lock / read_unlock    — read critical section enter / exit
//                                         (1 cycle 退化为 store 一个 epoch)
//   jit_rcu_synchronize                — modify-side 等所有 hart 出 critical
//
// ============================================================================
// 命名 (session_002 拍)
// ============================================================================
//
// 类型 jit_cache_*_t (jit_cache_status_t / jit_cache_entry_t); enum 值
// JIT_CACHE_* (JIT_CACHE_EMPTY / JIT_CACHE_COUNTING / JIT_CACHE_COMPILED /
// JIT_CACHE_BLACK); 函数 jit_cache_* / jit_rcu_* (snake_case, module_verb_object).
//

#ifndef JIT_JIT_CACHE_H
#define JIT_JIT_CACHE_H

#include <stdatomic.h>
#include <stdint.h>

#include "core/mmu.h"   // regime_t (key 第二维; mmu.h 不重复 include riscv.h)
#include "riscv.h"      // uxlen_t (key_pa; dummy.txt §13 typedef family)


// ----------------------------------------------------------------------------
// jit_cache_status_t —— hash entry 状态机 (b_01 T2 4 状态)
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
//   EMPTY → BLACK    (backend fail 路径; jit_api 层 set; T3 真做时拍最终路径)
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
// jit_cache_entry_t —— 单 hash slot (b_01 T2; ~32 byte; 64K slot × 32 = 2MB)
//
// 字段:
//   key_pa         — 块入口 PA (uxlen_t; dummy.txt §13)
//   key_regime     — baked priv 视角 (BARE / SV32_S / SV32_U; core/mmu.h)
//   status         — 状态机 (atomic; load acquire / store release)
//   counter        — 解释器执行次数计数 (热度阈值; plan §1.23.8; 非 atomic, CAS
//                     status COUNTING 后 owner 独占写)
//   host_code_ptr  — backend 编译产物入口 (类型 jit_block_func_t = void (*)
//                     (cpu_t*, tlb_p, uint32_t*); Q1 a; T2 阶段 stub 假指针, T3
//                     backend 真做时填真 mmap RX 段)
//   next_in_page   — per-page block list 链表 next 索引 (uint16_t; JIT_CACHE_SIZE
//                     = 65536 索引正好用满 uint16_t; 0xFFFF = 链尾哨兵 = JIT_PAGE_HEAD_END)
//
// 字段非 atomic (key_pa / key_regime / counter / host_code_ptr / next_in_page):
//   install CAS status EMPTY→COUNTING 后 owner 独占写这些字段; release store
//   status = COMPILED 配 lookup acquire load 形成 happens-before, lookup 看到
//   COMPILED 时其他字段已 visible.
// ----------------------------------------------------------------------------
typedef struct {
    uxlen_t           key_pa;
    regime_t          key_regime;
    _Atomic uint32_t  status;          // jit_cache_status_t
    uint32_t          counter;
    void             *host_code_ptr;
    uint16_t          next_in_page;
} jit_cache_entry_t;


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
//     (host_code 执行过程 host_code mmap 区 RX 不能被 backend 真 unmap;
//      invalidate 路径 jit_rcu_synchronize 等出 critical 才允许 backend unmap)
//   - miss 路径: 调 jit_rcu_read_unlock 后走 interpreter fallback (T4 真接时)
// ----------------------------------------------------------------------------
jit_cache_entry_t *jit_cache_lookup(uxlen_t pa, regime_t regime);


// ----------------------------------------------------------------------------
// jit_cache_install —— RCU modify-side, atomic CAS 路径无锁
//
// 流程:
//   1. idx = jit_cache_hash(pa, regime)
//   2. 线性探测 ≤ JIT_INSTALL_MAX_PROBES 步找 EMPTY slot:
//      - status atomic CAS EMPTY → COUNTING
//        - 失败 = 别 hart 抢先 / status 非 EMPTY → 继续探测下一槽
//        - 成功 = owner 独占该 slot, 进 step 3
//   3. 填 key_pa / key_regime / host_code_ptr / counter / next_in_page (非 atomic)
//   4. atomic CAS 挂 page_block_head 链表头 (退栈式, do-while):
//        do { old_head = load(head); entry.next_in_page = old_head; }
//        while (!CAS(head, &old_head, idx));
//   5. atomic store status = COMPILED (release; 让 lookup 看到 COMPILED 时
//      其他字段已 visible)
//
// 返:
//   0  — install 成功 (status = COMPILED)
//   -1 — install FULL (探测 N 步都没 EMPTY slot → hash table 满 → caller Flush)
//
// caller (jit_api 层) 处理 -1 (Q11 a): 调 jit_flush_all + 再调 install 重试 1 次;
// 二次 -1 进 BLACK (set status = BLACK; 跟 IR_ERROR / BACKEND_INTERNAL 同处理).
// T2 阶段 jit_api stub backend 跑不通 retry, T3 backend 真做时拍最终形态.
// ----------------------------------------------------------------------------
int  jit_cache_install(uxlen_t pa, regime_t regime, void *host_code_ptr);


// ----------------------------------------------------------------------------
// jit_cache_invalidate_page —— RCU modify-side, sync grace period
//
// 流程:
//   1. atomic_exchange page_block_head[page_idx] = JIT_PAGE_HEAD_END (acq_rel)
//      拿 old_head
//   2. 顺 old_head 链表清: 每 slot atomic store status = EMPTY (release;
//      包括 BLACK → EMPTY, Q12 c)
//   3. jit_rcu_synchronize() — 等所有 hart 出 read critical section
//   4. (T2 stub: backend.invalidate_block 是 nop) backend 真做后 unmap host_code
//      mmap 区
//
// caller: b_03 SMC handler 触发 page_dirty bitmap → dispatcher 主循环顶扫
// bitmap → 调 jit_cache_invalidate_page(page_idx); T2 阶段 self-check 模拟调用.
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
//   4. (T2 stub: backend.flush_all 是 nop) backend 真做后 reset code_cache mmap 区
//
// caller: jit_api_flush_all (Q11 a CODE_CACHE_FULL 路径; T3 backend 真做时拍).
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


#endif //JIT_JIT_CACHE_H
