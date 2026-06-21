//
// jit/smc.h —— SMC chain (self-modifying code 失效) 隐式触发路径接口.
//
// ============================================================================
// 设计依据 (b_03 T1.a; b_03_audit_decision.md Q4.1-Q4.5 + plan §1.17)
// ============================================================================
//
// 隐式 SMC 触发路径 (b_03 T1.a 接通; fence.i 显式路径 T2 加 doc, 不动 helper):
//   1. jit_cache_install 一个新 COMPILED slot 时 (store COMPILED release **之前**)
//      调 smc_protect_page(page_idx) → host kernel mprotect 该 page PROT_READ
//   2. 客户机一写 → CPU 触发 SIGSEGV → kernel 走 smc_signal_handler:
//        - async-signal-safe: 只调 _exit / atomic_fetch_or / mprotect (裸 syscall)
//        - 标 page_dirty bitmap (atomic_fetch_or release)
//        - smc_unprotect_page (mprotect 改回 R/W; 让 kernel 重执行触发 fault 的
//          host 指令 — 此次 store 不再 fault)
//   3. dispatcher 主循环顶 (block 跑前) 扫 page_dirty bitmap, smc_consume_dirty
//      逐 bit CAS 清, 每 dirty bit 调 jit_invalidate_page (jit_entry.cc 组合层)
//
// 详 b_03_audit_decision.md Q4.1.a/Q4.2.1/Q4.2.3 + plan §1.17.1/.2/.3.
//
// ============================================================================
// SMP release 顺序 (plan §1.17.3)
// ============================================================================
//
// install 路径 (新发布 JIT 块):
//   1. fill entry 字段
//   2. smc_protect_page (mprotect R-only)
//   3. atomic store status = COMPILED (release)   ← lookup hit 此后
//   完整 race-free 时序解释见 jit_cache.c install 实装注释 + plan §6 of plan file.
//
// SIGSEGV handler 路径:
//   1. atomic_fetch_or page_dirty (release)       ← bitmap 先标
//   2. smc_unprotect_page (mprotect R/W)          ← 然后开锁让 store 成功
//   3. dispatcher 顶扫 → jit_invalidate_page      ← 最后清 JIT 块
//
// 顺序保证 "dispatcher 看到 dirty bit 时, page 已开锁, JIT 块语义仍未失效但即将
// 被清; 客户机的 store 跟 dispatcher 的 invalidate 之间不会撞 RX 段 use-after-free".
//
// ============================================================================
// async-signal-safe 严格遵守 (CLAUDE.md "Do not" 段 + plan §1.17.1)
// ============================================================================
//
// smc_signal_handler 内只允许:
//   - C11 atomic_* (lock-free, POSIX 安全)
//   - 裸 syscall: mprotect / _exit (POSIX async-signal-safe 白名单)
//   - 算术 / 位运算 / 指针计算
//
// 禁止:
//   - malloc / free / printf / fprintf / 任何 stdio
//   - pthread_* (任何锁 / cond / spawn)
//   - 模拟器普通函数 (除非确认完全不用以上禁止项 — smc_unprotect_page 是裸
//     mprotect, 安全; 其余统不调)
//

#ifndef JIT_SMC_H
#define JIT_SMC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


// ----------------------------------------------------------------------------
// smc_init / smc_destroy —— lifecycle (main POR / teardown 一次配对)
//
// smc_init:
//   - page_dirty bitmap 已编译期 0 (static 数组 zero-init)
//   - sigaction 注册 SIGSEGV (SA_SIGINFO + sa_sigaction = smc_signal_handler),
//     保存 old handler 到 file-static old_sigsegv
//   - 失败 (sigaction != 0) 返 -1, fprintf 具体原因; main 处理 (跟 wfi_init /
//     clint_init 同体例)
//
// smc_destroy:
//   - 恢复 SIGSEGV old handler (跟 runtime_restore_signal_handlers 体例)
//   - 不还 mprotect — 进程退出 OS 回收
// ----------------------------------------------------------------------------
int  smc_init(void);
void smc_destroy(void);


// ----------------------------------------------------------------------------
// smc_protect_page —— mprotect host page PROT_READ (caller: jit_cache_install 内)
//
// 必须在 jit_cache_install 的 atomic store COMPILED release **之前** 调; 详
// jit_cache.c install 实装注释 (b_03_session_002 修正时序; race-free 关键).
//
// page_idx = (pa - GUEST_RAM_START) >> 12; host_addr = host_ram_base + page_idx
// * 4096; PROT_EXEC 不要 (guest RAM 在 host 不需 exec, JIT host code 在 asmjit
// JitRuntime 自管 RX 段).
//
// fail (mprotect != 0): T1.a 起步静默 (void)cast; 真撞 fixture 不通过再升 fatal.
// ----------------------------------------------------------------------------
void smc_protect_page(uint32_t page_idx);


// ----------------------------------------------------------------------------
// smc_unprotect_page —— mprotect host page PROT_READ | PROT_WRITE
//
// caller: smc_signal_handler 内 (handler 标完 page_dirty 后调; 让 kernel 重
// 执行触发 fault 的 host 指令 — 此次 store 走 R/W page 成功)
//
// 也可 caller = invalidate / flush 后 cleanup (T1.a 暂不需 — invalidate 后
// page 可能仍被其他 JIT 块用着, 不自动 unprotect; future 真撞需要再加).
//
// mprotect 是 POSIX async-signal-safe (man 7 signal-safety), handler 内可调.
// ----------------------------------------------------------------------------
void smc_unprotect_page(uint32_t page_idx);


// ----------------------------------------------------------------------------
// smc_consume_dirty —— dispatcher 顶扫: 找第一个 set bit + CAS 清, 返 page_idx
//
// caller: dispatcher 主循环顶 (block 跑前 / trap 检查前) while loop:
//   uint32_t page_idx;
//   while (smc_consume_dirty(&page_idx)) {
//       jit_invalidate_page(page_idx);
//   }
//
// 流程:
//   1. 扫 page_dirty[0..GUEST_RAM_NPAGES/64): 找第一个非零 word
//   2. __builtin_ctzll 拿最低 set bit 位置 b
//   3. atomic_fetch_and 清该 bit (acq_rel): 成功 (返回值原值的该 bit = 1) → 返
//      page_idx = w*64 + b + true; 失败 (别 hart 已 consume) → 继续扫
//   4. 全 bitmap 无 set bit → 返 false
//
// 多 hart 安全: CAS 保证 only one hart 拿到任一 dirty bit; "ABA" 不存在 (bit
// 是单设单清, 重复 set 仍 OR=1 idempotent).
// ----------------------------------------------------------------------------
bool smc_consume_dirty(uint32_t *out_page_idx);


#ifdef __cplusplus
}
#endif

#endif //JIT_SMC_H
