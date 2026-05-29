//
// Created by liujilan on 2026/5/28.
// WFI 唤醒框架 — per-hart cond+mutex slot + 5 接口函数。
//
// 用途: RV WFI 指令的真实装 (interpreter case OP_WFI 调 wfi_wait 挂起 hart;
// CLINT/PLIC 各 pending 路径在 0→1 翻转处调 wfi_kick 唤醒)。比 NOP 实装省主机
// CPU (不空转 dispatcher), 比 sleep poll 简单 (cond_wait/cond_signal 标准
// pthread 体例)。SMP 之后直接复用, 不需要重写。
//
// 设计要点 (详 plan / session log 015 / trade_off_log 待补):
//   1. per-hart 一对 (mutex, cond) — false sharing 防御 (dummy.txt §14)
//   2. predicate 走 callback (wfi_predicate_fn) — 本模块不依赖 cpu_t/csr/runtime
//   3. cond_timedwait 500ms 兜底 — 防 wfi_kick 漏调时 hart 永睡 (防御性)
//   4. wfi_kick 幂等 — 多次调同 hart 只多醒一次 (hart predicate re-check 自处理)
//   5. wfi_kick 空 waitqueue 时近零成本 (glibc cond_signal 无 waiter 跳 syscall)
//
// 唤醒源 (调用点; auto mode 复查清单):
//   - CLINT timer thread tick: mtip[i] 0→1 翻转 (clint_recompute_mtip)
//   - CLINT guest 写 mtime: 全 hart mtip 重算 (clint_recompute_all_mtip)
//   - CLINT guest 写 mtimecmp[i]: 单 hart mtip 重算
//   - CLINT guest 写 msip[i]: 直接 wfi_kick (msip 不 cache)
//   - PLIC recompute_ctx_eip_locked: eip 0→1 翻转 (plic_ctx_to_hartid 反查)
//
// shutdown 路径不在唤醒源里 (016+17-chat 拍): 哲学 "所有人 self-poll SRS/SDS
// 自己停", hart 在 wfi 时靠 cond_timedwait 兜底 500ms 自醒 + predicate 重检 SRS
// 退出, 接受此 tail latency. signal handler / 非 signal handler 路径一致.
// 详 main.c L606 dispatcher 之后占位段 trail。
//
// 报错风格见 dummy.txt §5; 线程 lifecycle 见 dummy.txt §12; per-hart slot 协议
// 见 dummy.txt §14。
//

#ifndef CORE_WFI_H
#define CORE_WFI_H

#include <stdbool.h>
#include <stdint.h>


// ----------------------------------------------------------------------------
// wfi_predicate_fn — wfi_wait 判定 "是否该醒" 的 callback 签名
//
// closure: caller 透传的不透明指针 (典型 = cpu_t *); 模块不解释。
// 返回:    true = 该醒 (退出 cond_wait); false = 继续睡
//
// 调用时机: wfi_wait 内部 mutex hold 下, 进 cond_wait 前 + 每次 wake (timeout /
// kick / spurious) 后。predicate 必须只读 (无副作用), 且自带 atomic / mutex
// 保证读到的状态满足 memory_order_acquire (跟 wfi_kick 在 mutex hold 下 signal 配对)。
// ----------------------------------------------------------------------------
typedef bool (*wfi_predicate_fn)(void *closure);


// ----------------------------------------------------------------------------
// lifecycle: wfi_init / wfi_destroy
// ----------------------------------------------------------------------------

// POR 调一次 (main, cpu_create 之后, clint_start_timer_thread 之前 —
// timer thread 一旦跑就可能调 wfi_kick, 必须 slots 已 init)。
// 内部对每 hart slot 调 pthread_mutex_init + pthread_cond_init。
// 失败返 -1 (内部已 fprintf), 调用方走 cleanup + return。
int wfi_init(void);

// POR 收尾调 (main, 所有 wfi_kick 来源已停: timer thread 已 join, clint/plic
// destroy 已不再调 kick)。对每 hart slot 调 pthread_cond_destroy +
// pthread_mutex_destroy。NULL 入参 do nothing (无入参; 无 op)。
void wfi_destroy(void);


// ----------------------------------------------------------------------------
// 接口函数 — caller (interpreter / clint / plic / main) 调用
// ----------------------------------------------------------------------------

// wfi_wait — 阻塞当前线程, 直到 predicate(closure) 返 true。
//
// hartid: 索引 wfi_slots[]; 越界 do nothing 立即返 (防御; caller 应已 bound check)
// pred:   predicate callback (typedef wfi_predicate_fn)
// closure: 透传给 pred (典型 = cpu_t *)
//
// 行为:
//   lock(slot.mutex)
//   while (!pred(closure))
//       cond_timedwait(slot.cond, slot.mutex, 500ms)    /* spurious wake / timeout / kick 都重 check */
//   unlock(slot.mutex)
//
// 阻塞: 直到 predicate 返 true; 500ms timeout 是兜底 (防 kick 漏调时永睡), 不
// 代表唤醒频率 (有 kick 时立即唤; 无中断时每 500ms 重 check predicate, 一般
// false 又睡回去)。
//
// 调用约束: 只能在普通线程上下文调 (pthread_mutex_lock 不是 async-signal-safe);
// signal handler 不能调本函数。
void wfi_wait(uint32_t hartid, wfi_predicate_fn pred, void *closure);

// wfi_kick — 唤醒可能在 wfi_wait 的 hart (lock + cond_signal + unlock)。
//
// hartid: 索引 wfi_slots[]; 越界 do nothing
//
// 幂等: hart 未在 wait → cond_signal 是 noop (glibc 空 waitqueue 跳 syscall);
// hart 在 wait → 醒来 predicate re-check (false 则又睡回去, 一次假醒可接受)。
//
// 调用约束: 普通线程上下文; signal handler 不能调 (pthread_mutex_*)。
void wfi_kick(uint32_t hartid);

// wfi_kick_all — 唤醒所有 hart (for 循环逐个调 wfi_kick)。
//
// 016+17-chat 拍: shutdown 路径不调本函数, 接受 hart 在 wfi 时 cond_timedwait
// WFI_TIMEOUT_NS (config.h 500ms) 兜底自醒 + predicate 重检 SRS 退出 (详 main.c
// L606 之后占位段 trail). 当前无调用者, 函数保留备多 hart / 灵感 — 候选
// (a) hart 自治 kick (本对话提议; 先醒 hart kick 其余) (b) source 端 kick
// (CLINT/PLIC stop / main spawn-join 范围内 kick; v2/v4 思路, 均撤). 真撞延迟
// 问题再回看. 线程安全: for 循环展开 N 个 per-slot mutex kick, 多线程并发调
// 同 slot 自动串行 (pthread_mutex), 无冲突。
//
// 调用约束: 同 wfi_kick (非 signal handler)。
void wfi_kick_all(void);


#endif //CORE_WFI_H
