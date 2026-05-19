//
// Created by liujilan on 2026/5/16.
// CLINT (Core-Local Interruptor) 模块对外接口。
//
// 职责: 维护 mtime / mtimecmp[N] / msip[N] 三组 MMIO 寄存器, 经 bus 暴露给 guest。
//        T5 起 mtime 由独立 timer 辅助线程 (clint.c file-static timer_run) 异步累
//        加 (方案 C); dispatcher / interpreter 主帧只读, 不写。
//
// 地址布局 / 寄存器形态 见 config.h CLINT_* 宏 + notes/bus_decision.md §2.8。
// 接口形态 (read/write 返 cause / 0=成功) 见 platform/bus.h + dummy.txt §9。
// 多线程语义 (atomic 字段 / shared 数据 + monitor 模型) 见 dummy.txt §7。
// timer thread spawn / join 协议 (谁 spawn 谁 join) 见 dummy.txt §12。
//
// clint = 完整 monitor 实例:
//   consumer 接口: is_clint_msip_pending / is_clint_timer_pending / clint_read (bus)
//   producer 接口: clint_write (bus)
//   lifecycle:    clint_init  (POR; atomic 字段 + bus 注册; **不发线程**)
//                 clint_reset (system reset 每 iter; mtimecmp/msip 清, mtime/timer 不动)
//                 clint_destroy (POR 收尾纯 cleanup; **不含 pthread_join**)
//   thread lifecycle (跟 lifecycle 解耦, dummy.txt §12 谁 spawn 谁 join):
//                 clint_start_timer (main 调; pthread_create timer 辅助线程)
//                 clint_join_timer  (main 调; 等 SDS=0 后 pthread_join)
//

#ifndef PLATFORM_CLINT_H
#define PLATFORM_CLINT_H

#include <stdint.h>     // uint32_t (is_clint_*_pending 参数)

// ----------------------------------------------------------------------------
// lifecycle
// ----------------------------------------------------------------------------

// POR 一次性 init (main 入口, ram_init 之后): atomic 字段清 (mtime=0, mtimecmp=
// UINT64_MAX 哨兵跟 OpenSBI sbi_timer_init 一致, msip=0) + 填 mmio_dev_t 调
// bus_register_mmio 把 CLINT 挂到 bus。**不发 timer 线程** (那由 main 显式调
// clint_start_timer)。
// 失败 -1 (跟 ram_init / cpu_create 同形态报错风格, dummy.txt §5)。
int clint_init(void);

// system reset 每 iter 调 (main while 顶段): mtimecmps[N] = UINT64_MAX, msip[N] = 0
// (跟 init 同初值); **mtime 不动, timer 辅助线程不动** (真硬件 RTC oscillator
// 不掉电不停; reset 三层 lifecycle 见 start_plan_a_02.md §T5 + a_02_t5_plan.md §3)。
// 当前实装返 0 (失败路径预留, 跟 clint_init 同签名)。
int clint_reset(void);

// POR 收尾 (main 末段, ram_destroy 之前): 纯模块 cleanup, 不含 pthread_join
// (timer thread 由 main 调 clint_join_timer 显式回收; 见 dummy.txt §12)。
// 当前实际工作量极小 (atomic 字段清 0 占位 lifecycle 完整, bus 未来加
// unregister 时这里调); 函数留作 lifecycle 对称。
void clint_destroy(void);


// ----------------------------------------------------------------------------
// thread lifecycle (T5 加; dummy.txt §12 谁 spawn 谁 join)
// ----------------------------------------------------------------------------

// main 调一次 (clint_init 之后, while 之前): pthread_create timer 辅助线程,
// thread routine 跨 system reset 一直跑 (随 SDS 起停), 异步 atomic_fetch_add
// clint.mtime + TIMEBASE_PER_WAKE 每 TIMER_WAKE_INTERVAL_NS。
// 失败 -1 (内部 fprintf), main 看到走 exit; pthread_t 句柄存 clint.c file-
// static struct clint 内部, 供 clint_join_timer 用。
int clint_start_timer(void);

// main 调一次 (while 退出后, clint_destroy 之前): 等 timer 辅助线程看到
// SDS=0 自然退后 pthread_join 回收。pthread_join 失败 fprintf 不 fatal (已在
// 退出路径)。
//
// 调用前置: main 已 atomic_store(&shutdown_signal, 0) 通知 timer thread 退;
// 否则 pthread_join 永远 block (timer thread 在 while(SDS) 内永不退)。
void clint_join_timer(void);


// ----------------------------------------------------------------------------
// 中断 pending 语义查询 — csr.c csr_mip_read 合成路径用 (dummy.txt §6 第 5 类
// _mip_sw 合成读模型)。
//
// 内部 atomic_load_explicit 读 clint 内部 _Atomic 字段 (跨 hart 安全); 调用方
// 不感知 clint 内部存储形态 + 不感知 "mtime 怎么累加" (T5 起 timer 辅助线程
// 异步累加; is_clint_timer_pending 语义 RV spec §3.2.1 "mtime ≥ mtimecmp[hartid]"
// 不变)。
//
// hartid 越界 (>= MAX_HARTS) 返 0 (防御; 不 pending), 不 abort。
// ----------------------------------------------------------------------------
int is_clint_msip_pending (uint32_t hartid);    /* 0/1; = !!atomic_load(msip[hartid]) */
int is_clint_timer_pending(uint32_t hartid);    /* 0/1; = (mtime ≥ mtimecmp[hartid]) */

#endif //PLATFORM_CLINT_H
