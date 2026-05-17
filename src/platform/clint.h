//
// Created by liujilan on 2026/5/16.
// CLINT (Core-Local Interruptor) 模块对外接口。
//
// 职责: 维护 mtime / mtimecmp[N] / msip[N] 三组 MMIO 寄存器, 经 bus 暴露给 guest。
//        T1 阶段只走 MMIO 读写路径 (写啥读啥), 不真自动推进 mtime, 不真触发
//        timer / soft interrupt — 那是 T4 / T5 的事。
//
// 地址布局 / 寄存器形态 见 config.h CLINT_* 宏 + notes/bus_decision.md §2.8。
// 接口形态 (read/write 返 cause / 0=成功) 见 platform/bus.h + dummy.txt §9。
// 多线程语义 (atomic 字段 / shared 数据) 见 dummy.txt §7。
//

#ifndef PLATFORM_CLINT_H
#define PLATFORM_CLINT_H

#include <stdint.h>     // uint32_t (clint_msip_pending / clint_timer_pending 参数)

// 入口: main.c 在 ram_init 之后, dispatcher 启动之前调一次。
// 内部: 初始化 atomic 字段 (mtime / msip 全清零; mtimecmp 初值 UINT64_MAX 表
//        "no timer interrupt scheduled", 跟 OpenSBI sbi_timer_init 惯例一致 —
//        避免 RV spec §3.2.1 "mtime >= mtimecmp" `0 >= 0` 触发 spurious MTIP)
//        + 填 mmio_dev_t 调 bus_register_mmio 把 CLINT 挂到 bus。
// 失败 -1 (跟 ram_init / cpu_create 同形态报错风格, dummy.txt §5)。
int clint_init(void);


// ----------------------------------------------------------------------------
// 中断 pending 语义查询 — csr.c csr_mip_read 合成路径用 (dummy.txt §6 第 5 类
// _mip_sw 合成读模型)。
//
// 内部 atomic_load_explicit 读 clint 内部 _Atomic 字段 (跨 hart 安全); 调用方
// 不感知 clint 内部存储形态 + 不感知 "mtime 怎么累加" (T2 阶段不真累加, T5 由
// timer 辅助线程异步累加; clint_timer_pending 语义 RV spec §3.2.1 "mtime ≥
// mtimecmp[hartid]" 不变)。
//
// hartid 越界 (>= MAX_HARTS) 返 0 (防御; 不 pending), 不 abort。
// ----------------------------------------------------------------------------
int clint_msip_pending (uint32_t hartid);    /* 0/1; = !!atomic_load(msip[hartid]) */
int clint_timer_pending(uint32_t hartid);    /* 0/1; = (mtime ≥ mtimecmp[hartid]) */

#endif //PLATFORM_CLINT_H
