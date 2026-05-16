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

// 入口: main.c 在 ram_init 之后, dispatcher 启动之前调一次。
// 内部: 初始化 atomic 字段 (mtime / mtimecmps / msip 全清零) + 填 mmio_dev_t 调
//        bus_register_mmio 把 CLINT 挂到 bus。
// 失败 -1 (跟 ram_init / cpu_create 同形态报错风格, dummy.txt §5)。
int clint_init(void);

#endif //PLATFORM_CLINT_H
