//
// Created by liujilan on 2026/4/28.
// dispatcher 模块对外接口。
//
// dispatcher 是 hart 主循环: sigsetjmp 一次性建立永久落点 + while(SRS==0) 多块循环 +
// 每轮迭代头做扫尾 (count 累加 / 未来 mtime 推进 / 中断检查 / perf_advance)。helper 端
// 的 longjmp (mmu_walker_helper_* / mmio_*_helper / trap_raise_exception 等) 都跳回这
// 个入口落点;
// main 调一次 dispatcher 返回时 hart 已 halt (halt 由 system_reset_signal 任一 bit
// 触发: HART_MDT / TEST_RESET / SHUTDOWN_TRIGGER / DEVICE_FAIL; bit 编码见 runtime.h)。
//
// 跨文件协议见 src/dummy.txt §1 (sigsetjmp) / §4 (TLB 分发机制)。
//

#ifndef CORE_DISPATCHER_H
#define CORE_DISPATCHER_H

#include "cpu.h"

// hart 主循环。返回时 hart 已 halt; halt 状态由 system_reset_signal 表达 (bit 编码
// 见 runtime.h; main 端按 ABORT_MASK 决定 cleanup return 还是 try_clear continue)。
void dispatcher(cpu_t *hart);

#endif //CORE_DISPATCHER_H
