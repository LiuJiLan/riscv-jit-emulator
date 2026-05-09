//
// Created by liujilan on 2026/4/28.
// dispatcher 模块对外接口。
//
// dispatcher 是 hart 主循环: sigsetjmp 一次性建立永久落点 + while(in_trap<3) 多块循环 +
// 每轮迭代头做扫尾 (count 累加 / 未来 mtime 推进 / 中断检查 / perf_advance)。helper 端
// 的 longjmp (mmu walker / store_helper / trap_raise_exception 等) 都跳回这个入口落点;
// main 调一次 dispatcher 返回时 hart 已 halt (halt 状态由 hart->trap.in_trap 表达,
// 位段编码见 dispatcher.c 末尾)。
//
// 跨文件协议见 src/dummy.txt §1 (sigsetjmp) / §4 (TLB 分发机制)。
//

#ifndef CORE_DISPATCHER_H
#define CORE_DISPATCHER_H

#include "cpu.h"

// hart 主循环。返回时 hart 已 halt; halt 状态由 hart->trap.in_trap 表达 (位段编码见
// dispatcher.c 末尾 in_trap 位段编码段)。
void dispatcher(cpu_t *hart);

#endif //CORE_DISPATCHER_H
