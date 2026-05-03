//
// Created by liujilan on 2026/4/28.
// a01_2 dispatcher 模块对外接口 (a_01 简化形态)。
//
// 完整 dispatcher (sigsetjmp + while loop + block 3 派发 + perf 同步等) 等 a_03 trap.c +
// a01_5 interpreter 真接入。当前 a01_2 形态: 一次性走完 block 1 (选叶 TLB) + block 2
// (mmu_translate_pc) + 临时 self-check 输出, 无循环。
//
// 跨文件协议见 src/dummy.txt §1 (sigsetjmp) / §4 (TLB 分发机制)。
//

#ifndef CORE_DISPATCHER_H
#define CORE_DISPATCHER_H

#include "cpu.h"

// 一次"块入口包准备 + block 翻译" 操作 (见 file_plan §1.dispatcher I 区):
//   - 按 priv + xatp 派发选叶 TLB
//   - 通过 mmu_translate_pc 取 pc 对应的 PA + HVA
//   - a_01: 临时 fprintf self-check; 不真派发到 jit / interpreter
//
// 返回值: a_01 范围内随便 (a_01 后 helper 走 exit 走不到这, 不需要传播错误)。
int dispatcher(cpu_t *hart);

#endif //CORE_DISPATCHER_H
