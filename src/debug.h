//
// Created by liujilan on 2026/5/12.
// debug 模块: 全局 trace tick counter + DEBUG_XXX 宏 (单字符流式输出到 stderr)。
//
// 设计目的: dispatcher / trap / 中断 路径上的"重新取指 / 异常 / 中断"事件流式
// 输出单字符到 stderr; 配合 fixture 人工观察跨页 / 中断密度, 不接入正式 unit test。
// 换行时机由调用方拍 (典型: dispatcher 退出 dump 前打一次 fputc('\n', stderr))。
//
// debug_cnt 不是 trap nesting / count_out; 是开发期 tick 累加, 仅作"已发生多少
// trace 事件"参考。
//
// ----------------------------------------------------------------------------
// 多线程 vs 多 HART 术语 (本项目协议; 跟 dummy.txt §7 镜像, 内容一致两边都看)
//
//   hart        — RV guest hart (architectural concept)。一个 hart 对应一份 cpu_t;
//                 SMP 真做时对应一个 host pthread; 跑 guest 指令的"线程实例"。
//   host thread — OS 级 pthread, 分两类:
//                   (a) hart 线程: 每 hart 一个, 持 cpu_t, 跑 dispatcher
//                   (b) 辅助线程: 不持 cpu_t, 不跑 guest, 只动 shared 数据
//                                例 timer 线程 (写 clint.mtime); 未来 monitor
//                                线程 (读 perf shm)
//
// 数据按线程归属分:
//   per-hart    : cpu_t (regs / trap_csrs / tlb_table / satp / ...)。只本 hart
//                 线程读写, 不需 atomic。
//   shared      : clint.mtime / clint.mtimecmps[N] / page_dirty bitmap / jit_cache
//                 表头。多 hart 线程 + 辅助线程都可能读写, 需 atomic。
//   thread-local: debug_cnt 等 host trace 状态。当前 hart 线程独占 (单 hart);
//                 SMP 多 hart 时若要"全局密度"需 atomic / 若要"per-hart 密度"
//                 改 per-hart 字段 (T5 时确认)。
//
// 关键: "多 hart" ≠ "多线程"。timer / monitor 等辅助线程数不算 hart 数 (它们不持
// cpu_t, 不影响 misa / mhartid 等 hart-counting 属性)。
// ----------------------------------------------------------------------------

#ifndef DEBUG_H
#define DEBUG_H

#include <stdint.h>
#include <stdio.h>

// 全局 trace tick counter。单 hart 单线程下安全; SMP 多 hart 时按上方
// "thread-local"分类策略改 atomic 或 per-hart 字段 (T5 时确认)。
extern uint32_t debug_cnt;

// DEBUG_TICK_TH: trace 流自动换行阈值 (每 N 个事件后 fputc('\n'))。80 按经典终端
// 宽度, 不强制; 改大改小看 fixture 密度调。
#define DEBUG_TICK_TH  80

// DEBUG_TICK(): 内部 tick — 累加 debug_cnt + 到阈值打 \n。各字符事件宏内部统一调
// 它, 不作"用户事件"独立暴露。
#define DEBUG_TICK()   do {                                      \
    debug_cnt++;                                                 \
    if ((debug_cnt % DEBUG_TICK_TH) == 0) fputc('\n', stderr);   \
} while (0)

// DEBUG_XXX 字符事件宏: 写单字符到 stderr + DEBUG_TICK (tick 内部判换行 → \n 跟在
// 字符后, 即"本行末尾"位置)。
//
// 事件 ↔ 字符映射 (字符可视觉调整, 不影响语义; 各 fixture 注释里仍用 fetch/check 概念
// 词描述 trace, 字符层是实现细节):
//   fetch (历史 'f' / 现 '_')  — dispatcher refetch (每轮 while 体进入, 真 fetch 即将发生)
//   check (历史 'c' / 现 '.')  — trap_check_interrupt 入口 ready==0 (poll 但未 fire)
//   exception ('E')           — sync trap raise (大写表 sync exception 严重)
//   timer ('t')               — timer interrupt fire
//   soft ('s')                — software interrupt fire
//   external ('e')            — external interrupt fire
//
// 字符选取依据 (调整时考虑视觉密度):
//   - fetch 是真"做事" (块执行), 给粗字符 ('_') 让动作可见
//   - check 是 poll 决定, 给细字符 ('.') 作背景, 不淹没 fire 事件
//   - fire / exception (t/s/e/E) 是粗字符, 跟 '_' 粗细近, 在 '.' 背景上跳出明显
//
// 互斥协议 (dispatcher 主帧每轮 while 体):
//   每轮 trap_check_interrupt 入口必出且仅出一个字符 (check 或 fire) —
//     ready == 0 → 本函数内打 check 字符 ('.')
//     ready != 0 → 不在本函数打, 由 trap_set_interrupt_state 内按 cause_low 打 t/s/e
//   随后:
//     check 返 0 → dispatcher 调 DEBUG_REFETCH 打 fetch 字符 ('_')
//     check 返非 0 → dispatcher continue 跳过 DEBUG_REFETCH, 本 iter 只一个 fire 字符
//   即每 iter 至少 1 字符 (fire 单字符) 至多 2 字符 (check + fetch).
//
//   trace 形态示例 (用字符):
//     '._._._._._.t._._.' = N 轮无 fire (check+fetch 对) → 一轮 timer fire (只 't', 无后续 fetch
//                            因 continue) → 下一轮 N+1 又 check+fetch (handler 内 MIE=0 → check
//                            no fire).
#define DEBUG_REFETCH()    do { fputc('_', stderr); DEBUG_TICK(); } while (0)
#define DEBUG_EXCEPTION()  do { fputc('E', stderr); DEBUG_TICK(); } while (0)
#define DEBUG_INT_CHECK()  do { fputc('.', stderr); DEBUG_TICK(); } while (0)
#define DEBUG_TIME_INTR()  do { fputc('t', stderr); DEBUG_TICK(); } while (0)
#define DEBUG_SOFT_INTR()  do { fputc('s', stderr); DEBUG_TICK(); } while (0)
#define DEBUG_EXT_INTR()   do { fputc('e', stderr); DEBUG_TICK(); } while (0)

#endif //DEBUG_H
