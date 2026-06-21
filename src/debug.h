//
// debug 模块: per-hart trace buffer + DEBUG_XXX 宏 (字符 append 到 buffer,
// flush 时一把 fwrite 到 stderr; SMP per-thread 隔离免 race)。
//
// 设计目的: dispatcher / trap / 中断 路径上的"重新取指 / 异常 / 中断"事件以
// 单字符形式累积到 per-hart trace_buf; 配合 fixture 人工观察跨页 / 中断密度,
// 不接入正式 unit test。
//
// flush 时机:
//   - dispatcher 退出前 (主 while 结束后) DEBUG_NEWLINE 触发 flush + EOL,
//     让 trace 流跟后面 [perf] / [dispatcher] halted 各占干净一行;
//   - hart_exec_run 末尾再调一次 debug_flush_local_trace 兜底 (空 buffer no-op)。
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
//   thread-local: trace_buf / trace_idx (本文件) 等 host trace 状态。__thread
//                 storage class, 每 hart 线程自动一份独立 buffer; hart 退出前
//                 flush 一把到 stderr。timer / monitor 等辅助线程不调 trace 宏,
//                 各自的 __thread 实例永远为空, 仅占少量 TLS (~4 KB/线程)。
//
// 关键: "多 hart" ≠ "多线程"。timer / monitor 等辅助线程数不算 hart 数 (它们不持
// cpu_t, 不影响 misa / mhartid 等 hart-counting 属性)。
// ----------------------------------------------------------------------------

#ifndef DEBUG_H
#define DEBUG_H

#include <stddef.h>   // size_t
#include <stdint.h>   // uint32_t (hartid_self)
#include <stdio.h>

#include "config.h"   // EOL 宏 (项目级 stderr 输出体例 = "\r\n")

// per-hart trace buffer (__thread storage)。trace_buf 满 (idx == DEBUG_BUF_SIZE)
// 时丢字符不报错, trace 是 best-effort 辅助信息 — 真要看完整密度调大 DEBUG_BUF_SIZE。
#define DEBUG_BUF_SIZE 4096

extern __thread char trace_buf[DEBUG_BUF_SIZE];
extern __thread size_t trace_idx;

// hartid_self — 当前 hart 线程的 hart id, 给 debug_flush_local_trace 打 prefix。
// 在 hart_exec_run 入口 set 一次 (hartid_self = ((cpu_t *)arg)->hartid)。主线程 /
// 辅助线程 (timer / uart_reader / virtio_io_worker) 不调 trace 宏, hartid_self
// 默认 0 也不被读 — 不影响其他 hart。
extern __thread uint32_t hartid_self;

// debug_buf_putc — append 单字符到当前线程的 trace_buf; 满了丢字符 (best-effort)。
// inline 让各 DEBUG_XXX 宏调用零开销 (编译器内联 + __thread 寻址)。
static inline void debug_buf_putc(char ch) {
    if (trace_idx < DEBUG_BUF_SIZE) {
        trace_buf[trace_idx++] = ch;
    }
}

// debug_flush_local_trace — flush 当前线程 trace_buf 到 stderr + EOL 分块,
// 重置 idx。空 buffer no-op。实装在 debug.c。
extern void debug_flush_local_trace(void);


// ----------------------------------------------------------------------------
// 调试打印编译标志 (5 个; 由 CMakeLists.txt 按 build type 定义, 不在源码内 #define)
//
//   DEBUG_TRACE_ON       — DEBUG_REFETCH / DEBUG_EXCEPTION / DEBUG_INT_CHECK /
//                          DEBUG_TIME/SOFT/EXT_INTR / DEBUG_NEWLINE
//                          (instruction-level fetch / interrupt fire trace 字符流;
//                           dispatcher / trap 用; 密集, 跟主循环每 iter 同频)
//   DEBUG_TRACE_WFI_ON   — DEBUG_WFI_ISSUE / DEBUG_WFI_SLEEP / DEBUG_WFI_WAKE
//                          (WFI 事件 trace 字符 'w' / 'S' / 'W'; interpreter / wfi 用;
//                           独立 gate — 跟 DEBUG_TRACE_ON 解耦, 因 WFI 事件粒度稀疏,
//                           fixture 测试时可单开 WFI trace 不要全 instruction trace,
//                           或反过来全开 trace 但关 WFI; 独立 gate)
//   DEBUG_PERF_ON        — dispatcher [perf] 主循环计时行 (dispatcher.c 内 #ifdef)
//   DEBUG_CPU_DUMP_ON    — cpu_destroy 内 CPU 寄存器/trap/state dump (cpu.c cpu_dump)
//   DEBUG_CLINT_TIMER_ON — [clint timer] stopped 行 (clint.c timer_log_stop)
//
// 机制: CMakeLists.txt add_compile_definitions 按配置发 -D ——
//   非 Release 配置 (Debug / Tsan 等): 五个全开 (GUI build, 全打印, 给人工观察)。
//   Release 配置: 只开 DEBUG_PERF_ON —— 自动化 perf 套件读 [perf] 的纯主循环 MIPS,
//     trace / WFI / CPU dump / [clint timer] 关掉, stderr 写入不污染 [perf] 计时。
//
// 范围: 这些标志只 gate "调试打印"。不影响各模块报错 fprintf / [dispatcher] halted /
//   [main] elapsed / [decode_test] —— 那些是诊断 / 报错输出, 常开。
// ----------------------------------------------------------------------------

// DEBUG_XXX 字符事件宏: append 单字符到 per-hart trace_buf (debug_buf_putc)。
// flush 由 DEBUG_NEWLINE / debug_flush_local_trace 控制, macro 内不直接 stderr。
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
// (WFI trace 字符 'w' / 'S' / 'W' 由 DEBUG_TRACE_WFI_ON gate 控制, 见下方独立段;
//  共享同一 trace_buf, 跟主组字符夹在一起, 按 in-thread 时序排列。)
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
//   即每 iter 至少 1 字符 (fire 单字符) 至多 2 字符 (check + fetch)。
//
//   trace 形态示例 (用字符):
//     '._._._._._.t._._.' = N 轮无 fire (check+fetch 对) → 一轮 timer fire (只 't', 无后续 fetch
//                            因 continue) → 下一轮 N+1 又 check+fetch (handler 内 MIE=0 → check
//                            no fire)。
#ifdef DEBUG_TRACE_ON

#define DEBUG_REFETCH()    do { debug_buf_putc('_'); } while (0)
#define DEBUG_EXCEPTION()  do { debug_buf_putc('E'); } while (0)
#define DEBUG_INT_CHECK()  do { debug_buf_putc('.'); } while (0)
#define DEBUG_TIME_INTR()  do { debug_buf_putc('t'); } while (0)
#define DEBUG_SOFT_INTR()  do { debug_buf_putc('s'); } while (0)
#define DEBUG_EXT_INTR()   do { debug_buf_putc('e'); } while (0)
#define DEBUG_JIT_HIT()    do { debug_buf_putc('J'); } while (0)

// DEBUG_NEWLINE() — 触发当前线程 trace_buf flush 到 stderr + EOL 分块。
// dispatcher 退出前调一次, 让 trace 流跟后面 [perf] / [dispatcher] halted 各占
// 干净一行。hart_exec_run 末尾再调 debug_flush_local_trace 兜底 (空 buffer no-op)。
#define DEBUG_NEWLINE()    debug_flush_local_trace()

#else  /* DEBUG_TRACE_ON 未定义 — trace 全部退化为 no-op (零开销, 不进 .text) */

#define DEBUG_REFETCH()    do { } while (0)
#define DEBUG_EXCEPTION()  do { } while (0)
#define DEBUG_INT_CHECK()  do { } while (0)
#define DEBUG_TIME_INTR()  do { } while (0)
#define DEBUG_SOFT_INTR()  do { } while (0)
#define DEBUG_EXT_INTR()   do { } while (0)
#define DEBUG_JIT_HIT()    do { } while (0)
#define DEBUG_NEWLINE()    do { } while (0)

#endif /* DEBUG_TRACE_ON */


// ----------------------------------------------------------------------------
// DEBUG_TRACE_WFI_ON gate — WFI 事件 trace 字符 (独立 gate, 共享 trace_buf)
// ----------------------------------------------------------------------------
//
// 独立 gate, 因 WFI 事件 (case OP_WFI 入口 / 进 cond_wait / wfi_wait 返回)
// 是 hart-coarse 事件 (单 fixture 通常只几次), 跟 dispatcher fetch/check 字符的
// 粒度差几个数量级。独立 gate 让 fixture 测试可以:
//   - 只开 WFI trace, 不打 fetch/check 海量字符 — 验 WFI 行为不被噪声淹
//   - 反过来开全 trace 但关 WFI — 调 dispatcher 主路径不被 WFI 字符干扰
//
// 字符语义:
//   wfi issue ('w') — case OP_WFI 入口 (interpreter.c, 任何 WFI 指令到达就打,
//                      包括 TW illegal 短路的; 表 "wfi 指令发出, 想 wfi")
//   wfi sleep ('S') — 真进 cond_wait 之前 (wfi.c, 在 while loop body 内, predicate
//                      进入即 true 时不打; 表 "真睡了"); spurious wake + 仍 false
//                      再 cond_wait → 多打 'S'
//   wfi wake  ('W') — wfi_wait 返回后 (interpreter.c, 不纠结这个唤醒真不真都打)
//
// 3-char 组合解读:
//   'wW'    = wfi 指令但 predicate 进入即真, 没睡 (fixture 03/04 模式)
//   'wSW'   = wfi 指令 + 真睡一次 + 醒 (fixture 01/05 模式)
//   'wSSW'+ = spurious wake / timeout 重 check predicate 多睡几次, 最终 true 退出
//   'wE'    = wfi 指令 + TW illegal 短路 (没进 wfi_wait, 没 W; 'E' 是 DEBUG_EXCEPTION
//             — 需 DEBUG_TRACE_ON 也开才看见)
//   'w' 后无 W 出现 = 真 bug (wfi_wait 永没返回)
//
// 实装: 跟主组完全同构 — append 到 trace_buf, hart 退出 flush。SMP per-thread
// 隔离免 race。跟 DEBUG_TRACE_ON 都开时, WFI 字符跟 fetch/check 字符夹在同一
// per-hart trace_buf 里, 按 in-thread 时序排列 (hart 视角内顺序正确)。
#ifdef DEBUG_TRACE_WFI_ON

#define DEBUG_WFI_ISSUE()  do { debug_buf_putc('w'); } while (0)
#define DEBUG_WFI_SLEEP()  do { debug_buf_putc('S'); } while (0)
#define DEBUG_WFI_WAKE()   do { debug_buf_putc('W'); } while (0)

#else  /* DEBUG_TRACE_WFI_ON 未定义 — WFI trace no-op */

#define DEBUG_WFI_ISSUE()  do { } while (0)
#define DEBUG_WFI_SLEEP()  do { } while (0)
#define DEBUG_WFI_WAKE()   do { } while (0)

#endif /* DEBUG_TRACE_WFI_ON */


#endif //DEBUG_H
