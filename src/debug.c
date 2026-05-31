//
// Created by liujilan on 2026/5/12.
// debug 模块实现: per-hart trace buffer 定义 (__thread storage) + flush 函数。
// 宏在 debug.h (DEBUG_XXX 字符宏只 append 到 trace_buf, 不直接 fputc; flush
// 时一把 fwrite 到 stderr; SMP 多 hart 自动 per-thread 隔离, 无 race)。
//

#include "debug.h"

__thread char trace_buf[DEBUG_BUF_SIZE];
__thread size_t trace_idx;
__thread uint32_t hartid_self;

// debug_flush_local_trace — 把当前线程累积的 trace 字符前缀上 "[hart%u trace] "
// 一把 fwrite 到 stderr + EOL 行尾。trace 是 hart join 阶段 (trace + perf +
// halted) 的子项, 不自己收尾空行, 由 dispatcher 退出尾的 halted 那行后加 EOL
// 收尾整 hart join 块。空 buffer 时 no-op。
// 调用点: dispatcher 退出前 (DEBUG_NEWLINE 触发) + hart_exec_run 末尾兜底。
void debug_flush_local_trace(void) {
    if (trace_idx == 0) return;
    fprintf(stderr, "[hart%u trace] ", hartid_self);
    fwrite(trace_buf, 1, trace_idx, stderr);
    fputs(EOL, stderr);
    fflush(stderr);
    trace_idx = 0;
}
