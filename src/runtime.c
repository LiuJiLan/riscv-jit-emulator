//
// Created by liujilan on 2026/5/19.
// runtime 模块实现 — 仅两个 atomic flag 定义 (协议见 runtime.h)。
//
// 初值 1 兜底 — 即使 main 进 while 前忘记显式 set 1, 程序也能跑起来; main
// 显式重新 set 1 是 lifecycle 可读性, 不是必要。
//

#include "runtime.h"

_Atomic int system_reset_signal = 1;
_Atomic int shutdown_signal     = 1;
