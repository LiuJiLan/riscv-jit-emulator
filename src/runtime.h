//
// Created by liujilan on 2026/5/19.
// runtime 模块 — host-side 生命周期信号 (degenerate monitor; src/dummy.txt §7)。
//
// 两个 atomic flag, 都默认 1 = "继续运行", 0 = "触发对应停机路径":
//
//   system_reset_signal  (SRS)
//     极性: 1=继续, 0=触发 system reset (T5 简化下 = 退出 main while)
//     触发点: dispatcher tri-fault (in_trap >= 3) 后 set 0; 未来 user 端
//             reset 按钮 / 严重 hart 错也 set 0
//     check 点: dispatcher 主循环 (while (in_trap < 3 && atomic_load(SRS)))
//               + main while (while (atomic_load(SRS)))
//     T5 简化语义: dispatcher set SRS=0 → main while 失败 → 走 cleanup 退出
//                   (没真"reset 重 iter"路径; 那是未来事)
//
//   shutdown_signal  (SDS)
//     极性: 1=继续, 0=触发 shutdown (POR 退出; 通知所有辅助线程退)
//     触发点: main while 退出后 main 自己 set 0; 未来严重错路径同时 set 0
//     check 点: timer 辅助线程主循环 (clint.c timer_run); 未来 monitor 等
//               所有不受 SRS 控制的辅助线程都 check 它
//
// 触发关系契约 (协议规则):
//   set SDS=0 之前 / 同时, 必须先 set SRS=0; 反之不要求 (SR 不必触发 shutdown)
//   即 "SDS 蕴含 SRS" — 如果通知所有辅助线程退, 那 system 自己也得退
//
// degenerate monitor — 单字段无跨字段一致性问题, 不强制接口函数, caller 直接
// atomic_load_explicit / atomic_store_explicit 操作 (跟 host_ram_base extern
// 全局先例对齐, 接口面最小, gdb 可观察)。
//
// memory_order: relaxed 起步 (dispatcher 内层 check), release / acquire 在
// "producer 通知 consumer 退" 的语义边界用 (set 0 release / load acquire);
// 详 dummy.txt §7 monitor 模型 + §12 线程 lifecycle 协议。
//
// 模块 scope: 仅这两 flag。不预定其他函数接口 (init / destroy / signal_caught
// 等"未来扩展" 一律不写; 真用到再加)。
//

#ifndef RUNTIME_H
#define RUNTIME_H

#include <stdatomic.h>

extern _Atomic int system_reset_signal;
extern _Atomic int shutdown_signal;

#endif //RUNTIME_H
