//
// ram 模块对外接口。
//
// 由 ram_init 初始化的全局清单(声明在本头,定义在 ram.c):
//   host_ram_base
//     mmap 返回的 host 起点指针。对应的 guest 物理地址 = GUEST_RAM_START。
//   gpa_to_hva_offset
//     = (uint8_t *)host_ram_base - GUEST_RAM_START。
//     省去每次访问的减法,使得 host_addr = gpa_to_hva_offset + gpa。
//     类型是 uint8_t * 而非整型, 调用方直接做指针算术不用 cast。
//
// 报错风格见 src/dummy.txt §5。
//

#ifndef PLATFORM_RAM_H
#define PLATFORM_RAM_H

#include <stdint.h>

#include "config.h"   // GUEST_RAM_START / GUEST_RAM_SIZE (IS_GPA_RAM 宏用)

extern void *host_ram_base;
extern uint8_t *gpa_to_hva_offset;

// ----------------------------------------------------------------------------
// IS_GPA_RAM(pa) — guest 物理地址是否落在 RAM 区
// ----------------------------------------------------------------------------
//
// 实现: 无符号下溢比较 (一次 sub + 一次 cmp, 不需要两个比较)。
//   pa < GUEST_RAM_START 时 (pa - GUEST_RAM_START) 在 uint32_t 下下溢成很大的值,
//   自然 >= GUEST_RAM_SIZE, 比较失败。
//   pa >= GUEST_RAM_START 时 (pa - GUEST_RAM_START) 是正常 offset, 跟 SIZE 比较。
//
// 调用方 (lsu / mmu walker / mmu_translate_pc): 决定 RAM 直通 (gpa_to_hva_offset
//   + pa → host_ptr memcpy) 还是 bus_dispatch (MMIO 派发); 跟 dummy.txt §8 三层
//   模型层 (3) 内存访问层的 "RAM-like 直通 vs 纯 MMIO 派发" 对齐。
//
// 未来 ROM 接入: 加一个 IS_GPA_ROM 平行宏, callsite 自己决定查哪个; 不要把 ROM
// 塞进 IS_GPA_RAM (RAM/ROM 写语义不同 — 写 RAM OK / 写 ROM = access fault)。
#define IS_GPA_RAM(pa)  ((uint32_t)((pa) - GUEST_RAM_START) < GUEST_RAM_SIZE)

// 成功返回 0,失败返回 -1
//      内部通过 fprintf 具体原因。
// 失败模式:
//   1. mmap 失败
//   2. madvise(MADV_NOHUGEPAGE) 失败(此时已 munmap 清理)
int ram_init(void);

// 释放 ram_init 分配的 host mmap; host_ram_base / gpa_to_hva_offset 重置为
// NULL。POR 收尾 (main 末段) 调一次, 跟 cpu_destroy / clint_destroy 三 destroy
// 对称。
//
// 进程退出时 OS 会自动 munmap, 函数留作 lifecycle 对称 + valgrind clean 输出
// (POR 退出前已主动还内存)。重复调或 ram_init 未成功调时 do nothing (host_ram_
// base 已是 NULL)。
void ram_destroy(void);

#endif //PLATFORM_RAM_H