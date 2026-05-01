//
// Created by liujilan on 2026/4/29.
// a_01 ram 模块对外接口。
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

extern void *host_ram_base;
extern uint8_t *gpa_to_hva_offset;

// 成功返回 0,失败返回 -1
//      内部通过 fprintf 具体原因。
// 失败模式:
//   1. mmap 失败
//   2. madvise(MADV_NOHUGEPAGE) 失败(此时已 munmap 清理)
int ram_init(void);

#endif //PLATFORM_RAM_H