//
// Created by liujilan on 2026/4/29.
// a_01 全局编译期宏。
//
// 本文件只放编译期常量(#define)。
// 运行期变量(host_ram_base / host_ram_base_adjusted / 未来其他 ram 派生量)由 ram_init 初始化, 暴露在 ram.h —— 不放这里。
//
//

#ifndef CONFIG_H
#define CONFIG_H

// guest 物理地址空间内, RAM 区域的起点 GPA。
// 0x80000000 为 RISC-V常用的内存起点
#define GUEST_RAM_START   0x80000000UL

// guest RAM 大小(字节)。128MB。
// page on demand, 名义大不会占物理;
// 留足空间,后期不用回头改。
#define GUEST_RAM_SIZE    (128UL * 1024 * 1024)

// a01_1 范围内 config.h 只这两个宏。
// 后续可能加入(均不在 a01_1):
//   ASID_MAX / TLB_ASID_BITS    a01_4 (cpu + tlb)
//   TLB 大小 / 关联度等         a01_4
//   软边界 max block length     a01_3 / a01_5
// 加入时各自带 doc comment 说明取值理由。

#endif //CONFIG_H
