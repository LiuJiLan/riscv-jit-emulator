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

// ----------------------------------------------------------------------------
// TLB / ASID 配置(a01_2 加入)
// ----------------------------------------------------------------------------
//
// TLB_ASID_BITS = 4 → ASIDLEN = 4。Sv32 规范: ASIDMAX ≤ 9, 我们选 4 < 9, 合法。
// guest 看到的 ASIDLEN 就是 4 位; csr.c 的 satp 写 helper 必须做 WARL 截断, 详见
// dummy.txt §3 (satp 合法性契约)。
#define TLB_ASID_BITS     4U
#define ASID_MAX          (1U << TLB_ASID_BITS)         /* = 16 */
#define ASID_MASK         (ASID_MAX - 1U)               /* = 0xF, fast path 用 */

// 单套叶 TLB 的 entry 数 (direct-mapped index)。
// 64 entry × 16 B/entry = 1 KB / 套, cache line 友好。
// set-associative 是改进项, 现阶段 direct-mapped 足够。
#define TLB_INDEX_BITS    6U
#define TLB_NUM_ENTRIES   (1U << TLB_INDEX_BITS)        /* = 64 */

// ----------------------------------------------------------------------------
// 后续可能加入(均不在 a01_2):
//   软边界 max block length     a01_3 / a01_5
// 加入时各自带 doc comment 说明取值理由。

#endif //CONFIG_H
