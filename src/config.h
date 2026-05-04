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
// IALIGN(指令地址对齐, a01_4 加入)
// ----------------------------------------------------------------------------
//
// RV spec: IALIGN 是 hart 级常量(单位:位), 由是否实现 C / Zca 扩展决定:
//   - 实现 C/Zca → IALIGN = 16(所有 PC 必须 2 字节对齐)
//   - 不实现 C   → IALIGN = 32(所有 PC 必须 4 字节对齐)
// misa.C 决定 IALIGN; 多数实现把 misa.C 写死, IALIGN 是编译期常量。
// 本项目: misa.C 强制为 1(decode 已含 RVC 路径), IALIGN 固定 16。
// 动态切 misa 是远期议题, 真要做时再回来动这里 + 加运行时切换路径。
//
// 实际效果: jal / branch (imm[0]=0 编码强制) + jalr (& ~1u 强制 mask LSB) 路径下,
// (target & IALIGN_MASK) 永远 = 0, instruction-address-misaligned 异常 (cause 0)
// 在本项目中是 dead code; 但代码里仍写检查 + 调 trap_raise_exception 占位, 保结构
// 完整, 与 spec 语义对齐, 也方便日后切 IALIGN=32 时只改宏不改逻辑。
#define IALIGN            16U
#define IALIGN_BYTES      (IALIGN / 8U)        /* = 2 */
#define IALIGN_MASK       (IALIGN_BYTES - 1U)  /* = 1; (target & IALIGN_MASK) != 0 → 不对齐 */

// ----------------------------------------------------------------------------
// 软边界:单 block 最大指令数(a01_4 加入,interpreter / 未来 translator 共用)
// ----------------------------------------------------------------------------
//
// plan §1.23.2 软边界初版默认 64;[32, 128] 区间都合理, 性能数据出来后再调。
// interpreter / translator 各自循环维护自己的计数器, 共享同一个上限常量。
// 当前用途:fixture 写错时 (例如忘 ecall 收尾, 或 branch 死循环) 失控保护;
// a_05+ OS 场景下还要叠加跨 4K page 检查 + 真边界 op (硬边界由
// is_block_boundary_inst 处理), 这两条是另两个独立的 block 截断条件。
#define BLOCK_INST_LIMIT  64U

#endif //CONFIG_H
