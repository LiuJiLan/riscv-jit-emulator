// a_02_2 / 05_sv32_bus_unmapped_reject — setup_pt()
//
// 跟 04 同模板 + 多一段 leaf_pt_bus 把 vaddr 0x03000000 段映射到 PA 0x03000000
// (bus 未注册区, identity)。
//
// PT 三层:
//   root_pt[0x200] → leaf_pt_code   (vaddr 0x80000000 段, 代码段)
//   root_pt[0x00C] → leaf_pt_bus    (vaddr 0x03000000 段, bus 未注册)
//   leaf_pt_code[0] = 代码段 identity, R+X+A
//   leaf_pt_bus[0]  = 未注册 PA identity, R+W+A+D (起步 A+D=1 跟 04 同)
//
// vpn1 解码: 0x03000000 >> 22 = 0x00C; vpn0 = 0 → leaf_pt_bus[0] 覆盖 vaddr
//            0x03000000-0x03000FFF 段, identity 到 PA 0x03000000-0x03000FFF。
//
// 触发路径 (跟 a_02_2/02 BARE 版同语义, 多走一遍 walker):
//   sw → 0x03000000 → lsu_store_helper SV32 miss → mmu_walker_helper_store →
//     mmu_walk OK (pa = 0x03000000) → IS_GPA_RAM(0x03000000) false →
//     mmio_write_helper → 扫表无命中 → trap_raise(cause 7 STORE_ACCESS_FAULT, gva)
//   lw → 0x03000000 → 同形态 → mmio_read_helper → cause 5

#include <stdint.h>

#define ROOT_PT_PA         0x80100000U
#define LEAF_PT_CODE_PA    0x80101000U
#define LEAF_PT_BUS_PA     0x80102000U

#define PTE_V  0x01U
#define PTE_R  0x02U
#define PTE_W  0x04U
#define PTE_X  0x08U
#define PTE_A  0x40U
#define PTE_D  0x80U

void setup_pt(void) {
    volatile uint32_t *root_pt   = (volatile uint32_t *)ROOT_PT_PA;
    volatile uint32_t *leaf_code = (volatile uint32_t *)LEAF_PT_CODE_PA;
    volatile uint32_t *leaf_bus  = (volatile uint32_t *)LEAF_PT_BUS_PA;

    root_pt[0x200] = ((LEAF_PT_CODE_PA >> 12) << 10) | PTE_V;
    root_pt[0x00C] = ((LEAF_PT_BUS_PA  >> 12) << 10) | PTE_V;

    leaf_code[0] = (0x80000U << 10) | PTE_V | PTE_R | PTE_X | PTE_A;

    // identity 到 0x03000000 (bus 未注册), R+W+A+D
    leaf_bus[0]  = (0x03000U << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D;
}
