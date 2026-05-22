// a02_7 / 10_mmu_sparse_load — setup_pt()
//
// Sv32 两层页表: 代码页 + 8 个数据页 (故意撞同一叶 TLB index → 条条 miss)。
//   root_pt[0x200] → leaf_pt_code (代码段); root_pt[0x201] → leaf_pt_data (数据段)
//   leaf_code[0]   = 代码页 identity, V+R+X+A
//   8 个数据页: vaddr = 0x80401000 + k*0x40000 (k=0..7), 每页相距 256KB。
//
// TLB index = (vaddr>>12) & 0x3F (TLB_NUM_ENTRIES=64, direct-mapped; tlb.h L50):
//   8 个数据页 VPN 低 6 位都 = 1 → 全撞 TLB index 1;
//   代码页 0x80000000 VPN 低 6 位 = 0 → index 0, 取指 TLB 不被数据访问挤掉。
// → 循环里 8 条 lw 顺序访这 8 页, direct-mapped 下每条都挤掉上一条 → 条条 TLB miss
//   → 每条都走 mmu_walker_helper_load → mmu_walk 两级重走。
// PTE.A/D 起步置 1, walker 不卡 hw-managed。

#include <stdint.h>

#define ROOT_PT_PA       0x80100000U
#define LEAF_PT_CODE_PA  0x80101000U
#define LEAF_PT_DATA_PA  0x80102000U

#define PTE_V  0x01U
#define PTE_R  0x02U
#define PTE_W  0x04U
#define PTE_X  0x08U
#define PTE_A  0x40U
#define PTE_D  0x80U

void setup_pt(void) {
    volatile uint32_t *root_pt   = (volatile uint32_t *)ROOT_PT_PA;
    volatile uint32_t *leaf_code = (volatile uint32_t *)LEAF_PT_CODE_PA;
    volatile uint32_t *leaf_data = (volatile uint32_t *)LEAF_PT_DATA_PA;

    // 代码段 (vaddr 0x80000000) — V+R+X+A
    root_pt[0x200] = ((LEAF_PT_CODE_PA >> 12) << 10) | PTE_V;
    leaf_code[0]   = (0x80000U << 10) | PTE_V | PTE_R | PTE_X | PTE_A;

    // 数据段 (vaddr 0x80400000 段) — 8 个 4KB 页, identity, V+R+W+A+D
    root_pt[0x201] = ((LEAF_PT_DATA_PA >> 12) << 10) | PTE_V;
    for (uint32_t k = 0; k < 8; k++) {
        uint32_t va  = 0x80401000U + k * 0x40000U;     // 每页相距 256KB
        uint32_t ppn = va >> 12;                       // identity 映射
        leaf_data[(va >> 12) & 0x3FFU] =
            (ppn << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D;
    }
}
