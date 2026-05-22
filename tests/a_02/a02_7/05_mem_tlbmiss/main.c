// a02_7 / 05_mem_tlbmiss — setup_pt()
//
// Sv32 两层页表, 代码段 1 页 + 数据段 256 个 4KB 页:
//   root_pt[0x200]  → leaf_pt_code   (vaddr 0x80000000 段, VPN[1]=0x200)
//   leaf_pt_code[0] = 代码页 identity, V+R+X+A
//   root_pt[0x201]  → leaf_pt_data   (vaddr 0x80400000 段, VPN[1]=0x201)
//   leaf_pt_data[0..255] = 数据页 identity, V+R+W+A+D (1MB 区, 0x80400000 起)
// satp = 0x80080100 (Sv32 + PPN=0x80100 → root_pt @ 0x80100000)。
//
// 数据段刻意铺 256 个 4KB 页 (> 64-entry 叶 TLB): stub.S 步进跨页扫这 256 页,
// 稳态下每次 load 都撞 TLB miss → mmu_walk 重走 (这就是本 fixture 要测的开销)。
// PTE.A / PTE.D 起步置 1, walker 不卡 hw-managed (那是 a02_2/09 的事)。

#include <stdint.h>

#define ROOT_PT_PA       0x80100000U
#define LEAF_PT_CODE_PA  0x80101000U
#define LEAF_PT_DATA_PA  0x80102000U
#define DATA_VBASE       0x80400000U   /* 数据区虚拟基址 (VPN[1]=0x201) */
#define DATA_PAGES       256U          /* 256 × 4KB = 1MB 数据区 */

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

    // 数据段 (vaddr 0x80400000 起 256 个 4KB 页) — V+R+W+A+D, identity
    root_pt[0x201] = ((LEAF_PT_DATA_PA >> 12) << 10) | PTE_V;
    for (uint32_t i = 0; i < DATA_PAGES; i++) {
        uint32_t ppn = (DATA_VBASE >> 12) + i;       // identity 映射
        leaf_data[i] = (ppn << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D;
    }
}
