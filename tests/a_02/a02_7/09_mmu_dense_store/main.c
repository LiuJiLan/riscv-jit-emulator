// a02_7 / 09_mmu_dense_store — setup_pt()
//
// Sv32 两层页表: 代码页 + 单个数据页 0x80010000 (两页同在 vaddr 0x80000000 的 4MB
// 段内, 共用 leaf_pt_code):
//   root_pt[0x200]   → leaf_pt_code   (VPN[1]=0x200)
//   leaf_pt_code[0]  = 代码页 identity, V+R+X+A          (PA 0x80000000)
//   leaf_pt_code[16] = 数据页 identity, V+R+W+A+D        (PA 0x80010000)
// satp = 0x80080100 (Sv32 + PPN=0x80100 → root_pt @ 0x80100000)。
//
// "密集" = 循环 8 次 sw 同一数据页 → 叶 TLB 首轮 fill、之后恒命中。PTE.W/D 起步
// 置 1, store 走通且 walker 不卡 hw-managed D。

#include <stdint.h>

#define ROOT_PT_PA       0x80100000U
#define LEAF_PT_CODE_PA  0x80101000U

#define PTE_V  0x01U
#define PTE_R  0x02U
#define PTE_W  0x04U
#define PTE_X  0x08U
#define PTE_A  0x40U
#define PTE_D  0x80U

void setup_pt(void) {
    volatile uint32_t *root_pt   = (volatile uint32_t *)ROOT_PT_PA;
    volatile uint32_t *leaf_code = (volatile uint32_t *)LEAF_PT_CODE_PA;

    root_pt[0x200]   = ((LEAF_PT_CODE_PA >> 12) << 10) | PTE_V;
    leaf_code[0]     = (0x80000U << 10) | PTE_V | PTE_R | PTE_X | PTE_A;            // 代码页
    leaf_code[16]    = (0x80010U << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D;    // 数据页
}
