// a02_7 / 02_mmu_sv32 — setup_pt()
//
// Sv32 两层页表, 只映射代码页 (S-mode 纯计算循环不碰数据 / MMIO):
//   root_pt[0x200]  → leaf_pt_code   (vaddr 0x80000000 段, VPN[1]=0x200)
//   leaf_pt_code[0] = 代码页 identity, V+R+X+A (PA 0x80000000)
// satp = 0x80080100 (Sv32 + PPN=0x80100 → root_pt @ 0x80100000)。
// PTE.A 起步置 1, walker 不卡 hw-managed A/D (那是 a02_2/09 的事)。

#include <stdint.h>

#define ROOT_PT_PA       0x80100000U
#define LEAF_PT_CODE_PA  0x80101000U

#define PTE_V  0x01U
#define PTE_R  0x02U
#define PTE_X  0x08U
#define PTE_A  0x40U

void setup_pt(void) {
    volatile uint32_t *root_pt   = (volatile uint32_t *)ROOT_PT_PA;
    volatile uint32_t *leaf_code = (volatile uint32_t *)LEAF_PT_CODE_PA;

    // root_pt entry (pointer-to-next-level, R=W=X=0)
    root_pt[0x200] = ((LEAF_PT_CODE_PA >> 12) << 10) | PTE_V;

    // leaf_pt_code: 代码页 (PA 0x80000000) — V+R+X+A; U=0 给 S
    leaf_code[0] = (0x80000U << 10) | PTE_V | PTE_R | PTE_X | PTE_A;
}
