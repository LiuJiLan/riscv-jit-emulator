// a_01_8 / 08_sv32_pt_not_in_ram — setup_pt()
//
// PT 仅写两个 root_pt entry:
//   [0x200] (vaddr 0x80000000 段) = 4MB superpage identity (代码段 fetch 走通)
//   [0x240] (vaddr 0x90000000 段) = pointer-to-next-level, leaf_pt PA 0xC0000000
//                                    故意不在 RAM, 让 walker level=0 PT 读 RAM 检查失败
// 不分配 leaf_pt (PA 0xC0000000 走不到; walker level=0 检查 fail 提前 return)。

#include <stdint.h>

#define ROOT_PT_PA   0x80100000U
#define BAD_LEAF_PA  0xC0000000U     // 故意不在 RAM (RAM 0x80000000~0x88000000)

#define PTE_V  0x01U
#define PTE_R  0x02U
#define PTE_W  0x04U
#define PTE_X  0x08U
#define PTE_A  0x40U
#define PTE_D  0x80U

void setup_pt(void) {
    volatile uint32_t *root_pt = (volatile uint32_t *)ROOT_PT_PA;

    // root_pt[0x200] (vaddr 0x80000000 段 4MB) = leaf 4MB superpage identity (代码段)
    // PTE = (PPN[1]=0x200 << 20) | (PPN[0]=0 << 10) | (V|R|W|X|A|D) = 0x200000CF
    root_pt[0x200] = (0x200U << 20) | PTE_V | PTE_R | PTE_W | PTE_X | PTE_A | PTE_D;

    // root_pt[0x240] (vaddr 0x90000000 段) = pointer-to-next-level → BAD_LEAF_PA
    // PTE = (PPN(BAD_LEAF_PA) << 10) | V; PPN = 0xC0000000 >> 12 = 0xC0000
    // PTE = (0xC0000 << 10) | 0x01 = 0x30000000 | 0x01 = 0x30000001
    root_pt[0x240] = ((BAD_LEAF_PA >> 12) << 10) | PTE_V;          // 0x30000001
}
