// a_01_9 / 03_medeleg_deliver_s — setup_pt()
//
// 跟 a_01_8 fixture 02 / 07 同模板 — root_pt[0x200] 4MB superpage leaf identity (PA 0x80000000
// 起 4MB 范围 identity 映射)。S phase 跑代码段经 superpage walker fetch 到 PA。

#include <stdint.h>

#define ROOT_PT_PA   0x80100000U

#define PTE_V  0x01U
#define PTE_R  0x02U
#define PTE_W  0x04U
#define PTE_X  0x08U
#define PTE_A  0x40U
#define PTE_D  0x80U

void setup_pt(void) {
    volatile uint32_t *root_pt = (volatile uint32_t *)ROOT_PT_PA;

    // root_pt[0x200] (vaddr 0x80000000 段 4MB) = leaf 4MB superpage identity
    // PTE = (PPN[1]=0x200 << 20) | (PPN[0]=0 << 10) | (V|R|W|X|A|D) = 0x200000CF
    root_pt[0x200] = (0x200U << 20) | PTE_V | PTE_R | PTE_W | PTE_X | PTE_A | PTE_D;
}
