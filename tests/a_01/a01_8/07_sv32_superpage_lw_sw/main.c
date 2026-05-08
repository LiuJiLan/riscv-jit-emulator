// a_01_8 / 07_sv32_superpage_lw_sw — setup_pt()
//
// 跟 fixture 04/05 同模板, 区别: PT 仅一个 entry (root_pt[0x200] 4MB superpage leaf),
// 不设 leaf_pt — walker 走 level=1 leaf 路径直接到 PA, 不进 level=0。
//
// SV32 4MB superpage PTE 字段 (RV Privileged Spec Vol II §4.3.2):
//   bits[31:20] = PPN[1] (12 位, 4MB-aligned PPN; 这里 0x200 = PA 0x80000000 高段)
//   bits[19:10] = PPN[0] (10 位, superpage 必须 0)
//   bits[9:0]   = flags (D|A|G|U|X|W|R|V + 2 bit RSW)
//
// 起步 A=1 D=1 让 fast path 命中, 不验 hw-managed (那是 fixture 05 的事)。

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

    // root_pt[0x200] (vaddr 0x80000000 段 4MB) = leaf 4MB superpage, identity
    // PPN[1] = 0x200 (PA 0x80000000 >> 22); PPN[0] = 0 (superpage 必须)
    // PTE = (0x200 << 20) | 0 | (V|R|W|X|A|D) = 0x20000000 | 0xCF = 0x200000CF
    root_pt[0x200] = (0x200U << 20) | PTE_V | PTE_R | PTE_W | PTE_X | PTE_A | PTE_D;
}
