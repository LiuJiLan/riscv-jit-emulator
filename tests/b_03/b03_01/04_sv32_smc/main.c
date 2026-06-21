// b_03_1 / 04_sv32_smc — setup_pt()
//
// PT setup 跟 b02_06/01 同体例 (root_pt + leaf_pt; leaf[0]=code / leaf[1]=data),
// 区别: leaf[0] 加 PTE_W + PTE_D — 让 S-mode code page 可 sw (SMC trigger 需要
// 客户机 SV32 sw 改 inst). 加 W 不影响 JIT cache key (key = (PA, regime), regime
// baked priv 视角, 不 bake PTE 字段值; perm 是 TLB entry runtime 读).

#include <stdint.h>

#define ROOT_PT_PA   0x80100000U
#define LEAF_PT_PA   0x80101000U

#define PTE_V  0x01U
#define PTE_R  0x02U
#define PTE_W  0x04U
#define PTE_X  0x08U
#define PTE_A  0x40U
#define PTE_D  0x80U

void setup_pt(void) {
    volatile uint32_t *root_pt = (volatile uint32_t *)ROOT_PT_PA;
    volatile uint32_t *leaf_pt = (volatile uint32_t *)LEAF_PT_PA;

    // root_pt[0x200] (vaddr 0x80000000 段 4MB) → leaf_pt
    root_pt[0x200] = ((LEAF_PT_PA >> 12) << 10) | PTE_V;                              // 0x20040401

    // leaf_pt[0] (vaddr 0x80000000 4KB) → PA 0x80000000 (代码段; 加 W + D 让 SMC sw 可写)
    leaf_pt[0] = (0x80000U << 10) | PTE_V | PTE_R | PTE_W | PTE_X | PTE_A | PTE_D;    // 0x200000CF

    // leaf_pt[1] (vaddr 0x80001000 4KB) → PA 0x80200000 (数据段; R+W; 跟 b02_06 同)
    leaf_pt[1] = (0x80200U << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D;            // 0x200800C7
}
