// a_01_8 / 05_sv32_hw_managed_ad — setup_pt()
//
// 跟 fixture 04 几乎一致, 唯一区别: leaf_pt[1] PTE 起步 A=0 D=0 (= 0x20080007),
// 验 walker hw-managed A/D 写回 PT 路径。
//
// 跑在 _start 调用栈内 (M-mode BARE), satp 还没设, walker 未激活, 写 PA 直接走
// host RAM identity。

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

    // root_pt[0x200] = pointer-to-next-level → leaf_pt
    root_pt[0x200] = ((LEAF_PT_PA >> 12) << 10) | PTE_V;          // 0x20040401

    // leaf_pt[0] (vaddr 0x80000000 4KB, 代码段) — A=1 让 fetch fast path 命中, 不
    // 验 fetch hw-managed (避免引入额外噪声; fetch 路径 hw-managed A 跟 store 路径
    // 同源, walker fixture 05 验 store 即可)
    leaf_pt[0] = (0x80000U << 10) | PTE_V | PTE_R | PTE_X | PTE_A;  // 0x2000004B

    // leaf_pt[1] (vaddr 0x80001000 4KB, 数据段) — A=0 D=0 起步, 强制 store_helper
    // fast path D 检查失败 → fall back walker → walker set A+D 写回 PT
    leaf_pt[1] = (0x80200U << 10) | PTE_V | PTE_R | PTE_W;          // 0x20080007
}
