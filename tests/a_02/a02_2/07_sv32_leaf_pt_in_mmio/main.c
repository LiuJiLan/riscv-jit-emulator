// a_02_2 / 07_sv32_leaf_pt_in_mmio — setup_pt()
//
// 模仿 a_01_8/08 (leaf PT 落 RAM 外), 但 leaf PT 故意指 CLINT 区 (MMIO 区) 而不是
// 完全 RAM 外。验 mmu_walk level=0 PT 物理地址检查 (IS_GPA_RAM(pte0_pa) → false →
// access fault), 不分 "完全不存在" vs "落在 MMIO" — 两者都不在 RAM, walker 都拒。
//
// PT layout:
//   root_pt @ 0x80100000:
//     [0x200] = 4MB superpage identity 代码段 (跟 a_01_8/08 同, 让 fetch 顺利)
//             = (0x80000 << 10) | V|R|W|X|A|D
//             PPN 0x80000 << 10 = 0x20000000; | 0xCF = 0x200000CF
//     [0x240] = pointer-to-next-level, leaf_pt PA 故意指 CLINT 0x02000000
//             = (0x02000 << 10) | V = 0x00800001
//   leaf_pt 不分配 (PA 0x02000000 是 CLINT 不是 RAM, walker 检查 fail 不会真去读)
//
// 触发路径 (跟 a_01_8/08 同, 区别 PT 落点 MMIO 不是 0xC0000000):
//   walker walk vaddr 0x90000000:
//     vpn1=0x240, vpn0=0x000
//     pte1_pa = 0x80100000 + 0x240*4 = 0x80100900 (RAM 内, 读 OK)
//     pte1 = 0x00800001 (V=1, R=W=X=0 → pointer-to-next-level)
//     pte1_full_pa = 0x02000 << 12 = 0x02000000 (CLINT, !IS_GPA_RAM → fail)
//     pte0_pa = 0x02000000
//     fault_cause = af_cause_for(R) = CAUSE_LOAD_ACCESS_FAULT = 5

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

    // root_pt[0x200] (vaddr 0x80000000 段) = 4MB superpage identity 代码段
    // 让 fetch 顺利 (用 superpage 简化 — leaf_pt_code 不需要单独分配)
    root_pt[0x200] = (0x80000U << 10) | PTE_V | PTE_R | PTE_W | PTE_X | PTE_A | PTE_D;  // 0x200000CF

    // root_pt[0x240] (vaddr 0x90000000 段) = pointer-to-next-level, leaf_pt 故意指
    // CLINT 区 (0x02000000); walker 读 pte0 时 IS_GPA_RAM(0x02000000) false → access fault
    root_pt[0x240] = (0x02000U << 10) | PTE_V;  // 0x00800001
}
