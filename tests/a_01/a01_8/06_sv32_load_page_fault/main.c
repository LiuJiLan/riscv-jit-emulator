// a_01_8 / 06_sv32_load_page_fault — setup_pt()
//
// 跟 fixture 04/05 同模板, 区别: leaf_pt[1] = 0 (PTE.V=0), 让 lw vaddr 0x80001000
// 触发 walker fail → load page fault (cause 13)。
//
// 代码段 leaf_pt[0] 仍正常 (S 跑 fetch 用); 数据段 leaf_pt[1] V=0 故意失败。

#include <stdint.h>

#define ROOT_PT_PA   0x80100000U
#define LEAF_PT_PA   0x80101000U

#define PTE_V  0x01U
#define PTE_R  0x02U
#define PTE_X  0x08U
#define PTE_A  0x40U

void setup_pt(void) {
    volatile uint32_t *root_pt = (volatile uint32_t *)ROOT_PT_PA;
    volatile uint32_t *leaf_pt = (volatile uint32_t *)LEAF_PT_PA;

    root_pt[0x200] = ((LEAF_PT_PA >> 12) << 10) | PTE_V;          // 0x20040401

    // 代码段 leaf_pt[0] (vaddr 0x80000000 4KB) — 正常, S 跑 fetch
    leaf_pt[0] = (0x80000U << 10) | PTE_V | PTE_R | PTE_X | PTE_A; // 0x2000004B

    // 数据段 leaf_pt[1] (vaddr 0x80001000 4KB) — V=0, 故意 walker fail
    leaf_pt[1] = 0U;
}
