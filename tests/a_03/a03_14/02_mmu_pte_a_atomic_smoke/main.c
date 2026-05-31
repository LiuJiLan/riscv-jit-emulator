// a03_14 / 02_mmu_pte_a_atomic_smoke — setup_pt
//
// PT 起步: code 页 V+R+X+A (A=1 防 fetch walker 反复 set, 不卡循环); data 页
// V+R+W (**A=0 故意不 set**, 让 hart 0 S-mode lw 时 walker 真触发 atomic OR set A)。
//
// satp = 0x80080100 (Sv32 + PPN=0x80100 → root_pt @ 0x80100000), 跟 a02_7/08 同。
//
// 验证机制:
//   - hart 0 lw 0x80010000 → walker_helper_load 走 atomic_fetch_or set PTE.A
//   - IPI 后 hart 1 读 leaf_code[16] (PA=0x80101040) → 验 (PTE & PTE_A) != 0

#include <stdint.h>

#define ROOT_PT_PA       0x80100000U
#define LEAF_PT_CODE_PA  0x80101000U

#define PTE_V  0x01U
#define PTE_R  0x02U
#define PTE_W  0x04U
#define PTE_X  0x08U
#define PTE_A  0x40U

void setup_pt(void) {
    volatile uint32_t *root_pt   = (volatile uint32_t *)ROOT_PT_PA;
    volatile uint32_t *leaf_code = (volatile uint32_t *)LEAF_PT_CODE_PA;

    root_pt[0x200]   = ((LEAF_PT_CODE_PA >> 12) << 10) | PTE_V;
    // code 页: A=1 起步 (fetch walker 不反复 set A)
    leaf_code[0]     = (0x80000U << 10) | PTE_V | PTE_R | PTE_X | PTE_A;
    // data 页: **A=0** 起步, 让 hart 0 S-mode lw 时 walker 真 atomic OR set A
    leaf_code[16]    = (0x80010U << 10) | PTE_V | PTE_R | PTE_W;
}
