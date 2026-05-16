// a_02_2 / 06_sv32_clint_reject — setup_pt()
//
// 跟 04 同模板, leaf_pt_mmio 多一个 entry [12] 覆盖 vaddr 0x0200C000 段 (clint final
// else 触发 case 用)。
//
// PT 三层:
//   root_pt[0x200] → leaf_pt_code   (代码段)
//   root_pt[0x008] → leaf_pt_mmio   (CLINT 段)
//   leaf_pt_mmio entries:
//     [0]  msip page     (PA 0x02000000)
//     [4]  mtimecmp page (PA 0x02004000)
//     [12] final-else page (PA 0x0200C000, CLINT 内部, off >= 0x200BFF8+4)
//
// 5 个 case 都映射 PT 然后让 clint_read/clint_write 内部触发不同 branch:
//   case 0: sw 0x02000004 (msip idx 越界, MAX_HARTS=1) → cause 7
//   case 1: sw 0x02004008 (mtimecmp idx 越界)            → cause 7
//   case 2: sw 0x0200C000 (final else off)              → cause 7
//   case 3: sb 0x02000000 (size != 4)                   → cause 7
//   case 4: lw 0x02000004 (msip idx 越界 read)          → cause 5

#include <stdint.h>

#define ROOT_PT_PA         0x80100000U
#define LEAF_PT_CODE_PA    0x80101000U
#define LEAF_PT_MMIO_PA    0x80102000U

#define PTE_V  0x01U
#define PTE_R  0x02U
#define PTE_W  0x04U
#define PTE_X  0x08U
#define PTE_A  0x40U
#define PTE_D  0x80U

void setup_pt(void) {
    volatile uint32_t *root_pt   = (volatile uint32_t *)ROOT_PT_PA;
    volatile uint32_t *leaf_code = (volatile uint32_t *)LEAF_PT_CODE_PA;
    volatile uint32_t *leaf_mmio = (volatile uint32_t *)LEAF_PT_MMIO_PA;

    root_pt[0x200] = ((LEAF_PT_CODE_PA >> 12) << 10) | PTE_V;
    root_pt[0x008] = ((LEAF_PT_MMIO_PA >> 12) << 10) | PTE_V;

    leaf_code[0] = (0x80000U << 10) | PTE_V | PTE_R | PTE_X | PTE_A;

    leaf_mmio[0]  = (0x02000U << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D;
    leaf_mmio[4]  = (0x02004U << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D;
    leaf_mmio[12] = (0x0200CU << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D;
}
