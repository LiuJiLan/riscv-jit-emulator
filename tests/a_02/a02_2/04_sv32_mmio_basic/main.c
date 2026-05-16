// a_02_2 / 04_sv32_mmio_basic — setup_pt()
//
// 模仿 a_01_8/04 的 .S+.c 模板。PT 多一段 leaf_pt_mmio 覆盖 CLINT 区。
//
// PT 三层:
//   root_pt[0x200] → leaf_pt_code   (vaddr 0x80000000 段, 代码段)
//   root_pt[0x008] → leaf_pt_mmio   (vaddr 0x02000000 段, CLINT MMIO 段)
//   leaf_pt_code[0]  = 代码段 identity, R+X+A (PA 0x80000000)
//   leaf_pt_mmio[0]  = msip page identity   (PA 0x02000000), R+W+A+D
//   leaf_pt_mmio[4]  = mtimecmp page identity (PA 0x02004000), R+W+A+D
//   leaf_pt_mmio[11] = mtime page identity    (PA 0x0200B000), R+W+A+D
//
// PTE.A / PTE.D 起步都 1, 让 fast path / walker 直接走通, 不验 hw-managed
// (那是 fixture 09 的事)。

#include <stdint.h>

#define ROOT_PT_PA          0x80100000U
#define LEAF_PT_CODE_PA     0x80101000U
#define LEAF_PT_MMIO_PA     0x80102000U

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

    // root_pt entries (pointer-to-next-level, R=W=X=0)
    root_pt[0x200] = ((LEAF_PT_CODE_PA >> 12) << 10) | PTE_V;
    root_pt[0x008] = ((LEAF_PT_MMIO_PA >> 12) << 10) | PTE_V;

    // leaf_pt_code: 代码段 (PA 0x80000000) — R+X+A; U=0 给 S
    leaf_code[0] = (0x80000U << 10) | PTE_V | PTE_R | PTE_X | PTE_A;

    // leaf_pt_mmio: CLINT 三 page (msip / mtimecmp / mtime) identity 映射, R+W+A+D
    // (起步 A+D=1 让 fast path / walker 顺利, 不卡 hw-managed; A/D 测试留 fixture 09)
    leaf_mmio[0]  = (0x02000U << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D;  // msip
    leaf_mmio[4]  = (0x02004U << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D;  // mtimecmp
    leaf_mmio[11] = (0x0200BU << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D;  // mtime
}
