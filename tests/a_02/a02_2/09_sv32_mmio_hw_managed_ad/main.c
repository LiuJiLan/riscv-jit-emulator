// a_02_2 / 09_sv32_mmio_hw_managed_ad — setup_pt()
//
// 模仿 a_01_8/05 (hw-managed A/D 单测) 的 MMIO 版本: leaf_pt_mmio[4] (mtimecmp page)
// 起步 A=0 D=0, 强制 walker fall back set A+D 写回 PT。
//
// 关键观察 (P3 后 + session_004 D2 finding 仍然适用):
//   - A/D 是 PTE 描述的"page 被 access/dirty", 跟 page 内容 (RAM/MMIO) 无关
//   - PT 本身永远在 RAM (leaf_pt_mmio @ 0x80102000 是 RAM); walker set A+D 写回 PT
//     是写 RAM, 不是写 MMIO
//   - 即使 leaf PA 是 MMIO (0x02004000 → CLINT mtimecmp), walker 也 set A+D 写回
//     PT, 然后调 mmio_write_helper 完成 MMIO 写
//
// P2 副作用 (本 fixture 不直接验, 留 P2 修时另起 fixture):
//   walker set A+D 写回 PT **先于** walker_helper 判 MMIO; 即使 mmio_write_helper 失败
//   trap_raise, PTE.A/D 已 set。本 fixture mmio_write_helper 成功不会触发该 corner case。
//
// PT layout (跟 04 同, 但 leaf_pt_mmio[4] 起步 A=0 D=0):
//   root_pt[0x200] → leaf_pt_code  (代码段 superpage 也行, 但维持模板用 leaf)
//   root_pt[0x008] → leaf_pt_mmio  (CLINT 段)
//   leaf_pt_mmio[4] = 0x00801007    (V|R|W, A=0 D=0 起步; 验 walker 真 set 写回)

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

    // leaf_pt_mmio[4]: mtimecmp page; **起步 A=0 D=0**, 验 walker set A+D 写回 PT
    leaf_mmio[4] = (0x02004U << 10) | PTE_V | PTE_R | PTE_W;  // 0x00801007 (无 A 无 D)
}
