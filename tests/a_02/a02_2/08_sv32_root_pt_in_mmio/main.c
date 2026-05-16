// a_02_2 / 08_sv32_root_pt_in_mmio — setup_pt() (空)
//
// 模仿 a_01_8/09 (root PT 落 RAM 外), 但 satp.PPN 故意指 CLINT 区 (MMIO 区) 而不是
// 完全 RAM 外。验 mmu_walk level=1 PT 物理地址检查 (IS_GPA_RAM(pte1_pa) → false)
// 对 root PT 落 MMIO 行为正确 (cause 1 INST_ACCESS_FAULT, 走 mmu_translate_pc 路径
// 2b 不长跳)。
//
// setup_pt() 空 — 真 root_pt 不分配, satp.PPN=0x02000 (CLINT) 故意指 MMIO。

#include <stdint.h>

void setup_pt(void) {
    // root_pt 故意不分配, satp 直接指 CLINT (0x02000000), walker 读 pte1 时
    // IS_GPA_RAM 检查失败 → access fault (cause 1, fetch 路径)
}
