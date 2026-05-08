// a_01_8 / 04_sv32_lw_sw_basic — setup_pt()
//
// xv6 main.c 风格的 C 函数容器 (a_01_8 fixture 验证 task 首次启用 .S+.c 模板)。
// 不调 libc, 用 volatile uint32_t* 直接写 PA — 跑在 _start 调用栈内 (M-mode BARE),
// satp 还没设, walker 未激活, 写 PA 直接走 host RAM identity。
//
// PT 三个 entry:
//   root_pt[0x200] = pointer-to-next-level → leaf_pt    = 0x20040401
//   leaf_pt[0]     = leaf 4KB 代码段 (PA 0x80000000)    = 0x2000004B
//   leaf_pt[1]     = leaf 4KB 数据段 (PA 0x80200000)    = 0x200800C7
//
// PTE 位号跟 src/riscv.h PTE_* 宏一致 (V/R/W/X/U/G/A/D 对应 bit 0-7); 这里 fixture
// 自带宏副本, 不 include 项目头文件 (fixture 走 riscv64-unknown-elf-gcc 交叉编译, 不
// 共享 host include path)。

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

    // root_pt[0x200] (vaddr 0x80000000 段 4MB) = pointer-to-next-level → leaf_pt
    // PPN(LEAF_PT_PA) = LEAF_PT_PA >> 12 = 0x80101
    // PTE bits[31:10] = PPN, low bits R=W=X=0 (非叶子) + V=1
    root_pt[0x200] = ((LEAF_PT_PA >> 12) << 10) | PTE_V;          // 0x20040401

    // leaf_pt[0] (vaddr 0x80000000 4KB) → PA 0x80000000 (代码段; X 不要 W; U=0 给 S)
    // PPN(0x80000000) = 0x80000
    leaf_pt[0] = (0x80000U << 10) | PTE_V | PTE_R | PTE_X | PTE_A; // 0x2000004B

    // leaf_pt[1] (vaddr 0x80001000 4KB) → PA 0x80200000 (数据段; R+W; U=0; A+D 1
    // 让 fast path 命中 — D=0 的 fall back 路径留 fixture 05 单测)
    // PPN(0x80200000) = 0x80200
    leaf_pt[1] = (0x80200U << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D;  // 0x200800C7
}
