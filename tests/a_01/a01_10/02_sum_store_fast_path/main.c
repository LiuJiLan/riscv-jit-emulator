// a_01_10 / 02_sum_store_fast_path — setup_pt() + 数据段预填
//
// 跟 fixture 01 (load SUM 验证) 互补 — 02 验 store 路径 SUM corner case。
// PT layout 跟 fixture 01 几乎一致, 唯一区别 leaf_pt[1] 加 PTE_W (store 需要 W=1) +
// 加 PTE_D (让 store fast path 命中 V+tag+D, 不 fall back walker)。
//
// PT 三个 entry:
//   root_pt[0x200] = pointer-to-next-level → leaf_pt    = 0x20040401
//   leaf_pt[0]     = leaf 4KB 代码段 (PTE_U=0, R+X+A)   = 0x2000004B
//   leaf_pt[1]     = leaf 4KB 数据段 (PTE_U=1, R+W+A+D) = 0x200800D7
//
// 跟 fixture 01 区别:
//   - leaf_pt[1] 加 PTE_W (store 路径 check_perm(W) 必查 W=1) + 加 PTE_D (让 store
//     fast path V+tag+D 命中, 不 fall back walker hw-managed D 路径)
//   - 数据段不预填 (sw 写值后 lw 回读验)

#include <stdint.h>

#define ROOT_PT_PA   0x80100000U
#define LEAF_PT_PA   0x80101000U
#define DATA_PA      0x80200000U

#define PTE_V  0x01U
#define PTE_R  0x02U
#define PTE_W  0x04U
#define PTE_X  0x08U
#define PTE_U  0x10U
#define PTE_A  0x40U
#define PTE_D  0x80U

void setup_pt(void) {
    volatile uint32_t *root_pt = (volatile uint32_t *)ROOT_PT_PA;
    volatile uint32_t *leaf_pt = (volatile uint32_t *)LEAF_PT_PA;

    // root_pt[0x200] (vaddr 0x80000000 段 4MB) = pointer-to-next-level → leaf_pt
    root_pt[0x200] = ((LEAF_PT_PA >> 12) << 10) | PTE_V;          // 0x20040401

    // leaf_pt[0] (vaddr 0x80000000 4KB) → PA 0x80000000 代码段 (PTE_U=0, R+X+A)
    leaf_pt[0] = (0x80000U << 10) | PTE_V | PTE_R | PTE_X | PTE_A; // 0x2000004B

    // leaf_pt[1] (vaddr 0x80001000 4KB) → PA 0x80200000 数据段 PTE_U=1+W+D
    // **核心验证点**: PTE_U=1 让 S 模式 sw 受 mstatus.SUM 控制
    //   SUM=1 → check_perm(W) 通过 → sw 成功
    //   SUM=0 → check_perm(W) 拒绝 → store page fault cause 15
    // R+W+A+D=1 让 store fast path 命中 V+tag+D (不 fall back walker hw-managed D);
    // R 顺手设是因 walker 内 W=1+R=0 reserved 项目跟 spike 风格不查, 但设 R=1 更稳
    leaf_pt[1] = (0x80200U << 10)
                 | PTE_V | PTE_R | PTE_W | PTE_U | PTE_A | PTE_D;  // 0x200800D7
}
