// a_01_10 / 01_sum_load_fast_path — setup_pt() + 数据段预填
//
// xv6 main.c 风格的 C 函数容器 (跟 a_01_8 fixture 04 同模板)。不调 libc, 用
// volatile uint32_t* 直接写 PA — 跑在 _start 调用栈内 (M-mode BARE), satp 还没设,
// walker 未激活, 写 PA 直接走 host RAM identity。
//
// PT 三个 entry (跟 fixture 04 几乎一致, 唯一区别 leaf_pt[1] 加 PTE_U=1):
//   root_pt[0x200] = pointer-to-next-level → leaf_pt    = 0x20040401
//   leaf_pt[0]     = leaf 4KB 代码段 (PTE_U=0)          = 0x2000004B
//   leaf_pt[1]     = leaf 4KB 数据段 (**PTE_U=1**)      = 0x20080053
//
// 跟 fixture 04 区别:
//   - leaf_pt[1] 加 PTE_U=1 (user page) — 核心验证点 — 让 S 模式访问受 mstatus.SUM 控制
//   - leaf_pt[1] 去 PTE_W (load 路径不需要 W) + 去 PTE_D (load 路径 walker 不 set D)
//   - 数据段 PA 0x80200000 预填 0xDEADBEEF (s_phase_sum1 lw 验值)

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
    volatile uint32_t *data_pg = (volatile uint32_t *)DATA_PA;

    // root_pt[0x200] (vaddr 0x80000000 段 4MB) = pointer-to-next-level → leaf_pt
    root_pt[0x200] = ((LEAF_PT_PA >> 12) << 10) | PTE_V;          // 0x20040401

    // leaf_pt[0] (vaddr 0x80000000 4KB) → PA 0x80000000 代码段 (PTE_U=0, R+X+A)
    // S 模式从此 page fetch — PTE_U=0 让 S 通过, X+R+A=1
    leaf_pt[0] = (0x80000U << 10) | PTE_V | PTE_R | PTE_X | PTE_A; // 0x2000004B

    // leaf_pt[1] (vaddr 0x80001000 4KB) → PA 0x80200000 数据段 PTE_U=1
    // **核心验证点**: PTE_U=1 让 S 模式 lw 受 mstatus.SUM 控制
    //   SUM=1 → check_perm 通过 → lw 成功
    //   SUM=0 → check_perm 拒绝 → load page fault cause 13
    // R+A=1 让 fast path 命中 (load 路径 walker 不 set D, 这里 D 不 set 也能命中,
    // check_perm(R) 不查 D)
    leaf_pt[1] = (0x80200U << 10) | PTE_V | PTE_R | PTE_U | PTE_A; // 0x20080053

    // 预填数据段: 0x80200000 = 0xDEADBEEF (SUM=1 lw 验读到此值)
    data_pg[0] = 0xDEADBEEFU;
}
