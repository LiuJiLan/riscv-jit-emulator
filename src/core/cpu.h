//
// Created by liujilan on 2026/4/28.
// cpu 模块对外接口。
//
// cpu_t = 单 hart 的 guest CPU 状态镜像 (per-hart, 无并发, 普通读写)。
//
// 字段布局规约 (与 dummy.txt §2 协议一致):
//   regs[32], 单一连续数组
//     - regs[0] 物理上占 x0 的位置, 但实际存 pc
//     - regs[1..31] = x1..x31
//     - x0 由 translator / interpreter 特殊路径处理 (读 = 字面量 0, 写 = 丢弃),
//       永远不真碰 regs[0]
//   priv / satp / jmp_buf_ptr / tlb_table[4]
//
// 这种 regs[32] 而非 pc + regs[31] 的布局是有意设计:
//   - decoded rd 是 5-bit 数字, 可以直接 regs[rd] 索引, 不需要 rd-1 减法
//   - JIT emit 出的 host 代码计算 cpu_base + N*4 直接 load/store, 翻译规则统一
//   - dummy.txt §2 文字描述的"pc 在偏移 0"在物理上就是 regs[0]
// 伪代码 / 注释中的 hart->pc / cpu->pc 是表意, 实际访问点请用 regs[0]。
//
// jmp_buf_ptr 实体在 dispatcher 栈, cpu_t 只持指针, 见 dummy.txt §1。
// tlb_table[4] 4 槽语义见 tlb.h 顶部注释 (v3 对称设计下元素类型统一为 tlb_t **,
//   ASID 数组容器, 直接索引 [asid] 拿 tlb_t *, 不需要 cast)。
//
// 未来扩展:
//   - isa/fpu 进来时加 fcsr 指针 (POD 外挂, 不嵌入主结构)
//   - monitor / 调试需要时加 perf_counters 指针
//   - csr 状态 (mip / mie / mideleg / medeleg ...) 真接 OS / S-mode 时加
//
// 报错风格见 src/dummy.txt §5。
//

#ifndef CORE_CPU_H
#define CORE_CPU_H

#include <setjmp.h>
#include <stdint.h>

#include "tlb.h"   // tlb_t * 类型 (tlb_table 元素是 tlb_t **)
#include "trap.h"  // trap_csrs_t 内嵌字段类型 (trap.h forward decl cpu_t,
                   // 单向链 trap.h ← cpu.h, 不循环)

// struct tag "cpu_s" 是为了让 trap.h 能 forward typedef cpu_t (trap.h helper 签名要 cpu_t*,
// 但 trap.h 不能 #include cpu.h 形成循环)。tag 名 cpu_s 仅服务 forward decl, 没人直接用
// "struct cpu_s" 这个写法; 项目代码全用 cpu_t typedef。零运行时成本 (struct 加 tag 不改
// ABI / 字段布局)。
// cpu_info_per_hart_t (类 4: per-hart 私有 RO CSR 数据; 见 dummy.txt §6)
//
// per-hart 私有 — 异构 SMP (e.g. 1×MU + 4×MSU) 时不同 hart 的字段值不同, 不能共享。
// mhartid: hartid 编号; misa: 该 hart 实际支持的扩展 (MU 跟 MSU 的 misa 字段不一样)。
// 嵌入 cpu_t 字段 (非指针; per-hart 私有就跟着 cpu_t 走, 不需要外部 alloc)。
typedef struct cpu_info_per_hart_s {
    uint32_t mhartid;     // RV spec §3.1.5; per-hart 不同; csr_mhartid_read 直读
    uint32_t misa;        // RV spec §3.1.1; bit 30 = MXL (RV32 = 1); bits[25:0] = extension flags;
                          //   异构 SMP 时不同 hart 可不同 (例如某些 hart 不带 S-mode)
} cpu_info_per_hart_t;

// cpu_info_shared_t (类 4: 多 hart 共享 RO CSR 数据; 见 dummy.txt §6)
//
// 制造商信息类 — 全机器统一 (mvendorid/marchid/mimpid 都是机器整体属性, 不区分 hart)。
// 多 hart 共享一份 (cpu.c 内 static const cpu_info_shared_default); cpu_t 内持指针。
// 只放 CSR — cache_size / tlb_size 等"硬件参数"不进这里, 进 config.h (编译期宏)。
typedef struct cpu_info_shared_s {
    uint32_t mvendorid;   // RV spec §3.1.2; JEDEC vendor ID; 0 = open-source / no JEDEC
    uint32_t marchid;     // RV spec §3.1.3; architecture ID;  0 = no architecture ID
    uint32_t mimpid;      // RV spec §3.1.4; implementation ID; 0 = no impl ID
} cpu_info_shared_t;

typedef struct cpu_s {
    // _Alignas(64) 在第一字段, 强制整个 struct 以 64B (cache line) 对齐。
    // cpu_create 用 aligned_alloc(64, sizeof(cpu_t)) 分配, 与之配套。
    //
    // regs[0] 实际是 pc (物理占 x0 位置, x0 走特殊路径不碰 regs[0])
    // regs[1..31] = x1..x31, offset(reg N) = N * 4
    _Alignas(64) uint32_t regs[32];
    uint8_t               priv;             // RV privilege encoding (riscv.h PRIV_*); 当前启动 PRIV_M
    uint32_t              satp;             // Sv32 satp; 当前 0 (bare; MODE=0, ASID=0, PPN=0)
    sigjmp_buf           *jmp_buf_ptr;      // 实体在 dispatcher 栈, 见 dummy.txt §1
    tlb_t               **tlb_table[4];     // 4 槽派发数组, 语义见 tlb.h 顶部
    trap_csrs_t              trap;             // trap-related CSR 镜像 + host trap 流程状态
                                                // (in_trap 计数器), 内嵌后置, ~80 B; 设计意图
                                                // 见 trap.h 顶部 doc + dummy.txt §1
    cpu_info_per_hart_t      per_hart_info;     // per-hart 私有 RO CSR (mhartid + misa);
                                                //   嵌入 (非指针, 跟 cpu_t 走); cpu_create
                                                //   入参 mhartid + misa 写入
    const cpu_info_shared_t *shared_info;       // 多 hart 共享 RO CSR (mvendorid + marchid +
                                                //   mimpid); 指向 cpu.c static const
                                                //   cpu_info_shared_default
} cpu_t;

// 工厂: 分配 (cache-line 对齐) + 初始化 cpu_t。POR 一次性调用 (main 入口)。
// 失败返回 NULL, 内部已 fprintf。
//
// 内部含 RV "硬件 reset 后默认状态" 写入 (regs[0]=GUEST_RAM_START / priv=PRIV_M /
// satp=0 / regs[10]=mhartid 等; 跟 cpu_reset 一致), 所以 main 拿到 cpu_t 直接
// 可进 dispatcher, 不需要 main 端再写 hart 字段。
//
// TODO future: 按 misa 派发 cpu_t 子结构 alloc — F/D 扩展按 misa.fdv 决定 fcsr
// alloc, H 扩展按 misa.h 决定 [PRIV_H] tlb 容器 alloc 等。当前 misa 字段只作
// csr_misa_read 返回值, 不真做运行时派发。
//
// 入参:
//   misa    — 写入 hart->per_hart_info.misa (csr_misa_read 直读); per-hart 私有, 异构 SMP
//             时不同 hart 可不同 (例如某些 hart 不带 S-mode 扩展位)。
//   mhartid — 写入 hart->per_hart_info.mhartid (csr_mhartid_read 直读); per-hart 不同。
cpu_t *cpu_create(uint32_t misa, uint32_t mhartid);

// system reset 每 iter 调 (main while 顶段): 重置 RV-spec reset-state 字段, 让
// hart 从"硬件 reset 后状态"重新跑。
//
// 重置:
//   regs[0] = GUEST_RAM_START   (pc 启动点 / reset vector)
//   regs[1..31] = 0; 但 regs[10] = mhartid (a0; RV Linux boot 协议)
//   regs[11] = 0                 (a1 = dtb 占位, 未来)
//   priv    = PRIV_M
//   satp    = 0                  (bare; dispatcher 再选 leaf TLB)
//   trap    全 memset 0          (xcause/xtval/xepc/xtvec/xscratch 各 [4] +
//                                 _mstatus/_medeleg/mideleg/_mie/_mip_sw/in_trap)
//   tlb_table 容器 不动, entries 调 tlb_table_reset(hart) 清
//
// 保留 (硬件 ID 类, 真硬件 reset 后不变):
//   per_hart_info (mhartid / misa); shared_info 指针 (mvendorid/marchid/mimpid)
//   jmp_buf_ptr (dispatcher 进入时重设, reset 时不动 NULL 也 OK)
//
// TODO future: 按 hart 内部 misa 派发 (运行时 misa 切换决定哪些 CSR 字段要清零;
// 当前 misa 字段不实装运行时切换, 全 hart 同 reset 序)。
void cpu_reset(cpu_t *hart);

// 释放 cpu_create 分配的 cpu_t 及其下所有子结构 (tlb 容器 + 共享 leaf + 已懒分配的 entries)。
// NULL 入参 do nothing。
void cpu_destroy(cpu_t *hart);

#endif //CORE_CPU_H
