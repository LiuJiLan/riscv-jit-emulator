//
// Created by liujilan on 2026/4/28.
// cpu 模块实现 (cpu_create / cpu_destroy)。
//
// tlb_table[4] 设计:
//   - REGIME_BARE 走 identity bypass TLB → [PRIV_M] 槽永远 NULL, 不分配
//   - REGIME_SV32 走 [priv][asid] 二级索引 → [PRIV_S] 容器 eager 分配, entries 懒 (walker 填)
//   - [PRIV_U] alias [PRIV_S] (MSU 默认; U 共享 S 的 ASID 命名空间)
//   - [PRIV_H] VS 占位, NULL (未来 H 扩展)
//
// 报错风格见 src/dummy.txt §5。
//

#include "cpu.h"

#include "config.h"
#include "riscv.h"
#include "tlb.h"

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// cpu_info_shared_default — 多 hart 共享出场信息 RO CSR (cpu_t::shared_info 指向)
//
// 制造商信息 (mvendorid/marchid/mimpid) 是机器整体属性, 不区分 hart; 当前 open-source
// 项目全 0 (RV spec: 0 = no JEDEC vendor / no architecture ID / no impl ID, 合法值)。
// 异构 SMP 时这里加多份 cpu_info_shared_default_a/b 给不同 hart 组; 项目当前同构, 单份。
//
// 注: misa 跟 mhartid 不在这里 (per-hart 私有, 进 cpu_info_per_hart_t — 异构 SMP 时
// 不同 hart 的字段值不同; mhartid 显然 per-hart 不同, misa 也可能 per-hart 不同)。
// 015 dual storage: hart->per_hart_info.mhartid (CSR 镜像) + cpu_t.hartid (index 用),
// 都由 cpu_create 入参 mhartid 写; 详 cpu.h cpu_t.hartid 字段注释。
static const cpu_info_shared_t cpu_info_shared_default = {
    .mvendorid = 0,
    .marchid   = 0,
    .mimpid    = 0,
};

cpu_t *cpu_create(uxlen_t misa, uxlen_t mhartid) {
    // init 依赖 misa (future) — 当前只把 misa 值存入 per_hart_info.misa 给
    // csr_misa_read 用, 不按 misa 真做派发 (F/D 扩展按 misa.fdv 决定 fcsr alloc /
    // H 扩展按 misa.h 决定 [PRIV_H] tlb 容器 alloc 等都属于未来)。
    cpu_t *hart = aligned_alloc(64, sizeof(cpu_t));
    if (hart == NULL) {
        fprintf(stderr, "cpu_create: aligned_alloc(64, %zu) failed: %s\n",
                sizeof(cpu_t), strerror(errno));
        return NULL;
    }
    memset(hart, 0, sizeof(*hart));

    // dual storage of hartid (015):
    //   per_hart_info.mhartid (uxlen_t) — CSR 镜像; csr_mhartid_read 直读;
    //                                      跟 misa 同 per_hart_info container 走 RV spec
    //                                      §3.1.5 MXLEN-bit 体例 (dummy.txt §13).
    //   cpu_t.hartid (uint32_t)         — index 用; clint_per_hart[hartid] /
    //                                      wfi_slots[hartid] / wfi_kick(hartid) /
    //                                      is_clint_*_pending(hartid) 等高频 index
    //                                      入参直读不 cast (§13 index 非目标 uint32_t).
    // 同时写, lifetime 内都不变 (mhartid RO + hartid 硬件 wired), 不会不同步。
    hart->per_hart_info.mhartid = mhartid;
    hart->hartid                = (uint32_t)mhartid;

    // per-hart 私有 RO CSR 数据 (misa); cpu_create 入参直接写入 cpu_t.per_hart_info。
    // misa 入参当前仅作为 csr_misa_read 返回值 (不按 misa 派发 lifecycle, 例如 F/D
    // 扩展按 misa.fdv 决定 fcsr alloc — 那是未来)。
    hart->per_hart_info.misa    = misa;

    // 多 hart 共享 RO CSR 数据 (mvendorid/marchid/mimpid); 指针指向全局 static const default。
    hart->shared_info = &cpu_info_shared_default;

    // ------------------------------------------------------------------------
    // RV "硬件 reset 后默认状态" 写入 (regs / priv / satp; 原 main.c hart 字段
    // 初始化段 L314~L321 挪入)。跟 cpu_reset 内的写入序列一致 — POR 一次性 init
    // 等同于 "刚加电 + 第一次 reset" 后的 hart 状态。详 cpu_reset doc 段。
    //
    // 启动状态参考 https://docs.kernel.org/arch/riscv/boot.html
    // ------------------------------------------------------------------------
    hart->regs[0]   = GUEST_RAM_START;      // pc = 程序起点 (reset vector)
    hart->regs[10]  = mhartid;              // a0 = mhartid (Linux RV boot 协议; uxlen_t 入参直传)
    // regs[11] = 0 (a1 = dtb 占位, 未来; memset 已置 0)
    hart->priv      = PRIV_M;               // M 模式 (启动)
    hart->satp      = 0;                    // bare 模式 (MODE=0, ASID=0, PPN=0 全 0)

    // mstatus.MDT (Smdbltrp) + sstatus.SDT (Ssdbltrp) reset 初值 = 1 — spec
    // §3.1.6.2 / §4.1.1.5 明确要求 "Upon reset, the MDT/SDT field is set to 1".
    // 其他 _mstatus 字段 memset 0 已覆盖。意味着 reset 后:
    //   - mstatus.MIE/SIE 默认 0 (MDT/SDT=1 时硬件不允许 MIE/SIE=1)
    //   - 第一次 trap 之前, boot ROM / SBI 要 csrw mstatush 清 MDT, 才能开 MIE
    hart->trap._mstatus |= MSTATUS_MDT_BIT64 | (u64_t)MSTATUS_SDT;

    // ------------------------------------------------------------------------
    // tlb_table[4] 装载 (Trust regime bypass TLB; 当前默认 MSU)
    //
    // [PRIV_M] M/bare 槽: 永远 NULL。Trust regime (M-mode 或任何 priv 带 bare satp) 直接
    //                     走 identity + IS_GPA_RAM 检查, 不需要 TLB; real CPU bare 下也
    //                     bypass MMU/TLB, 我们对齐。memset 0 已置 NULL, 不需显式赋值。
    //
    // [PRIV_S] S 槽: ASID 容器, entries 由 walker 在 Sv32 路径懒分配。
    //
    // [PRIV_U] U 槽: 始终是副本语义 (U 是 S 或 M 的副本, 取决于 misa 实际配置)。即使
    //                MU-only CPU 中 U 副本 M 态、槽内 NULL (M 走 bare 不查 TLB), "副本"
    //                语义本身不变。副本分配两路:
    //                  - 初始化: cpu_create 负责 (未来根据 misa 派发: MSU 副本 S; MU-only
    //                            副本 M, 即 NULL)
    //                  - 运行时: H 扩展激活 (VS / VU 等切换) 由对应 csr_helper 维护 mirror
    //                当前默认 MSU, 直接赋值 S 槽 (U 与 S 共享 ASID 命名空间)。
    //
    // [PRIV_H] VS 占位: NULL (memset 0 已置), 未来 H 扩展激活时 calloc。
    //
    // memset 0 已经把 tlb_table[0..3] 置 NULL, 失败回滚路径上 free(NULL) 无害。
    // ------------------------------------------------------------------------

    // [PRIV_S] S: ASID 容器, entries 由 walker 在 Sv32 路径懒分配。
    hart->tlb_table[PRIV_S] = calloc(ASID_MAX, sizeof(tlb_t *));
    if (hart->tlb_table[PRIV_S] == NULL) {
        fprintf(stderr, "cpu_create: calloc tlb_table[PRIV_S] failed: %s\n", strerror(errno));
        cpu_destroy(hart);
        return NULL;
    }

    // [PRIV_U] U: alias [PRIV_S] (MSU 默认; U 共享 S 的 ASID 命名空间)。
    hart->tlb_table[PRIV_U] = hart->tlb_table[PRIV_S];

    // [PRIV_M] / [PRIV_H]: 不分配, memset 0 保证 NULL。

    return hart;
}

void cpu_reset(cpu_t *hart) {
    // reset 依赖 misa (future) — 真硬件 reset 后哪些字段清 / 哪些保留按 misa 决定
    // (例 misa.F=0 时 fcsr 不存在不需清; misa.H=0 时 [PRIV_H] tlb 容器不存在不需
    // tlb_table_reset 内遍历)。当前不实装运行时 misa 切换, 全 hart 同 reset 序。
    if (hart == NULL) return;

    // regs: 全清 0, 然后写 RV-spec reset 后的启动协议字段。
    // (顺序: 先 memset, 再写需要非 0 的 — regs[0]=pc / regs[10]=a0)
    memset(hart->regs, 0, sizeof(hart->regs));
    hart->regs[0]  = GUEST_RAM_START;            // pc = reset vector
    hart->regs[10] = hart->per_hart_info.mhartid; // a0 = mhartid (Linux RV boot 协议; uxlen_t 直传)
    // regs[11] = 0  (a1 = dtb 占位, memset 已 0)

    // 控制状态: priv = M / satp = 0 (bare)
    hart->priv = PRIV_M;
    hart->satp = 0;

    // trap_csrs_t 全 memset 0 — xcause/xtval/xepc/xtvec/xscratch 各 [4] +
    // _mstatus/_medeleg/mideleg/_mie/_mip_sw/mtval2 等所有字段一次清。
    // jmp_buf_ptr / per_hart_info / shared_info 不在 trap 内, 不影响。
    memset(&hart->trap, 0, sizeof(hart->trap));

    // mstatus.MDT + sstatus.SDT reset 初值 = 1 (Smdbltrp/Ssdbltrp 扩展; spec
    // §3.1.6.2 / §4.1.1.5)。跟 cpu_create 同序; memset 0 之后单独 OR 设置。
    hart->trap._mstatus |= MSTATUS_MDT_BIT64 | (u64_t)MSTATUS_SDT;

    // tlb_table 容器不动 (跟 sfence 形态一致); entries 全清 (跨地址空间防 PTE
    // 残留撞 hash 命中)。详 tlb.h tlb_table_reset doc 段。
    tlb_table_reset(hart);

    // 保留 (硬件 ID 类, 真硬件 reset 后不变):
    //   hartid (顶层 uint32_t) + per_hart_info (mhartid + misa); shared_info 指针
    //   (mvendorid 等); jmp_buf_ptr (dispatcher 进入时重设永久落点, reset 时不动也无害)
}

// ----------------------------------------------------------------------------
// cpu_dump — CPU 寄存器 / trap / state dump (DEBUG_CPU_DUMP_ON gate)
//
// cpu_destroy 销毁 hart 前调一次, 把终态打到 stderr 让 fixture 肉眼对照期望值。
// 受 DEBUG_CPU_DUMP_ON gate (CMake 非 Release 配置才开; 见 debug.h / CMakeLists.txt)
// —— Release 自动化测试不打 dump。
//
// dump 格式:
//   - reg 分 [reg dec] + [reg hex] 两段, 每段 x1-x31 (x0 跳过, 占位 pc)
//   - ABI 标注 xN(abi), space padding 到 9 char 宽度对齐 (x8(s0/fp) 9 char 不单独优化)
//   - 每行 4 reg (decimal %11u / hex 0x%08X), 行头 \t
//   - pc 放 state dump 末行, hex 显示
//
// privrd 在 csr.c 内 csrr 时直接 fprintf 流式输出, 不缓存到 cpu_t, 故 dump 不含。
// ----------------------------------------------------------------------------
static void cpu_dump(const cpu_t *hart) {
#ifdef DEBUG_CPU_DUMP_ON
    fprintf(stderr,
            "[cpu] reg dec:\n"
            "\tx1(ra)    = %11u  |  x2(sp)    = %11u  |  x3(gp)    = %11u  |  x4(tp)    = %11u\n"
            "\tx5(t0)    = %11u  |  x6(t1)    = %11u  |  x7(t2)    = %11u  |  x8(s0/fp) = %11u\n"
            "\tx9(s1)    = %11u  |  x10(a0)   = %11u  |  x11(a1)   = %11u  |  x12(a2)   = %11u\n"
            "\tx13(a3)   = %11u  |  x14(a4)   = %11u  |  x15(a5)   = %11u  |  x16(a6)   = %11u\n"
            "\tx17(a7)   = %11u  |  x18(s2)   = %11u  |  x19(s3)   = %11u  |  x20(s4)   = %11u\n"
            "\tx21(s5)   = %11u  |  x22(s6)   = %11u  |  x23(s7)   = %11u  |  x24(s8)   = %11u\n"
            "\tx25(s9)   = %11u  |  x26(s10)  = %11u  |  x27(s11)  = %11u  |  x28(t3)   = %11u\n"
            "\tx29(t4)   = %11u  |  x30(t5)   = %11u  |  x31(t6)   = %11u\n",
            hart->regs[1],  hart->regs[2],  hart->regs[3],  hart->regs[4],
            hart->regs[5],  hart->regs[6],  hart->regs[7],  hart->regs[8],
            hart->regs[9],  hart->regs[10], hart->regs[11], hart->regs[12],
            hart->regs[13], hart->regs[14], hart->regs[15], hart->regs[16],
            hart->regs[17], hart->regs[18], hart->regs[19], hart->regs[20],
            hart->regs[21], hart->regs[22], hart->regs[23], hart->regs[24],
            hart->regs[25], hart->regs[26], hart->regs[27], hart->regs[28],
            hart->regs[29], hart->regs[30], hart->regs[31]);

    fprintf(stderr,
            "[cpu] reg hex:\n"
            "\tx1(ra)    = 0x%08X  |  x2(sp)    = 0x%08X  |  x3(gp)    = 0x%08X  |  x4(tp)    = 0x%08X\n"
            "\tx5(t0)    = 0x%08X  |  x6(t1)    = 0x%08X  |  x7(t2)    = 0x%08X  |  x8(s0/fp) = 0x%08X\n"
            "\tx9(s1)    = 0x%08X  |  x10(a0)   = 0x%08X  |  x11(a1)   = 0x%08X  |  x12(a2)   = 0x%08X\n"
            "\tx13(a3)   = 0x%08X  |  x14(a4)   = 0x%08X  |  x15(a5)   = 0x%08X  |  x16(a6)   = 0x%08X\n"
            "\tx17(a7)   = 0x%08X  |  x18(s2)   = 0x%08X  |  x19(s3)   = 0x%08X  |  x20(s4)   = 0x%08X\n"
            "\tx21(s5)   = 0x%08X  |  x22(s6)   = 0x%08X  |  x23(s7)   = 0x%08X  |  x24(s8)   = 0x%08X\n"
            "\tx25(s9)   = 0x%08X  |  x26(s10)  = 0x%08X  |  x27(s11)  = 0x%08X  |  x28(t3)   = 0x%08X\n"
            "\tx29(t4)   = 0x%08X  |  x30(t5)   = 0x%08X  |  x31(t6)   = 0x%08X\n",
            hart->regs[1],  hart->regs[2],  hart->regs[3],  hart->regs[4],
            hart->regs[5],  hart->regs[6],  hart->regs[7],  hart->regs[8],
            hart->regs[9],  hart->regs[10], hart->regs[11], hart->regs[12],
            hart->regs[13], hart->regs[14], hart->regs[15], hart->regs[16],
            hart->regs[17], hart->regs[18], hart->regs[19], hart->regs[20],
            hart->regs[21], hart->regs[22], hart->regs[23], hart->regs[24],
            hart->regs[25], hart->regs[26], hart->regs[27], hart->regs[28],
            hart->regs[29], hart->regs[30], hart->regs[31]);

    // trap dump (M+S 两槽): MDT=1 / SDT=1 表示 hart 在对应 priv trap handler 内
    // 还没退 (mret/sret 清); MDT/SDT 同时 1 + 再来 trap 就触发 critical-error /
    // double-trap 升级 (Smdbltrp/Ssdbltrp). S 槽 mtval2 显示 double-trap 升级时
    // 原本要写 stval 的值 (S-trap unexpected 路径).
    fprintf(stderr,
            "[cpu] trap dump (M): MDT=%u mcause=%u mtval=0x%08X mepc=0x%08X mtvec=0x%08X mtval2=0x%08X\n",
            (uint32_t)((hart->trap._mstatus & MSTATUS_MDT_BIT64) != 0u),
            hart->trap.xcause[PRIV_M], hart->trap.xtval[PRIV_M],
            hart->trap.xepc[PRIV_M],   hart->trap.xtvec[PRIV_M],
            hart->trap.mtval2);
    fprintf(stderr,
            "[cpu] trap dump (S): SDT=%u scause=%u stval=0x%08X sepc=0x%08X stvec=0x%08X\n",
            (uint32_t)((hart->trap._mstatus & (uint64_t)MSTATUS_SDT) != 0u),
            hart->trap.xcause[PRIV_S], hart->trap.xtval[PRIV_S],
            hart->trap.xepc[PRIV_S],   hart->trap.xtvec[PRIV_S]);

    // state dump: pc + priv + mstatus 低 32 位 + mstatush (含 MDT bit10)。
    fprintf(stderr,
            "[cpu] state dump: pc=0x%08X  priv=%u  mstatus=0x%08X mstatush=0x%08X\n",
            hart->regs[0],
            (uint32_t)hart->priv,
            (uint32_t)(hart->trap._mstatus & 0xFFFFFFFFu),
            (uint32_t)((hart->trap._mstatus >> 32) & 0xFFFFFFFFu));
#else
    (void)hart;
#endif
}

void cpu_destroy(cpu_t *hart) {
    if (hart == NULL) return;

    // 销毁前 dump 终态 (DEBUG_CPU_DUMP_ON gate; Release 不打)。放 free 之前 hart 字段
    // 仍完整可读。注: cpu_create 失败回滚路径也走 cpu_destroy, 那时 hart 只半初始化,
    // dump 出来多是 0 值 —— 属错误路径, 无害 (失败原因前面已 fprintf 过)。
    cpu_dump(hart);

    // [PRIV_S] S: 递归 free 非 NULL entries (walker 懒分配的叶 TLB), 然后 free 容器。
    if (hart->tlb_table[PRIV_S] != NULL) {
        for (uint32_t i = 0; i < ASID_MAX; i++) {
            free(hart->tlb_table[PRIV_S][i]);    // free(NULL) 无害, 未懒分配的槽位天然 no-op
        }
        free(hart->tlb_table[PRIV_S]);
    }

    // [PRIV_H] V 占位: 同 [PRIV_S] 模式; 当前始终 NULL, 循环天然 no-op, 写出来未来 H 扩展不漏。
    if (hart->tlb_table[PRIV_H] != NULL) {
        for (uint32_t i = 0; i < ASID_MAX; i++) {
            free(hart->tlb_table[PRIV_H][i]);
        }
        free(hart->tlb_table[PRIV_H]);
    }

    // [PRIV_U] U: 副本语义, 跟所副本的槽 (S 或 M) 共享内存, 不在这里 free。
    // [PRIV_M] M: 不分配 (Trust regime 不走 TLB), 不再 free。

    free(hart);
}
