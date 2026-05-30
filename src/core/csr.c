//
// Created by liujilan on 2026/5/4.
// csr 模块实现 (csr_op 大 helper + 各小 r/w helper 真读写 hart->trap / hart->satp 字段)。
//
// 顶部模块文档见 csr.h; 跨文件协议见 src/dummy.txt §1。
//
// csr 编号 → 字段映射:
//   mstatus  (0x300) → trap._mstatus 低 32 位 (mstatus 物理 64 位被 RV32 拆 mstatus + mstatush 两 csr)
//   mstatush (0x310) → trap._mstatus 高 32 位 (RV32-only csr 入口)
//   mtvec    (0x305) → trap.xtvec[PRIV_M]; write WARL MODE bit[1:0] = 0/1 都接受, 2/3 reserved → 0
//   mepc     (0x341) → trap.xepc[PRIV_M];  write WARL mask 低 IALIGN_MASK 位 (RV spec mepc[0]=0
//                       when IALIGN=16; mepc[1:0]=0 when IALIGN=32)
//   mcause   (0x342) → trap.xcause[PRIV_M]
//   mtval    (0x343) → trap.xtval[PRIV_M]
//   mscratch (0x340) → trap.xscratch[PRIV_M]
//   medeleg  (0x302) → trap._medeleg 低 32 位
//   mideleg  (0x303) → trap.mideleg (类 3 单字段; trap_set_interrupt_state + trap_check_interrupt 真读)
//   mie      (0x304) → trap._mie (类 (2a) 副本基本字段, sie 是 mask view; WARL 截 MIE_VALID_MASK)
//   mip      (0x344) → 合成读: trap._mip_sw (类 (5) 软件可写子集) OR is_clint_msip_pending OR
//                       is_clint_timer_pending OR is_plic_meip_pending OR is_plic_seip_pending;
//                       csrw 只动 _mip_sw, RO 位忽略
//   satp     (0x180) → hart->satp (cpu_t 直接持有字段, 不在 trap_csrs_t — satp 不属于 trap-related
//                       CSR 范畴; write WARL ASID 截断到 ASID_MASK 位, 见 dummy.txt §3)
//   sstatus  (0x100) → trap._mstatus 低 32 位 ∩ SSTATUS_MASK (mask 视图; 物理共用 mstatus)
//   sepc     (0x141) → trap.xepc[PRIV_S];  WARL 截 IALIGN 对齐位 (跟 mepc 同)
//   sscratch (0x140) → trap.xscratch[PRIV_S]
//   stvec    (0x105) → trap.xtvec[PRIV_S];  WARL MODE 0/1 都接受 (跟 mtvec 同)
//   scause   (0x142) → trap.xcause[PRIV_S]
//   stval    (0x143) → trap.xtval[PRIV_S]
//   sie      (0x104) → trap._mie & SIE_MASK (mask view; 写时只动 SIE_MASK 子集)
//   sip      (0x144) → csr_mip_read(hart) & SIP_MASK (合成的 mask view); 写仅 SSIP
//                       (SIP_WRITABLE_MASK = 1<<IRQ_S_SOFT; STIP/SEIP RO in sip)
//   mhartid  (0xF14) → hart->per_hart_info.mhartid (RO; uxlen_t CSR 镜像直读;
//                       dual storage: cpu_t.hartid 是 uint32_t index 镜像不走此路径)
//   misa     (0x301) → hart->per_hart_info.misa (RW-effective-RO; write WARL noop)
//   mvendorid(0xF11) → hart->shared_info->mvendorid (RO)
//   marchid  (0xF12) → hart->shared_info->marchid (RO)
//   mimpid   (0xF13) → hart->shared_info->mimpid (RO)
//   time     (0xC01) → clint_read_mtime() 低 32 位 (RV Unpriv Spec Ch 10; RO)
//   timeh    (0xC81) → clint_read_mtime() 高 32 位 (RV32 only; RO)
//   privrd   (0xCC0) → 不存字段; read 直接 fprintf "[priv] X" + return hart->priv (临时 RO)
//
// 组织哲学:
//   类 1 — 扩展 CSR (F/V/Debug 等): 字段 + 函数都在 isa/<扩展>.{c,h}; csr.c 不放, 仅 csr_op
//          大 switch case dispatch 到对应模块 extern 接口。项目当前没实现, 占位说明。
//   类 2 — 跨模块 CSR (satp): 字段在 cpu_t; 函数留 csr.c (csr 入口集中)。sfence 跟 satp
//          写是运行期协议 (dummy.txt §3), 不是代码组织耦合。
//   类 3 — 核心 CSR (_mstatus, xtvec/xepc/xcause/xtval/xscratch, _medeleg, mideleg, sstatus/
//          sepc/sscratch/stvec/scause/stval): 字段在 trap_csrs_t, 函数留 csr.c。哲学: data
//          归 cpu_t, 动作分散在 isa/ + core/ (Linux struct task_struct 风格); cpu.c 只放
//          lifecycle (cpu_create / 字段初值)。
//   类 4 — 出场信息 RO CSR — 拆 per-hart 私有 + 多 hart 共享 两类:
//      4a per-hart 私有 (mhartid + misa): cpu_info_per_hart_t struct 嵌入 cpu_t (不指针;
//          per-hart 私有就跟着 cpu_t 走; cpu_t 释放时自然回收)。异构 SMP 时不同 hart 的 misa
//          不同 (例如 MU hart 不带 S-mode 扩展位), mhartid 也不同 (0/1/2/...) — 必须 per-hart
//          独立, 不能共享。cpu_create 入参 misa + mhartid 直接写入 hart->per_hart_info。
//          csr.c csr_mhartid/misa_read 读 hart->per_hart_info.xxx。
//
//          dual storage: mhartid 也窄化 (uint32_t) 写 cpu_t.hartid 顶层字段服务 index
//          用 (clint/plic/wfi 数组下标), csr 读 mhartid 仍走 per_hart_info 本字段 (uxlen_t).
//          详 cpu.h cpu_t.hartid 字段注释。
//      4b 多 hart 共享 (mvendorid + marchid + mimpid): cpu_info_shared_t struct + cpu_t 内
//          const cpu_info_shared_t *shared_info 指针; cpu.c static const cpu_info_shared_default
//          一份, 所有 hart shared_info 指向它。这些是机器整体属性, 不区分 hart。
//          csr.c csr_mvendorid/marchid/mimpid_read 读 hart->shared_info->xxx 解引用。
//   类 5 — 临时调试 CSR (privrd): 字段不存 cpu_t; csr.c 内 read 直接 fprintf 流式输出。
//          uart 实装后删除整段 (csr.c 内 helper + csr_op 内 case + riscv.h 宏)。
//

#include "csr.h"

#include "config.h"            // IALIGN_MASK
#include "cpu.h"
#include "platform/clint.h"    // is_clint_msip_pending / is_clint_timer_pending (mip 合成读)
                                // + clint_read_mtime (time / timeh CSR view)
#include "platform/plic.h"     // is_plic_meip/seip_pending (mip 合成读)
#include "riscv.h"
#include "trap.h"              // trap_raise_exception (csr_op 入口判 priv/RO 失败时长跳)

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>


// ============================================================================
// M-mode Trap-Related CSR (类 3)
//
// 含 mstatus/mstatush + mtvec/mepc/mcause/mtval/mscratch + medeleg/mideleg 共 9 个 csr;
// 每个 csr 一对 file-static r/w helper, csr_op 大 switch 调用。
// 命名规则 (与 trap.h 一致):
//   - mstatus / mstatush 操作 _mstatus (uint64_t) 的低/高 半边
//   - mtvec / mepc / mcause / mtval / mscratch 操作 xxx[PRIV_M] (priv-indexed, x 前缀)
//   - medeleg / medelegh 操作 _medeleg (uint64_t) 的低/高 32 位 (RV32 拆访问, 跟 mstatus/mstatush 同体例)
//   - mideleg 操作 mideleg (uint32_t, 类 3 MXLEN-bit 单字段; spec 无 midelegh)
// ============================================================================

// ---- mstatus 半边 (mstatus 物理 64 位, mstatus = 低 32, mstatush = 高 32) ----
//
// read 取对应半边; write 通过 mask 改半边保留另半边。
// sstatus 是 _mstatus 的 masked view, 见下方 sstatus 段。

static uxlen_t csr_mstatus_read(cpu_t *hart) {
    return (uint32_t)(hart->trap._mstatus & 0xFFFFFFFFu);
}

static void csr_mstatus_write(cpu_t *hart, uxlen_t v) {
    // 低 32 位换成 v (经 WARL 截断), 高 32 位保留。
    //
    // WARL 截断 (按字段递增加, 当前只截 MPP; MIE/MPIE 等不截):
    //   MPP (bits 12:11): 项目支持 priv 集 = {PRIV_U=0, PRIV_S=1, PRIV_M=3}; PRIV_H=2 在没 H
    //     扩展时非法。RV spec WARL 允许实现自由选择 fallback; 项目选 PRIV_U=0 落点 ("least-priv
    //     风险最小": mret 切到 U 比切到 M 安全)。
    //
    // !!! 未来 H 扩展 / SVxx-mode 真做时这里要改 !!!
    //   - H 扩展: PRIV_H=2 编码不再保留, 项目 priv 集变成 {U=0, S=1, H=2, M=3}, 写 2 不截
    //   - 严格按 spec 的实现可能选 fallback 到"上次合法值"而不是 PRIV_U; 那时 csr.c 需要保留
    //      MPP 历史状态 (file-static prev_mpp 之类)
    //
    // MIE / MPIE 当前不截 (RV spec 允许实现接受任意位值; 只是行为体现是中断使能)。
    // 中断机制真做时, 如果 MIE 字段需要更细的 WARL (比如某些子字段保留), 在这里加。
    {
        uint32_t mpp = (v >> MSTATUS_MPP_SHIFT) & MSTATUS_MPP_MASK;
        if (mpp == PRIV_H) {
            /* PRIV_H = 2, 项目当前不支持 → 落 PRIV_U = 0
             * 操作: 清 MPP 字段位; 因 PRIV_U = 0, 不需要再 OR 设新值。 */
            v &= ~MSTATUS_MPP;
        }
    }

    // SD bit (mstatus[31]) WARL — SD 是 RO summary (= FS|XS dirty 的或); 项目当前无 F/V,
    // FS=XS=00, SD 永远 0。截 SD 防 csrw 设假状态 (写 1 后读回 1, 但实际 FS/XS=00 SD 应 0)。
    // 未来加 F/V 时 SD 仍不允许 csrw 直接写, 由 FS/XS 写时硬件联动设。
    v &= ~MSTATUS_SD_RV32;

    // Smdbltrp §3.1.6.2: "The MIE bit can only be set to 1 by an explicit CSR write
    // if the MDT bit is already 0". RV32 下 MDT 在 mstatush (_mstatus bit 42),
    // 跨 csr 不能同写, 仅看当前 _mstatus.MDT 状态: MDT=1 时 csrw mstatus 写 MIE=1
    // 被强制清 0 (regardless of v 内 MIE 值).
    if ((hart->trap._mstatus & MSTATUS_MDT_BIT64) != 0u) {
        v &= ~MSTATUS_MIE;
    }

    hart->trap._mstatus = (hart->trap._mstatus & 0xFFFFFFFF00000000ULL)
                        | (uint64_t)v;
}

static uxlen_t csr_mstatush_read(cpu_t *hart) {
    return (uint32_t)((hart->trap._mstatus >> 32) & 0xFFFFFFFFu);
}

static void csr_mstatush_write(cpu_t *hart, uxlen_t v) {
    // 高 32 位 WARL: 仅 MDT (mstatush bit 10, Smdbltrp 扩展) 真实装; 其他字段
    // (SBE bit 4 / MBE bit 5 / GVA / MPV / MPELP / 其余 WPRI) 项目不实现, 强制 0。
    // 未来真做 H 扩展或大端时按合法字段扩 mask。
    uint32_t v_high = v & MSTATUSH_MDT;

    // Smdbltrp §3.1.6.2: "When the MDT bit is set to 1 by an explicit CSR write,
    // the MIE bit is cleared to 0. ... this clearing occurs regardless of the value
    // written, if any, to the MIE bit by the same write" (RV64 视角同写; RV32 跨
    // csr 写, 这里在 csrw mstatush 写 MDT=1 时强制清 _mstatus 低 32 位 MIE bit)。
    if (v_high & MSTATUSH_MDT) {
        hart->trap._mstatus &= ~(uint64_t)MSTATUS_MIE;
    }

    hart->trap._mstatus = (hart->trap._mstatus & 0xFFFFFFFFu)
                        | ((uint64_t)v_high << 32);
}

// ---- mtvec / mepc / mcause / mtval (映射到 hart->trap.{xtvec,xepc,xcause,xtval}[PRIV_M]) ----

static uxlen_t csr_mtvec_read(cpu_t *hart) {
    return hart->trap.xtvec[PRIV_M];
}

static void csr_mtvec_write(cpu_t *hart, uxlen_t v) {
    // WARL MODE 位处理 (项目支持 Direct + Vectored 两种 mode):
    //   - mtvec[1:0] = MODE:
    //       00 = Direct   (sync exception + async interrupt 都跳 BASE; 项目支持)
    //       01 = Vectored (sync exception 跳 BASE; async interrupt 跳 BASE+4*cause; 项目支持)
    //       10/11 = Reserved by RV Priv Spec
    //   - mtvec[31:2] = BASE (IALIGN 对齐, IALIGN=16 时 bit 1 不强制, IALIGN=32 时 bit[1] 在 BASE 里)
    //   RV spec §3.1.7 WARL: 实现可拒绝非法 MODE, 但必须接受 0 + 1; 我们 reserved
    //   值落 Direct (合法值).
    //
    // 跟 trap_set_*_state 的分工:
    //   trap_set_exception_state: 永走 BASE (& ~0x3u 在自己函数里 mask, 忽略 MODE)
    //   trap_set_interrupt_state: 按 MODE 决定 BASE vs BASE+4*cause_low
    uint32_t mode = v & 0x3u;
    if (mode >= 2u) mode = 0u;             /* reserved → direct (legal value) */
    hart->trap.xtvec[PRIV_M] = (v & ~0x3u) | mode;
}

static uxlen_t csr_mepc_read(cpu_t *hart) {
    return hart->trap.xepc[PRIV_M];
}

static void csr_mepc_write(cpu_t *hart, uxlen_t v) {
    // WARL 截断 IALIGN 对齐位:
    //   - IALIGN=16 (项目当前): mepc[0] = 0 (低 1 位强制 0)
    //   - IALIGN=32: mepc[1:0] = 0 (低 2 位强制 0)
    // config.h IALIGN_MASK = IALIGN_BYTES - 1 (= 1 当 IALIGN=16; = 3 当 IALIGN=32),
    // & ~IALIGN_MASK 即"截断到 IALIGN 对齐"。
    // RV spec §3.1.15 mepc: "the low bit of mepc (mepc[0]) is always zero. ... If an
    // implementation supports only IALIGN=32, then the two low bits (mepc[1:0]) are always
    // zero." 我们按 config.h 编译期 IALIGN 配置自动适配。
    hart->trap.xepc[PRIV_M] = v & ~IALIGN_MASK;
}

static uxlen_t csr_mcause_read(cpu_t *hart) {
    return hart->trap.xcause[PRIV_M];
}

static void csr_mcause_write(cpu_t *hart, uxlen_t v) {
    // mcause 字段: bit[31] = Interrupt (1) vs Exception (0); bit[30:0] = Exception/Interrupt code。
    // RV spec §3.1.16 没强制 WARL (除了 MSB; "implementations may further restrict"), 我们当前
    // 接受全 32 位写入。fixture 一般也不直接写 mcause (handler 只读, trap_set_*_state 写)。
    hart->trap.xcause[PRIV_M] = v;
}

static uxlen_t csr_mtval_read(cpu_t *hart) {
    return hart->trap.xtval[PRIV_M];
}

static void csr_mtval_write(cpu_t *hart, uxlen_t v) {
    // mtval RV spec §3.1.17 没强制 WARL, 接受任意值。
    hart->trap.xtval[PRIV_M] = v;
}

/* mtval2 (0x34B, Ssdbltrp/H 扩展): S-trap unexpected 升级 M 时, trap.c 把原本要写
 * stval 的 tval 写到 mtval2, 让 M-handler 拿到 root-cause 信息 (spec §4.1.1.5)。
 * 项目无 H 扩展, mtval2 不复用为 GPA。RV spec 无特殊 WARL, 接受任意值 */
static uxlen_t csr_mtval2_read(cpu_t *hart) {
    return hart->trap.mtval2;
}

static void csr_mtval2_write(cpu_t *hart, uxlen_t v) {
    hart->trap.mtval2 = v;
}

// ---- mscratch (xscratch[PRIV_M], 类 3) ----

static uxlen_t csr_mscratch_read(cpu_t *hart) {
    return hart->trap.xscratch[PRIV_M];
}

static void csr_mscratch_write(cpu_t *hart, uxlen_t v) {
    /* RV spec §3.1.18 mscratch RW, 任意值, 无 WARL */
    hart->trap.xscratch[PRIV_M] = v;
}

// ---- medeleg / mideleg (medeleg 拆访问类 1, mideleg 单字段类 3) ----
//
// medeleg (0x302) + medelegh (0x312): 类 1 (_medeleg uint64_t 拆访问)。M-mode 同步
//   异常 trap delegation bitmask, per-cause bit (bit N = cause N delegate 到 S-mode);
//   bit 11 (ecall_from_M) WARL hardwire 0 — M can't delegate to less-privileged
//   (SiFive U74-MC 同此); 其他位接受全 32 位写。priv spec 1.12 定义 medelegh 为
//   RV32 高 32 位入口 (cause 32-63), 跟 mstatus / mstatush 同体例拆访问。
//   trap_set_exception_state 按 _medeleg.bit(cause) 真生效 (U/S-mode trap + bit=1 → deliver S)。
// mideleg (0x303): 类 3 (mideleg uint32_t 单字段)。M-mode 中断 delegation, MXLEN-bit
//   (spec 未定义 midelegh, 中断 cause 不会超 32 位)。trap_set_interrupt_state 按
//   mideleg.bit(cause_low) 派发; trap_check_interrupt 也用 mideleg 算 deliver_mask。

static uxlen_t csr_medeleg_read(cpu_t *hart) {
    return (uint32_t)(hart->trap._medeleg & 0xFFFFFFFFu);
}

static void csr_medeleg_write(cpu_t *hart, uxlen_t v) {
    /* WARL: bit 11 (ecall_from_M) hardwire 0 — M 不能 delegate ecall_from_M 给 S */
    v &= ~(1u << CAUSE_ECALL_FROM_M);
    /* 低 32 位换成 v, 高 32 位 (medelegh 入口) 保留 — 跟 _mstatus 拆访问同形态 */
    hart->trap._medeleg = (hart->trap._medeleg & 0xFFFFFFFF00000000ULL) | (uint64_t)v;
}

/* medelegh (0x312): _medeleg 高 32 位 (cause 32-63 delegation bitmap)。RV spec
 * priv 1.12 不强制特殊 WARL — ecall_from_M bit 11 在低 32 位, 跟本入口无关; 接受
 * 全 32 位写入。跟 mstatush 平行 (拆访问 future-proof) */
static uxlen_t csr_medelegh_read(cpu_t *hart) {
    return (uint32_t)((hart->trap._medeleg >> 32) & 0xFFFFFFFFu);
}

static void csr_medelegh_write(cpu_t *hart, uxlen_t v) {
    hart->trap._medeleg = (hart->trap._medeleg & 0xFFFFFFFFu)
                        | ((uint64_t)v << 32);
}

static uxlen_t csr_mideleg_read(cpu_t *hart) {
    return hart->trap.mideleg;
}

static void csr_mideleg_write(cpu_t *hart, uxlen_t v) {
    /* mideleg WARL 项目当前简化 — 接受全 32 位写 (中断机制未实现, 字段不真用)。
     * 中断机制真做时按 RV spec 加 mask reserved bits + per-bit WARL */
    hart->trap.mideleg = v;
}

// ---- mie (类 (2a), trap._mie 物理字段; sie 是其 mask view, 见下方 S-mode 段) ----
//
// mie 入口物理字段直接读写; WARL 截 MIE_VALID_MASK 强制 reserved bits 为 0
// (跟 mstatus_write WARL 风格一致). 项目 mie 6 有效位 (IRQ_S/M × SOFT/TIMER/EXT,
// bit 1/3/5/7/9/11), 其他位 reserved 永远 0.

static uxlen_t csr_mie_read(cpu_t *hart) {
    return hart->trap._mie;
}

static void csr_mie_write(cpu_t *hart, uxlen_t v) {
    /* WARL: 截 reserved bits (高 20 位 + 偶数低位永远 0) */
    hart->trap._mie = v & MIE_VALID_MASK;
}

// ---- mip (类 (5), 软件可写子集 _mip_sw + 异步源 OR 合成读) ----
//
// csr_mip_read 合成 6 bit (跟 dummy.txt §6 第 5 类 helper 模式示例段一致):
//   bit 1/5/9 (SSIP/STIP/SEIP_sw) ← trap._mip_sw 软件 inject 字段
//   bit 3     MSIP ← CLINT.msip[hartid] 异步源 (is_clint_msip_pending)
//   bit 7     MTIP ← (mtime ≥ mtimecmp[hartid]) 派生 (is_clint_timer_pending)
//   bit 9     SEIP_hw ← PLIC s_pending OR _mip_sw bit 9 (未来; v1 hw_seip 永远 0)
//   bit 11    MEIP ← PLIC m_pending (未来; v1 永远 0)
//
// csr_mip_write 只动 _mip_sw 的 MIP_SW_WRITABLE_MASK 对应位 (SSIP/STIP/SEIP_sw);
// 其他位 (MSIP/MTIP/MEIP/reserved) 写忽略 (RO from M-mode csrw 视角).

/* csr_mip_read 非 static (跨模块 export): trap_check_interrupt 也调本函数算合成 mip view.
 * csr.h 加声明. */
uxlen_t csr_mip_read(cpu_t *hart) {
    uxlen_t  mip_view = hart->trap._mip_sw;
    uint32_t hartid   = hart->hartid;                  /* dual storage: cpu_t.hartid 是
                                                          uint32_t (index 用), 直接喂下游
                                                          is_clint_*_pending / is_plic_*_pending
                                                          uint32_t 入参. CSR 镜像走另字段
                                                          per_hart_info.mhartid (csr_mhartid_read). */

    if (is_clint_msip_pending(hartid))
        mip_view |= (1U << IRQ_M_SOFT);
    if (is_clint_timer_pending(hartid))
        mip_view |= (1U << IRQ_M_TIMER);

    /* PLIC 合成: MEIP / SEIP 位 OR (SEIP 跟 _mip_sw 已经 OR 的 sw_seip 共同合成视图,
     * 跟 RV spec §3.1.9 SEIP "hardware OR software" 语义一致). */
    if (is_plic_meip_pending(hartid))
        mip_view |= (1U << IRQ_M_EXT);
    if (is_plic_seip_pending(hartid))
        mip_view |= (1U << IRQ_S_EXT);

    return mip_view;
}

static void csr_mip_write(cpu_t *hart, uxlen_t v) {
    /* 只动 _mip_sw 软件可写位 (SSIP/STIP/SEIP_sw); RO 位 (MSIP/MTIP/MEIP) 忽略.
     * read-modify-write 单 hart 单线程 (本字段不跨 hart 写, 见 trap.h _mip_sw
     * 顶段并发注释). */
    hart->trap._mip_sw = (hart->trap._mip_sw & ~MIP_SW_WRITABLE_MASK)
                       | (v & MIP_SW_WRITABLE_MASK);
}

// ============================================================================
// Address Translation CSR (类 2)
//
// 跨模块 CSR — 字段 hart->satp 在 cpu_t (跨 mmu / dispatcher / sfence 模块用); 函数
// 留 csr.c 集中 (跟 RV spec "所有 CSR addr 走 csr_op dispatch" 一致); sfence 跟 satp
// 写是运行期协议 (dummy.txt §3, 不是代码组织耦合)。
// ============================================================================

// ---- satp (dummy.txt §3 satp 合法性契约的"生产者"职责) ----
//
// satp 物理存储在 hart->satp (cpu_t 顶层字段, 不在 trap_csrs_t 内 — satp 不属于
// trap-related CSR 范畴, 设计意图见 cpu.h)。

static uxlen_t csr_satp_read(cpu_t *hart) {
    return hart->satp;
}

static void csr_satp_write(cpu_t *hart, uxlen_t v) {
    // satp 字段 (RV32 Sv32; RV Privileged Spec Vol II §4.1.11 fig 4.11):
    //   bit  31     = MODE  (1-bit; 0 = Bare 恒等, 1 = Sv32)
    //   bits 30:22 = ASID  (Sv32 spec 9 位; 项目 ASIDLEN = TLB_ASID_BITS = 4 位)
    //   bits 21:0  = PPN   (root page table physical page number; root PA = PPN << 12)
    //
    // WARL 截断 (dummy.txt §3):
    //   - ASID 截到 ASID_MASK 位 (= 0xF, 4 位); dispatcher 直接按 cpu->satp.ASID 索引
    //     tlb_table[priv][asid] 无 bounds check, 本截断是 host 内存安全的根防线。
    //   - MODE 是 RV32 单 bit, 0/1 都合法, 不需截 (RV64 4-bit MODE 才需要落到合法值)。
    //   - PPN 不截; RV spec 不强制 WARL, 非法 PPN 由 walker 访问时按权限/范围检查触发 fault。
    //
    // satp 写不自动 sfence.vma (RV spec + plan §1.8 + dummy.txt §3): guest 软件必须显式
    // sfence.vma 才让新 ASID/MODE 在 TLB 生效。本 helper 只更新 hart->satp 字段, 不动 TLB;
    // dispatcher 下次 block 入口算 (regime, current_tlb) 时按新 satp 选叶 TLB, 但旧 ASID 槽
    // 的 entries 不被 invalidate (RV spec 设计 — 切回原 ASID 时缓存仍可用)。
    //
    // 未来 mstatus.TVM 检查:
    //   RV spec §3.1.6.5: TVM=1 + S-mode 写 satp → trap cause 2 (illegal). 当前不实现,
    //   真做 OS 隔离时在本 helper 入口或 csr_op 入口判段加。

    uint32_t mode = (v >> 31) & 0x1u;
    uint32_t asid = ((v >> 22) & 0x1FFu) & ASID_MASK;     /* WARL 截 ASID 到 ASID_MASK 位 */
    uint32_t ppn  = v & 0x3FFFFFu;                         /* PPN 22 位, 不截 */

    hart->satp = (mode << 31) | (asid << 22) | ppn;
}


// ============================================================================
// S-mode CSR (类 3 投影)
//
// 含 sstatus / sepc / sscratch / stvec / scause / stval 共 6 个 csr。sstatus 是 mstatus
// 的 masked view (SSTATUS_MASK; 物理共用 trap._mstatus); 其余 sxxx 是 trap.xxxx[PRIV_S]
// 槽 (priv-indexed 数组 [PRIV_S] 项, 跟 mxxx[PRIV_M] 对称)。
// ============================================================================

// ---- sstatus / sepc (sret 必带的最小集) ----
//
// sstatus 是 _mstatus 的 masked view (SSTATUS_MASK = SIE | SPIE | SPP | SUM | MXR; 详见
// riscv.h SSTATUS_MASK 段); 物理存储跟 mstatus 共用 trap._mstatus, csr_sstatus_read/write
// 通过 mask 操作只访问 sstatus 视图字段位, mstatus M-mode-only 字段 (MIE/MPIE/MPP) 不被影响。
//
// sepc 物理存储 = trap.xepc[PRIV_S] (按 priv 索引数组的 [PRIV_S] 槽, 跟 mepc=xepc[PRIV_M]
// 同形态)。WARL 截 IALIGN 对齐位 (跟 mepc 同)。

static uxlen_t csr_sstatus_read(cpu_t *hart) {
    /* 取 _mstatus 低 32 位 (mstatus 视图) ∩ SSTATUS_MASK = sstatus 视图 */
    return (uint32_t)(hart->trap._mstatus & 0xFFFFFFFFu) & SSTATUS_MASK;
}

static void csr_sstatus_write(cpu_t *hart, uxlen_t v) {
    /* 只改 sstatus mask 内的位; 保留 _mstatus 其余字段 (M-mode-only + 高 32 位)。
     * Ssdbltrp §4.1.1.5: "When the SDT bit is set to 1 by an explicit CSR write,
     * the SIE bit is cleared to 0. This clearing occurs regardless of the value
     * written, if any, to the SIE bit by the same write. The SIE bit can only be
     * set to 1 by an explicit CSR write if the SDT bit is being set to 0 by the
     * same write or is already 0."
     * sstatus 入口可同写 SDT + SIE, 实装: 计算写入的 v_masked (mask 内字段); 若
     * v_masked 内 SDT=1 (即写后 SDT 必然 1), 强制清 SIE; 等价于 "SDT=0 时 SIE 随便,
     * SDT=1 时 SIE 必清". */
    uint32_t v_masked = (uint32_t)v & SSTATUS_MASK;
    if (v_masked & MSTATUS_SDT) {
        v_masked &= ~MSTATUS_SIE;
    }
    const u64_t keep = hart->trap._mstatus & ~(uint64_t)SSTATUS_MASK;
    const u64_t set  = (uint64_t)v_masked;
    hart->trap._mstatus = keep | set;
}

static uxlen_t csr_sepc_read(cpu_t *hart) {
    return hart->trap.xepc[PRIV_S];
}

static void csr_sepc_write(cpu_t *hart, uxlen_t v) {
    /* WARL 截 IALIGN 对齐位 (跟 mepc 同; RV spec §3.1.15 / sepc 同样要求) */
    hart->trap.xepc[PRIV_S] = v & ~IALIGN_MASK;
}

// ---- sscratch / stvec / scause / stval (xxx[PRIV_S], 类 3) ----
//
// 跟 mscratch / mtvec / mcause / mtval 同形态, 只是 [PRIV_S] 槽。stvec WARL 处理 MODE 跟
// mtvec 同 (Direct + Vectored 都支持, 2/3 → 0)。trap_set_exception_state 按 medeleg /
// trap_set_interrupt_state 按 mideleg 派发 deliver_priv = S 时写 [PRIV_S] 槽。

static uxlen_t csr_sscratch_read(cpu_t *hart) {
    return hart->trap.xscratch[PRIV_S];
}

static void csr_sscratch_write(cpu_t *hart, uxlen_t v) {
    hart->trap.xscratch[PRIV_S] = v;
}

static uxlen_t csr_stvec_read(cpu_t *hart) {
    return hart->trap.xtvec[PRIV_S];
}

static void csr_stvec_write(cpu_t *hart, uxlen_t v) {
    /* WARL MODE 处理跟 mtvec 同 (Direct + Vectored 都支持, 2/3 reserved → 0; 详 csr_mtvec_write) */
    uint32_t mode = v & 0x3u;
    if (mode >= 2u) mode = 0u;
    hart->trap.xtvec[PRIV_S] = (v & ~0x3u) | mode;
}

static uxlen_t csr_scause_read(cpu_t *hart) {
    return hart->trap.xcause[PRIV_S];
}

static void csr_scause_write(cpu_t *hart, uxlen_t v) {
    /* RV spec §5.1.6 scause; 跟 mcause 同, 接受全 32 位 */
    hart->trap.xcause[PRIV_S] = v;
}

static uxlen_t csr_stval_read(cpu_t *hart) {
    return hart->trap.xtval[PRIV_S];
}

static void csr_stval_write(cpu_t *hart, uxlen_t v) {
    /* RV spec §5.1.7 stval; 跟 mtval 同, 接受全 32 位 */
    hart->trap.xtval[PRIV_S] = v;
}

// ---- sie / sip (mie/mip readout 的 mask view; 类 (2a) + 类 (5) 各自) ----
//
// sie 是 _mie 的 mask view (类 (2a), 跟 sstatus 是 _mstatus 的 mask view 同形态);
// sip 是 csr_mip_read 合成的 mask view + S 写位收紧 (类 (5) 的 S-mode 投影).
// sip RW 位仅 SSIP (SIP_WRITABLE_MASK = 1 << IRQ_S_SOFT); STIP / SEIP RO in sip
// (RV spec §5.1.4 + §3.1.9 强制 — S-mode 不直接清 STIP/SEIP, 清靠 PLIC complete
// 或 M-mode csrw mip).

static uxlen_t csr_sie_read(cpu_t *hart) {
    return hart->trap._mie & SIE_MASK;
}

static void csr_sie_write(cpu_t *hart, uxlen_t v) {
    /* 只动 SIE_MASK 子集; 保留 M-mode 中断使能位 */
    hart->trap._mie = (hart->trap._mie & ~SIE_MASK) | (v & SIE_MASK);
}

static uxlen_t csr_sip_read(cpu_t *hart) {
    /* mip readout 的 mask view (合成走 csr_mip_read) */
    return csr_mip_read(hart) & SIP_MASK;
}

static void csr_sip_write(cpu_t *hart, uxlen_t v) {
    /* S-mode 只可写 SSIP; STIP / SEIP RO in sip */
    hart->trap._mip_sw = (hart->trap._mip_sw & ~SIP_WRITABLE_MASK)
                       | (v & SIP_WRITABLE_MASK);
}


// ============================================================================
// M-mode RO Identity CSR (类 4) — 拆 per-hart + shared 两组
//
// 4a per-hart 私有 (mhartid + misa): 数据在 hart->per_hart_info.{mhartid, misa} (嵌入,
//    cpu_create 入参写入); 异构 SMP 时不同 hart 字段值不同。dual storage: mhartid
//    同时窄化写 cpu_t.hartid (uint32_t) 服务 index 用, csr 读仍走本 per_hart_info 字段。
// 4b 多 hart 共享 (mvendorid + marchid + mimpid): 数据在 hart->shared_info->xxx 解引用
//    (指针, cpu.c static const cpu_info_shared_default; 机器整体属性, 不区分 hart)。
//
// 写检查:
//   mhartid/mvendorid/marchid/mimpid 的 csr addr [11:10]=11 = RO; csr_op 入口判已 reject 写
//   (csrw 触发 cause 2 illegal); 不需要 write helper, 大 switch 写路径不加 case。
//   misa addr 0x301 [11:10]=00 = RW; 但 RV spec §3.1.1 WARL 允许实现 hardwire 全部扩展位 →
//   write helper noop (接受写但不真改); 大 switch 写路径加 case 调 noop helper。
// ============================================================================

// ---- 4a: mhartid + misa (嵌入 hart->per_hart_info; mhartid 同时窄化镜像到 cpu_t.hartid 顶层) ----

static uxlen_t csr_mhartid_read(cpu_t *hart) {
    return hart->per_hart_info.mhartid;     /* CSR 镜像直读 (dual storage; cpu_t.hartid 是 index
                                               用的 uint32_t 镜像, 这里走 per_hart_info uxlen_t) */
}

static uxlen_t csr_misa_read(cpu_t *hart) {
    return hart->per_hart_info.misa;
}

static void csr_misa_write(cpu_t *hart, uxlen_t v) {
    /* RV spec §3.1.1 WARL: 实现可 hardwire 不支持的扩展位忽略写入。项目所有扩展位
     * hardwire (per_hart_info.misa fixed by cpu_create 入参), 写入忽略 — read 仍返
     * hart->per_hart_info.misa。真要按 misa 切扩展行为时改这里 (累加可写位 mask)。 */
    (void)hart;
    (void)v;
}

// ---- 4b: mvendorid + marchid + mimpid (多 hart 共享, hart->shared_info 解引用) ----

static uxlen_t csr_mvendorid_read(cpu_t *hart) {
    return hart->shared_info->mvendorid;
}

static uxlen_t csr_marchid_read(cpu_t *hart) {
    return hart->shared_info->marchid;
}

static uxlen_t csr_mimpid_read(cpu_t *hart) {
    return hart->shared_info->mimpid;
}


// ============================================================================
// Unprivileged Counter/Timer CSR (类 5 派生合成读; RV Unpriv Spec Ch 10)
//
// 含 time (0xC01) + timeh (0xC81), 都是 clint.mtime 的副本 view:
//   - time  返 mtime 低 32 位
//   - timeh 返 mtime 高 32 位 (RV32 only; RV64 切换时直接删 timeh case + read helper)
//
// 接口拿 clint_read_mtime() (完整 u64), csr.c 截 lo/hi — 单 mtime read 跨 lo/hi 截
// 不会跨 atomic 边界 (lo/hi 来自同一 atomic_load), 不存在"半字段 stale" race。但
// software 跨两次 csrr time/timeh 间 mtime 涨可能高低不一致 (RV spec 已知 race);
// 标准协议 "read timeh / read time / read timeh again / 比对" 由 software 解决,
// 项目不在 CSR 层补救。
//
// TODO: mcounteren / scounteren 未实装 — U-mode 访问 time 在 spec 受 mcounteren.TM
// (bit 1) 控制, S-mode 受 scounteren.TM 控制; v1 简化让 U/S/M 三 priv 都可访问
// (跟 OpenSBI / Linux default mcounteren=0xFFFFFFFF 配置等效, 不违 spec)。真需要
// 严控时改本段 helper 入口加 mcounteren bit check + cause 2 illegal trap。
// ============================================================================

static uxlen_t csr_time_read(cpu_t *hart) {
    (void)hart;
    return (uint32_t)(clint_read_mtime() & 0xFFFFFFFFu);
}

static uxlen_t csr_timeh_read(cpu_t *hart) {
    (void)hart;
    return (uint32_t)((clint_read_mtime() >> 32) & 0xFFFFFFFFu);
}


// ============================================================================
// privrd (项目自定义 CSR 0xCC0) — silent 返当前 priv backdoor
//
// 用途: fixture / demo (mini-shell cmd_csr / cmd_su) 通过 csrr 0xCC0 读当前 priv
// (M=3 / S=1 / U=0; H=2 项目 reserved 不用). RV spec 不允许 User-mode 直接知道
// priv, 0xCC0 是项目 backdoor.
//
// 行为: silent 返 (uxlen_t)hart->priv. 不打 stderr (早期临时设计是 fprintf "[priv] X"
// 给 console 看, 但 uart 实装 + demo 高频读 0xCC0 频繁刷 stderr → 改 silent;
// fixture 用 GPR 读 priv 值仍 work).
//
// RO csr — csr_op 入口判 [11:10]=0b11 时 trap 写; 没 write helper.
// ============================================================================

static uxlen_t csr_privrd_read(cpu_t *hart) {
    return (uxlen_t)hart->priv;
}


// ============================================================================
// csr_op —— 大 helper, decode 分发入口
// ============================================================================

uxlen_t csr_op(cpu_t *hart, uint32_t csr_addr, uxlen_t new_val,
               csr_op_t op, u32_t raw_inst) {
    // ----------------------------------------------------------------------------
    // 入口判: priv 要求 + RO 写检查 (csr 编号自带权限位段, riscv.h CSR_ADDR_PRIV_*)
    //
    //   - csr_addr[9:8] = 最低 priv 要求 (RV Privileged Spec §2.1):
    //       00=U, 01=S, 10=H/VS, 11=M
    //     hart->priv < required_priv → cause 2 illegal instruction trap
    //     注意: PRIV_M=3 是最高, 数字最大; 比较用 `<` 即可
    //   - csr_addr[11:10] = RO 标识: 0b11 表 RO; 写 RO csr → cause 2
    //     "写"判断: CSR_OP_RW 永远算写; CSR_OP_RS/RC + new_val=0 不写 (RV spec §2.1.2 副作用
    //     不发生 — 跟下方 do_write 标志一致)
    //
    // tval = raw_inst (RV spec §3.1.16: illegal instruction trap 的 mtval = 触发指令编码)
    // ----------------------------------------------------------------------------
    {
        uint32_t required_priv = (csr_addr >> CSR_ADDR_PRIV_SHIFT) & CSR_ADDR_PRIV_MASK;
        if (hart->priv < required_priv) {
            trap_raise_exception(hart, CAUSE_ILLEGAL_INSTRUCTION, raw_inst);  // _Noreturn longjmp
        }

        uint32_t is_ro = ((csr_addr >> CSR_ADDR_RO_SHIFT) & CSR_ADDR_RO_MASK)
                         == CSR_ADDR_RO_VALUE;
        uint32_t is_write = (op == CSR_OP_RW) || (new_val != 0);
        if (is_ro && is_write) {
            trap_raise_exception(hart, CAUSE_ILLEGAL_INSTRUCTION, raw_inst);  // _Noreturn longjmp
        }
    }

    // ----------------------------------------------------------------------------
    // 大 switch: 按 csr_addr 分发到具体 csr 的小 r/w helper, 算 read_old + write_back。
    //
    // 加新 csr 时只在这里加 case + 写一对 csr_<name>_<r/w> file-static helper (上方)。
    // 不存在的 csr addr 走 default: fprintf 提示 + trap_raise_exception 真路径 (跟 lsu.h/c
    // BARE 不在 RAM 路径同风格 — fprintf 留下 dev-friendly 定位信息, trap 走 RV spec §2.1
    // "访问未实现 csr → illegal instruction" 路径)。
    // ----------------------------------------------------------------------------
    uxlen_t read_old;
    switch (csr_addr) {
        case CSR_MSTATUS:  read_old = csr_mstatus_read (hart); break;
        case CSR_MSTATUSH: read_old = csr_mstatush_read(hart); break;
        case CSR_MTVEC:    read_old = csr_mtvec_read   (hart); break;
        case CSR_MEPC:     read_old = csr_mepc_read    (hart); break;
        case CSR_MCAUSE:   read_old = csr_mcause_read  (hart); break;
        case CSR_MTVAL:    read_old = csr_mtval_read   (hart); break;
        case CSR_MTVAL2:   read_old = csr_mtval2_read  (hart); break;
        case CSR_MSCRATCH: read_old = csr_mscratch_read(hart); break;
        case CSR_MEDELEG:  read_old = csr_medeleg_read (hart); break;
        case CSR_MEDELEGH: read_old = csr_medelegh_read(hart); break;
        case CSR_MIDELEG:  read_old = csr_mideleg_read (hart); break;
        case CSR_MIE:      read_old = csr_mie_read     (hart); break;
        case CSR_MIP:      read_old = csr_mip_read     (hart); break;
        case CSR_SATP:     read_old = csr_satp_read    (hart); break;
        case CSR_SSTATUS:  read_old = csr_sstatus_read (hart); break;
        case CSR_SEPC:     read_old = csr_sepc_read    (hart); break;
        case CSR_SSCRATCH: read_old = csr_sscratch_read(hart); break;
        case CSR_STVEC:    read_old = csr_stvec_read   (hart); break;
        case CSR_SCAUSE:   read_old = csr_scause_read  (hart); break;
        case CSR_STVAL:    read_old = csr_stval_read   (hart); break;
        case CSR_SIE:      read_old = csr_sie_read     (hart); break;
        case CSR_SIP:      read_old = csr_sip_read     (hart); break;
        case CSR_MHARTID:  read_old = csr_mhartid_read (hart); break;   /* RO */
        case CSR_MISA:     read_old = csr_misa_read    (hart); break;   /* RW-effective-RO */
        case CSR_MVENDORID:read_old = csr_mvendorid_read(hart); break;  /* RO */
        case CSR_MARCHID:  read_old = csr_marchid_read (hart); break;   /* RO */
        case CSR_MIMPID:   read_old = csr_mimpid_read  (hart); break;   /* RO */
        case CSR_TIME:     read_old = csr_time_read    (hart); break;   /* RO; clint.mtime low  */
        case CSR_TIMEH:    read_old = csr_timeh_read   (hart); break;   /* RO; clint.mtime high (RV32) */
        case CSR_PRIVRD:   read_old = csr_privrd_read  (hart); break;   /* 临时 RO; RO 写 trap 由入口判 */
        default:
            fprintf(stderr,
                    "[csr] unknown csr addr=0x%03" PRIx32 " → trap cause 2" EOL,
                    csr_addr);
            trap_raise_exception(hart, CAUSE_ILLEGAL_INSTRUCTION, raw_inst);  // _Noreturn longjmp
    }

    // 算 new + write_back; switch on csr_op_t (3 case 全覆盖, -Wswitch-enum 强制)。
    // default 走 __builtin_unreachable: op 必是 3 个 csr_op_t 之一 (decode 保证), 这句把
    // 不变式断言给优化器 — 消 -O2 下 to_write "may be used uninitialized" 误报 (switch-
    // on-enum 无 default 时编译器证不出 to_write 必被赋值; 内联 csr_*_write 后暴露)。加
    // default 不影响 -Wswitch-enum: 它查枚举值有无显式 case, 跟 default 无关, 新增
    // csr_op_t 值照样编译报错。
    uxlen_t to_write;
    switch (op) {
        case CSR_OP_RW: to_write = new_val;             break;
        case CSR_OP_RS: to_write = read_old |  new_val; break;
        case CSR_OP_RC: to_write = read_old & ~new_val; break;
        default: __builtin_unreachable();
    }

    // RV spec §2.1.2: RS/RC + new_val=0 不写 (副作用不发生)。RW 总是写。
    // 这条同时为 RO csr 提供保护 (RS/RC + 0 不触发"写 RO" trap; 当然入口判已经放过了)。
    int do_write = (op == CSR_OP_RW) || (new_val != 0);
    if (do_write) {
        switch (csr_addr) {
            case CSR_MSTATUS:  csr_mstatus_write (hart, to_write); break;
            case CSR_MSTATUSH: csr_mstatush_write(hart, to_write); break;
            case CSR_MTVEC:    csr_mtvec_write   (hart, to_write); break;
            case CSR_MEPC:     csr_mepc_write    (hart, to_write); break;
            case CSR_MCAUSE:   csr_mcause_write  (hart, to_write); break;
            case CSR_MTVAL:    csr_mtval_write   (hart, to_write); break;
            case CSR_MTVAL2:   csr_mtval2_write  (hart, to_write); break;
            case CSR_MSCRATCH: csr_mscratch_write(hart, to_write); break;
            case CSR_MEDELEG:  csr_medeleg_write (hart, to_write); break;
            case CSR_MEDELEGH: csr_medelegh_write(hart, to_write); break;
            case CSR_MIDELEG:  csr_mideleg_write (hart, to_write); break;
            case CSR_MIE:      csr_mie_write     (hart, to_write); break;
            case CSR_MIP:      csr_mip_write     (hart, to_write); break;
            case CSR_SATP:     csr_satp_write    (hart, to_write); break;
            case CSR_SSTATUS:  csr_sstatus_write (hart, to_write); break;
            case CSR_SEPC:     csr_sepc_write    (hart, to_write); break;
            case CSR_SSCRATCH: csr_sscratch_write(hart, to_write); break;
            case CSR_STVEC:    csr_stvec_write   (hart, to_write); break;
            case CSR_SCAUSE:   csr_scause_write  (hart, to_write); break;
            case CSR_STVAL:    csr_stval_write   (hart, to_write); break;
            case CSR_SIE:      csr_sie_write     (hart, to_write); break;
            case CSR_SIP:      csr_sip_write     (hart, to_write); break;
            case CSR_MISA:     csr_misa_write    (hart, to_write); break;   /* noop (WARL) */
            default:
                // 上面 read 路径的 default 已 fprintf + trap_raise_exception
                // (_Noreturn longjmp), 控制流到此不可达; 保留 default break 作 -Wswitch
                // 默认防御 (switch on uint32_t 不受 -Wswitch-enum 约束, 但写出来更稳)。
                break;
        }
    }

    return read_old;
}
