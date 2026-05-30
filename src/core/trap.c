//
// Created by liujilan on 2026/5/4.
// trap 模块实现 (架构语义层 + exception/interrupt 双拆 set_state + raise/check 入口接通
// sigsetjmp 协议 + 真切 priv mode + 写 mstatus/sstatus 按 deliver_priv 分流)。
//
// 顶部接口 doc 见 trap.h; 跨文件协议见 src/dummy.txt §1 (路径 2a/2b/2b' 三分)。
//
// 函数分工:
//   trap_set_exception_state  — sync exception 路径 (medeleg / cause < 32 / tval caller 传)
//   trap_set_interrupt_state  — async interrupt 路径 (mideleg / cause_low | high bit / tval=0 / vectored mode)
//   trap_check_interrupt      — dispatcher 主帧 polling 入口 (合成读 mip + mie + 全局 IE + 优先级 + 调 set_interrupt_state)
//   trap_raise_exception      — interpreter/JIT 深栈 _Noreturn longjmp (内含 set_exception_state + siglongjmp)
//

#include "trap.h"

#include "cpu.h"        // cpu_t 完整定义 (trap.h 只 forward, 这里要访问字段)
#include "csr.h"        // csr_mip_read (trap_check_interrupt 合成读 mip)
#include "debug.h"      // DEBUG_EXCEPTION / DEBUG_INT_CHECK / DEBUG_TIME/SOFT/EXT_INTR
#include "riscv.h"      // PRIV_M / CAUSE_INTERRUPT_BIT / IRQ_* / MIE_VALID_MASK / SIE_MASK / MSTATUS_*
#include "runtime.h"    // system_reset_signal_set_bit (M-mode double trap critical-error)

#include <setjmp.h>     // siglongjmp
#include <stdint.h>


// ----------------------------------------------------------------------------
// trap_set_exception_state —— 架构语义层 (sync exception 路径), 不长跳
//
// spec-defined MDT/SDT 路径 (无独立 in_trap 嵌套计数字段):
//   S-trap entry: 若 sstatus.SDT=1, unexpected trap (Ssdbltrp §4.1.1.5) →
//                 升级到 M-mode, cause=CAUSE_DOUBLE_TRAP, mtval2=原 tval, tval=0
//   M-trap entry: 若 mstatus.MDT=1, unexpected trap (Smdbltrp §3.1.6.2) →
//                 critical-error state (项目无 NMI, abort): set HART_MDT, return 1
//                 不更新 architectural state (per spec)
//   正常 deliver: set 对应 xstatus.xDT=1 (MDT 在 _mstatus bit 42 / SDT 在 bit 24)
// 返值: 0 = 没操作 (当前不存在, 总会 deliver 或升级或停机); 非 0 = "已设状态,
//       dispatcher continue 重 fetch"。caller (mmu_translate_pc / dispatcher IALIGN
//       兜底 / trap_raise_exception) 一律 rc != 0 → continue 或 longjmp.
// ----------------------------------------------------------------------------
int trap_set_exception_state(cpu_t *hart, uint32_t cause, uxlen_t tval) {
    // DEBUG trace 'E'. 三条调用路径都是 sync exception:
    //   (a) trap_raise_exception 内部长跳 (interpreter / JIT helper 走 dummy.txt §1 路径 2a)
    //   (b) mmu_translate_pc fetch fault 非长跳 (dispatcher 主帧, 路径 2b)
    //   (c) dispatcher 循环顶 pc IALIGN 兜底非长跳 (cause 0, 跟 (b) 同形态; 见 §9)
    DEBUG_EXCEPTION();

    // ------------------------------------------------------------------------
    // deliver_priv 按 _medeleg 真生效 (RV Privileged Spec §3.1.8)
    //   - M-mode trap (caller priv = M): 总 deliver M
    //   - U/S-mode trap + _medeleg.bit(cause) = 1: deliver S
    //   - U/S-mode trap + _medeleg.bit(cause) = 0: deliver M
    // ------------------------------------------------------------------------
    uint8_t deliver_priv;
    if (hart->priv == PRIV_M) {
        deliver_priv = PRIV_M;
    } else if (cause < 64 && ((hart->trap._medeleg >> cause) & 1u)) {
        deliver_priv = PRIV_S;
    } else {
        deliver_priv = PRIV_M;
    }

    // ------------------------------------------------------------------------
    // Ssdbltrp §4.1.1.5: S-trap entry 检 SDT — SDT=1 时 unexpected, 升级到 M
    // cause=CAUSE_DOUBLE_TRAP (16); mtval2 = 原本要写 stval 的 tval; 新 tval=0.
    // (spec: "writes registers, except mcause and mtval2, with the same info
    //  that the unexpected trap would have written if it was taken into M-mode.
    //  The mtval2 register is then set to what would be otherwise written into
    //  the [stval]")
    // ------------------------------------------------------------------------
    if (deliver_priv == PRIV_S &&
        (hart->trap._mstatus & (uint64_t)MSTATUS_SDT) != 0u) {
        hart->trap.mtval2 = tval;
        cause = CAUSE_DOUBLE_TRAP;
        tval = 0u;
        deliver_priv = PRIV_M;
    }

    // ------------------------------------------------------------------------
    // Smdbltrp §3.1.6.2: M-trap entry 检 MDT — MDT=1 时 unexpected, hart 进
    // critical-error state. 项目无 NMI (Smrnmi 未实装), 按 QEMU 实践 abort:
    // set system_reset_signal HART_MDT, dispatcher 退出走 ABORT_MASK cleanup.
    // 不更新 architectural state (per spec "without updating any architectural
    // state, including the pc"); xcause/xtval/xepc/mstatus 保留 root-cause 状态。
    // ------------------------------------------------------------------------
    if (deliver_priv == PRIV_M &&
        (hart->trap._mstatus & MSTATUS_MDT_BIT64) != 0u) {
        system_reset_signal_set_bit(SYSRESET_BIT_HART_MDT);
        return 1;
    }

    hart->trap.xcause[deliver_priv] = cause;                  /* cause < 32, 不含 CAUSE_INTERRUPT_BIT */
    hart->trap.xtval [deliver_priv] = tval;
    hart->trap.xepc  [deliver_priv] = hart->regs[0];          /* 当前指令 PC */

    // ------------------------------------------------------------------------
    // 切 priv mode + 保存 mstatus 字段 + set xDT=1 (按 deliver_priv 分流)
    //   deliver M (§3.1.6.1): MPP=caller, MPIE=MIE, MIE=0, MDT=1
    //   deliver S (§5.1.1):   SPP=caller(1bit), SPIE=SIE, SIE=0, SDT=1
    // ------------------------------------------------------------------------
    {
        uint64_t ms = hart->trap._mstatus;
        uint32_t mstatus_lo = (uint32_t)(ms & 0xFFFFFFFFu);

        if (deliver_priv == PRIV_M) {
            mstatus_lo &= ~MSTATUS_MPP;
            mstatus_lo |= ((uint32_t)hart->priv << MSTATUS_MPP_SHIFT) & MSTATUS_MPP;
            if (mstatus_lo & MSTATUS_MIE) mstatus_lo |=  MSTATUS_MPIE;
            else                          mstatus_lo &= ~MSTATUS_MPIE;
            mstatus_lo &= ~MSTATUS_MIE;
            ms |= MSTATUS_MDT_BIT64;                          /* M-trap entry: MDT=1 */
        } else {
            /* deliver_priv == PRIV_S; caller 必为 U/S */
            if (hart->priv == PRIV_S) mstatus_lo |=  MSTATUS_SPP;
            else                      mstatus_lo &= ~MSTATUS_SPP;
            if (mstatus_lo & MSTATUS_SIE) mstatus_lo |=  MSTATUS_SPIE;
            else                          mstatus_lo &= ~MSTATUS_SPIE;
            mstatus_lo &= ~MSTATUS_SIE;
            mstatus_lo |= MSTATUS_SDT;                        /* S-trap entry: SDT=1 */
        }

        hart->trap._mstatus = (ms & 0xFFFFFFFF00000000ULL)
                            | (uint64_t)mstatus_lo;
    }

    hart->priv    = deliver_priv;
    /* Exception 永走 BASE (mode bit mask 掉) */
    hart->regs[0] = hart->trap.xtvec[deliver_priv] & ~0x3u;

    return 1;
}


// ----------------------------------------------------------------------------
// trap_set_interrupt_state —— 架构语义层 (async interrupt 路径), 不长跳
//
// 跟 trap_set_exception_state 共用 mstatus 翻转 + MDT/SDT 检查 (重复 by design;
// 5 点分歧: cause_low 加 INTERRUPT_BIT / mideleg / tval=0 / vectored / DEBUG 分流).
// MDT/SDT 路径同 exception (interrupt 也是 trap, spec 不区分).
// ----------------------------------------------------------------------------
int trap_set_interrupt_state(cpu_t *hart, uint32_t cause_low) {
    /* DEBUG trace: cause_low 分流 't/s/e'. */
    switch (cause_low) {
        case IRQ_M_TIMER: case IRQ_S_TIMER: DEBUG_TIME_INTR(); break;
        case IRQ_M_SOFT:  case IRQ_S_SOFT:  DEBUG_SOFT_INTR(); break;
        case IRQ_M_EXT:   case IRQ_S_EXT:   DEBUG_EXT_INTR();  break;
        default: break;  /* 不应到 (trap_check_interrupt 内 priority encoder 只产 6 个合法 IRQ) */
    }

    // ------------------------------------------------------------------------
    // deliver_priv 按 mideleg 真生效 (RV Privileged Spec §3.1.8 跟 medeleg 对偶)
    // ------------------------------------------------------------------------
    uint8_t deliver_priv;
    if (hart->priv == PRIV_M) {
        deliver_priv = PRIV_M;
    } else if ((hart->trap.mideleg >> cause_low) & 1u) {
        deliver_priv = PRIV_S;
    } else {
        deliver_priv = PRIV_M;
    }

    // ------------------------------------------------------------------------
    // Ssdbltrp §4.1.1.5: S-trap entry 检 SDT → unexpected 升级 M, cause=DOUBLE_TRAP.
    // interrupt 的 mtval2 也按 spec 设原本要写的 stval 值 (interrupt tval 是 0,
    // 所以 mtval2 也写 0)。
    // ------------------------------------------------------------------------
    uint32_t final_cause = cause_low;
    int      delivered_as_double = 0;
    if (deliver_priv == PRIV_S &&
        (hart->trap._mstatus & (uint64_t)MSTATUS_SDT) != 0u) {
        hart->trap.mtval2 = 0u;        /* interrupt tval=0 */
        final_cause = CAUSE_DOUBLE_TRAP;
        deliver_priv = PRIV_M;
        delivered_as_double = 1;
    }

    // ------------------------------------------------------------------------
    // Smdbltrp §3.1.6.2: M-trap entry 检 MDT → critical-error. 不更新 architectural state.
    // ------------------------------------------------------------------------
    if (deliver_priv == PRIV_M &&
        (hart->trap._mstatus & MSTATUS_MDT_BIT64) != 0u) {
        system_reset_signal_set_bit(SYSRESET_BIT_HART_MDT);
        return 1;
    }

    /* xcause: interrupt 走 cause_low | INTERRUPT_BIT; double_trap 升级路径走 sync cause (无 INT bit). */
    if (delivered_as_double) {
        hart->trap.xcause[deliver_priv] = final_cause;
    } else {
        hart->trap.xcause[deliver_priv] = final_cause | CAUSE_INTERRUPT_BIT;
    }
    hart->trap.xtval [deliver_priv] = 0u;                                /* RV spec interrupt tval=0 */
    hart->trap.xepc  [deliver_priv] = hart->regs[0];

    // ------------------------------------------------------------------------
    // 切 priv mode + mstatus 翻转 + set xDT=1
    // ------------------------------------------------------------------------
    {
        uint64_t ms = hart->trap._mstatus;
        uint32_t mstatus_lo = (uint32_t)(ms & 0xFFFFFFFFu);

        if (deliver_priv == PRIV_M) {
            mstatus_lo &= ~MSTATUS_MPP;
            mstatus_lo |= ((uint32_t)hart->priv << MSTATUS_MPP_SHIFT) & MSTATUS_MPP;
            if (mstatus_lo & MSTATUS_MIE) mstatus_lo |=  MSTATUS_MPIE;
            else                          mstatus_lo &= ~MSTATUS_MPIE;
            mstatus_lo &= ~MSTATUS_MIE;
            ms |= MSTATUS_MDT_BIT64;                          /* M-trap entry: MDT=1 */
        } else {
            if (hart->priv == PRIV_S) mstatus_lo |=  MSTATUS_SPP;
            else                      mstatus_lo &= ~MSTATUS_SPP;
            if (mstatus_lo & MSTATUS_SIE) mstatus_lo |=  MSTATUS_SPIE;
            else                          mstatus_lo &= ~MSTATUS_SPIE;
            mstatus_lo &= ~MSTATUS_SIE;
            mstatus_lo |= MSTATUS_SDT;                        /* S-trap entry: SDT=1 */
        }

        hart->trap._mstatus = (ms & 0xFFFFFFFF00000000ULL)
                            | (uint64_t)mstatus_lo;
    }

    hart->priv = deliver_priv;

    // ------------------------------------------------------------------------
    // jump-to: vectored mode 处理。注意 double_trap 升级路径走 sync exception 语义,
    // 永走 BASE (跟 exception 同), 不走 vectored.
    // ------------------------------------------------------------------------
    {
        uxlen_t  tvec = hart->trap.xtvec[deliver_priv];
        uint32_t mode = tvec & 0x3u;
        uxlen_t  base = tvec & ~0x3u;
        if (delivered_as_double) {
            hart->regs[0] = base;                                  /* exception 永走 BASE */
        } else {
            hart->regs[0] = (mode == 1u) ? (base + 4u * cause_low) : base;
        }
    }

    return 1;
}


// ----------------------------------------------------------------------------
// trap_check_interrupt —— dispatcher 主帧 polling 入口, 不长跳; 详见 trap.h doc
//
// 6 步: csr_mip_read 合成读 → mip & mie → deliver_mask (按 priv + 全局 IE + mideleg)
// → ready → 优先级 → trap_set_interrupt_state.
// trace 互斥: ready==0 时本函数打 'c'; ready!=0 时 set_interrupt_state 内打 't/s/e'.
// ----------------------------------------------------------------------------
int trap_check_interrupt(cpu_t *hart) {
    uxlen_t mip_view = csr_mip_read(hart);
    uxlen_t enabled  = mip_view & hart->trap._mie;

    /* deliver_mask: 当前 priv 下哪些 IRQ 在全局 IE / mideleg 允许下接受 deliver. */
    uxlen_t deliver_mask;
    if (hart->priv == PRIV_M) {
        uint32_t mstatus_lo = (uint32_t)(hart->trap._mstatus & 0xFFFFFFFFu);
        deliver_mask = (mstatus_lo & MSTATUS_MIE) ? MIE_VALID_MASK : 0u;
    } else if (hart->priv == PRIV_S) {
        uint32_t mstatus_lo = (uint32_t)(hart->trap._mstatus & 0xFFFFFFFFu);
        uxlen_t m_level = (~hart->trap.mideleg) & MIE_VALID_MASK;         /* M-level IRQ 始终接受 */
        uxlen_t s_level = (mstatus_lo & MSTATUS_SIE)
                          ? (hart->trap.mideleg & SIE_MASK)               /* S-level IRQ 视 SIE */
                          : 0u;
        deliver_mask = m_level | s_level;
    } else {
        /* PRIV_U: 低 priv 无法 mask 高 priv interrupt, 全部接受 */
        deliver_mask = MIE_VALID_MASK;
    }

    uxlen_t ready = enabled & deliver_mask;
    if (ready == 0u) {
        DEBUG_INT_CHECK();   /* 'c' = check but no fire (互斥协议 vs 't/s/e') */
        return 0;
    }

    /* RV Priv Spec §3.1.9 优先级: M_EXT > M_SOFT > M_TIMER > S_EXT > S_SOFT > S_TIMER */
    uint32_t irq;
    if      (ready & (1u << IRQ_M_EXT))   irq = IRQ_M_EXT;
    else if (ready & (1u << IRQ_M_SOFT))  irq = IRQ_M_SOFT;
    else if (ready & (1u << IRQ_M_TIMER)) irq = IRQ_M_TIMER;
    else if (ready & (1u << IRQ_S_EXT))   irq = IRQ_S_EXT;
    else if (ready & (1u << IRQ_S_SOFT))  irq = IRQ_S_SOFT;
    else                                  irq = IRQ_S_TIMER;

    // 透传 trap_set_interrupt_state 返值 (恒 1: deliver-OK / critical-error 同值,
    // caller dispatcher 不区分, 都 continue 让 while(SRS==0) 接管)。透传比
    // (void)cast + return 1 略直观 (表达"返值来自下层 helper"); 未来 set_*_state
    // 若引入多状态码 (e.g. 2=critical-error) 透传自然受益。
    return trap_set_interrupt_state(hart, irq);
}


// ----------------------------------------------------------------------------
// trap_raise_exception —— interpreter / JIT block 内 helper 长跳入口; 详见 trap.h doc
//
// _Noreturn, 内部 trap_set_exception_state + siglongjmp(*hart->jmp_buf_ptr, 1) 跳回
// dispatcher 入口的一次性 sigsetjmp 落点.
// ----------------------------------------------------------------------------
_Noreturn void trap_raise_exception(cpu_t *hart, uint32_t cause, uxlen_t tval) {
    (void)trap_set_exception_state(hart, cause, tval);
    siglongjmp(*hart->jmp_buf_ptr, 1);
    /* unreachable; siglongjmp 不返回. _Noreturn 标识. */
}
