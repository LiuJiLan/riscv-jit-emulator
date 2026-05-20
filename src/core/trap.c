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

#include <setjmp.h>     // siglongjmp
#include <stdint.h>


// ----------------------------------------------------------------------------
// trap_set_exception_state —— 架构语义层 (sync exception 路径), 不长跳; 详见 trap.h doc
// ----------------------------------------------------------------------------
uint8_t trap_set_exception_state(cpu_t *hart, uint32_t cause, uxlen_t tval) {
    // DEBUG trace 'E'. 三条调用路径都是 sync exception:
    //   (a) trap_raise_exception 内部长跳 (interpreter / JIT helper 走 dummy.txt §1 路径 2a)
    //   (b) mmu_translate_pc fetch fault 非长跳 (dispatcher 主帧, 路径 2b)
    //   (c) dispatcher 循环顶 pc IALIGN 兜底非长跳 (cause 0, 跟 (b) 同形态; 见 §9)
    // 在 in_trap++ 后, early return 之前打 — triple fault 那次也算 trace 发生 ('EEE' 数嵌套深度).
    DEBUG_EXCEPTION();

    hart->trap.in_trap++;

    // 候选 A: 第三次 (含) 进 set_state 早 return, 不 deliver. 字段保留第二次状态作 root cause.
    // 跟 RV Smdbltrp 扩展无关 (项目不实现; 见 trap.h doc).
    if (hart->trap.in_trap >= 3) {
        return hart->trap.in_trap;
    }

    // ------------------------------------------------------------------------
    // deliver_priv 按 _medeleg 真生效 (RV Privileged Spec §3.1.8)
    //   - M-mode trap (caller priv = M): 总 deliver M
    //   - U/S-mode trap + _medeleg.bit(cause) = 1: deliver S
    //   - U/S-mode trap + _medeleg.bit(cause) = 0: deliver M
    // medeleg bit 11 (CAUSE_ECALL_FROM_M) 由 csr_medeleg_write WARL hardwire 0.
    // cause < 64 边界检查: _medeleg uint64_t (类 1 future-proof RV64).
    // ------------------------------------------------------------------------
    uint8_t deliver_priv;
    if (hart->priv == PRIV_M) {
        deliver_priv = PRIV_M;
    } else if (cause < 64 && ((hart->trap._medeleg >> cause) & 1u)) {
        deliver_priv = PRIV_S;
    } else {
        deliver_priv = PRIV_M;
    }

    hart->trap.xcause[deliver_priv] = cause;                  /* cause < 32, 不含 CAUSE_INTERRUPT_BIT */
    hart->trap.xtval [deliver_priv] = tval;
    hart->trap.xepc  [deliver_priv] = hart->regs[0];          /* 当前指令 PC */

    // ------------------------------------------------------------------------
    // 切 priv mode + 保存 mstatus / sstatus 字段 (按 deliver_priv 分流)
    //   deliver M (§3.1.6.1): MPP=caller, MPIE=MIE, MIE=0
    //   deliver S (§5.1.1):   SPP=caller(1bit), SPIE=SIE, SIE=0
    // 操作 _mstatus 低 32 位 (跟 sstatus 入口共字段, mask 不同); 高 32 位 (mstatush) 不动.
    // ------------------------------------------------------------------------
    {
        uint32_t mstatus_lo = (uint32_t)(hart->trap._mstatus & 0xFFFFFFFFu);

        if (deliver_priv == PRIV_M) {
            mstatus_lo &= ~MSTATUS_MPP;
            mstatus_lo |= ((uint32_t)hart->priv << MSTATUS_MPP_SHIFT) & MSTATUS_MPP;
            if (mstatus_lo & MSTATUS_MIE) mstatus_lo |=  MSTATUS_MPIE;
            else                          mstatus_lo &= ~MSTATUS_MPIE;
            mstatus_lo &= ~MSTATUS_MIE;
        } else {
            /* deliver_priv == PRIV_S; caller 必为 U/S */
            if (hart->priv == PRIV_S) mstatus_lo |=  MSTATUS_SPP;
            else                      mstatus_lo &= ~MSTATUS_SPP;
            if (mstatus_lo & MSTATUS_SIE) mstatus_lo |=  MSTATUS_SPIE;
            else                          mstatus_lo &= ~MSTATUS_SPIE;
            mstatus_lo &= ~MSTATUS_SIE;
        }

        hart->trap._mstatus = (hart->trap._mstatus & 0xFFFFFFFF00000000ULL)
                            | (uint64_t)mstatus_lo;
    }

    hart->priv    = deliver_priv;
    /* Exception 永走 BASE (mode bit mask 掉); D9 解绑 csr_*tvec_write 后 xtvec 低 2 bit 可能
     * 是 mode=1 vectored, exception 函数自己 mask 保证跳到 4-byte 对齐 base. */
    hart->regs[0] = hart->trap.xtvec[deliver_priv] & ~0x3u;

    return hart->trap.in_trap;
}


// ----------------------------------------------------------------------------
// trap_set_interrupt_state —— 架构语义层 (async interrupt 路径), 不长跳; 详见 trap.h doc
//
// 跟 trap_set_exception_state 共用 mstatus 翻转 30 行 (重复 by design; 详 trap.h doc 段
// "跟 trap_set_exception_state 的分歧"). 分歧 5 点全在本函数体现:
//   1. cause_low ∈ [0,32); xcause = cause_low | CAUSE_INTERRUPT_BIT (高位本函数加)
//   2. 查 mideleg (替 _medeleg)
//   3. tval = 0 (RV spec §3.1.16 interrupt tval 永远 0)
//   4. jump-to 处理 vectored mode (mode==1 时 base + 4*cause_low)
//   5. DEBUG 按 cause_low 分流 't'/'s'/'e'
// ----------------------------------------------------------------------------
uint8_t trap_set_interrupt_state(cpu_t *hart, uint32_t cause_low) {
    /* DEBUG trace: cause_low 分流; 在 in_trap++ 后 early return 前打, triple fault 也 trace. */
    switch (cause_low) {
        case IRQ_M_TIMER: case IRQ_S_TIMER: DEBUG_TIME_INTR(); break;
        case IRQ_M_SOFT:  case IRQ_S_SOFT:  DEBUG_SOFT_INTR(); break;
        case IRQ_M_EXT:   case IRQ_S_EXT:   DEBUG_EXT_INTR();  break;
        default: break;  /* 不应到 (trap_check_interrupt 内 priority encoder 只产 6 个合法 IRQ) */
    }

    hart->trap.in_trap++;

    if (hart->trap.in_trap >= 3) {
        return hart->trap.in_trap;
    }

    // ------------------------------------------------------------------------
    // deliver_priv 按 mideleg 真生效 (RV Privileged Spec §3.1.8 跟 medeleg 对偶)
    //   - M-mode trap (caller priv = M): 总 deliver M
    //   - U/S-mode trap + mideleg.bit(cause_low) = 1: deliver S
    //   - U/S-mode trap + mideleg.bit(cause_low) = 0: deliver M
    // cause_low < 32 (IRQ_* 位号最大 11), bit shift 安全; 不需要 < 64 边界检查 (mideleg
    // 是 uint32_t, 跟 _medeleg uint64_t 不同, 见 dummy.txt §6 类 3 vs 类 1).
    // ------------------------------------------------------------------------
    uint8_t deliver_priv;
    if (hart->priv == PRIV_M) {
        deliver_priv = PRIV_M;
    } else if ((hart->trap.mideleg >> cause_low) & 1u) {
        deliver_priv = PRIV_S;
    } else {
        deliver_priv = PRIV_M;
    }

    hart->trap.xcause[deliver_priv] = cause_low | CAUSE_INTERRUPT_BIT;  /* 函数内 OR 高位 */
    hart->trap.xtval [deliver_priv] = 0u;                                /* RV spec interrupt tval=0 */
    hart->trap.xepc  [deliver_priv] = hart->regs[0];

    // ------------------------------------------------------------------------
    // 切 priv mode + mstatus 翻转 (跟 exception 函数完全同 30 行, 不抽 common helper)
    // ------------------------------------------------------------------------
    {
        uint32_t mstatus_lo = (uint32_t)(hart->trap._mstatus & 0xFFFFFFFFu);

        if (deliver_priv == PRIV_M) {
            mstatus_lo &= ~MSTATUS_MPP;
            mstatus_lo |= ((uint32_t)hart->priv << MSTATUS_MPP_SHIFT) & MSTATUS_MPP;
            if (mstatus_lo & MSTATUS_MIE) mstatus_lo |=  MSTATUS_MPIE;
            else                          mstatus_lo &= ~MSTATUS_MPIE;
            mstatus_lo &= ~MSTATUS_MIE;
        } else {
            if (hart->priv == PRIV_S) mstatus_lo |=  MSTATUS_SPP;
            else                      mstatus_lo &= ~MSTATUS_SPP;
            if (mstatus_lo & MSTATUS_SIE) mstatus_lo |=  MSTATUS_SPIE;
            else                          mstatus_lo &= ~MSTATUS_SPIE;
            mstatus_lo &= ~MSTATUS_SIE;
        }

        hart->trap._mstatus = (hart->trap._mstatus & 0xFFFFFFFF00000000ULL)
                            | (uint64_t)mstatus_lo;
    }

    hart->priv = deliver_priv;

    // ------------------------------------------------------------------------
    // jump-to: vectored mode 处理
    //   mtvec/stvec bit[1:0] = MODE (csr_*tvec_write WARL 接 0/1, reserved 落 0; D9)
    //     mode == 0 direct  : pc = base (所有 interrupt 跳同一入口, handler 自己判 cause)
    //     mode == 1 vectored: pc = base + 4 * cause_low (interrupt 按 IRQ 号跳分散入口)
    //     mode == 2/3       : WARL 已落 0, 不会到这条
    // ------------------------------------------------------------------------
    {
        uxlen_t  tvec = hart->trap.xtvec[deliver_priv];
        uint32_t mode = tvec & 0x3u;
        uxlen_t  base = tvec & ~0x3u;
        hart->regs[0] = (mode == 1u) ? (base + 4u * cause_low) : base;
    }

    return hart->trap.in_trap;
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

    (void)trap_set_interrupt_state(hart, irq);
    /* 返非 0 = dispatcher 应 continue. 返 set_interrupt_state 当前 in_trap, 跟
     * trap_set_exception_state 形态一致 (mmu_translate_pc 透传机制). */
    return (int)hart->trap.in_trap;
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
