//
// Created by liujilan on 2026/5/4.
// trap 模块实现 (架构语义层 + 双 raise 入口接通 sigsetjmp 协议 + 真切 priv mode + 写
// mstatus/sstatus 按 deliver_priv 分流)。
//
// 顶部接口 doc 见 trap.h; 跨文件协议见 src/dummy.txt §1。
//
// 未实现:
//   - mideleg-driven deliver_priv (中断机制未做; mideleg 字段就位但 trap_set_state 不读
//     — 项目当前 trap 都是 sync exception, 走 medeleg 路径)
//

#include "trap.h"

#include "cpu.h"        // cpu_t 完整定义 (trap.h 只 forward, 这里要访问字段)
#include "debug.h"      // DEBUG_EXCEPTION (trap_set_state 当前只 sync exception 一种路径)
#include "riscv.h"      // PRIV_M

#include <setjmp.h>     // siglongjmp
#include <stdint.h>


// ----------------------------------------------------------------------------
// trap_set_state —— 架构语义层, 不长跳; 详见 trap.h doc
// ----------------------------------------------------------------------------
uint8_t trap_set_state(cpu_t *hart, uint32_t cause, uint32_t tval) {
    // DEBUG trace: 'E' 打印。当前 trap_set_state 只服务 sync exception 一种语义 (两条
    // 路径都是 exception: trap_raise_exception 内部长跳 + mmu_translate_pc fetch fault
    // 非长跳); 中断机制未接, 中断不过此函数。
    //
    // T5 接 trap_raise_interrupt 时按形态拍位置 (落 a_02 时按场内分支密度决定):
    //   (a) 若 T5 拆 trap_set_exception_state / trap_set_interrupt_state 两个辅助函数,
    //       'E' 留这边, 't'/'s'/'e' 进对偶函数, DEBUG 调用各自独立
    //   (b) 若 T5 共用 trap_set_state + 按 cause 高位 (interrupt bit) 分流, 这条
    //       DEBUG_EXCEPTION() 改成 if-else 按 cause 高位走 't'/'s'/'e' 或 'E'
    // 在 in_trap >= 3 早 return 之前打, triple fault 那次也算 exception 发生 trace 见
    // 'EEE' 数到嵌套深度。
    DEBUG_EXCEPTION();

    hart->trap.in_trap++;

    // 候选 A: 第三次 (含) 进 trap_set_state 早 return, 不 deliver。
    // 字段保留第二次状态作为 root cause, 给 main 端 dump 用。
    //
    // 注: 这是项目自定义"triple fault halt" 协议 (跟 x86 triple fault → reset 风格类似), 不
    // 是 RV Smdbltrp 扩展 (riscv.h CAUSE_DOUBLE_TRAP=16, Smdbltrp 由硬件检查 mstatus.MDT 字段
    // 触发 cause=16 trap; 项目不实现该扩展, 这里 in_trap 计数 + 早 return 跟 Smdbltrp 没关系)。
    if (hart->trap.in_trap >= 3) {
        return hart->trap.in_trap;
    }

    // ------------------------------------------------------------------------
    // deliver_priv 按 medeleg 真生效
    //
    // RV Privileged Spec §3.1.8 medeleg: 同步 exception 委派 bitmask, bit(cause) = 1 时
    // 该 cause 在 U/S-mode 下触发时 deliver 给 S-mode (而不是 M-mode)。规则:
    //   - M-mode trap (caller priv = M): 总 deliver M (M 不能 delegate 给 less-privileged)
    //   - U/S-mode trap + medeleg.bit(cause) = 1: deliver S
    //   - U/S-mode trap + medeleg.bit(cause) = 0: deliver M
    //
    // medeleg bit 11 (CAUSE_ECALL_FROM_M) 由 csr_medeleg_write 入口 WARL hardwire 0
    // (M 不会出现 ecall_from_M trap 在 U/S-mode 下, 这条 bit 永远没意义)。
    //
    // 项目当前 trap 都是 sync exception (中断机制 mip/mie/mideleg 都未实现); 所以
    // trap_set_state 只查 medeleg, 不查 mideleg。中断机制真做时加 mideleg 路径。
    //
    // cause < 64 边界检查: _medeleg uint64_t (类 1 future-proof RV64); cause >= 64 时
    // bit shift UB; 项目当前所有 cause < 32, 边界检查防御。
    // ------------------------------------------------------------------------
    uint8_t deliver_priv;
    if (hart->priv == PRIV_M) {
        deliver_priv = PRIV_M;
    } else if (cause < 64 && ((hart->trap._medeleg >> cause) & 1u)) {
        deliver_priv = PRIV_S;
    } else {
        deliver_priv = PRIV_M;
    }

    hart->trap.xcause[deliver_priv] = cause;
    hart->trap.xtval [deliver_priv] = tval;
    hart->trap.xepc  [deliver_priv] = hart->regs[0];         // 当前指令 PC; mmu_translate_pc
                                                              // 调本 helper 时 regs[0] 是 fetch
                                                              // GVA, 也是该指令 PC, 含义一致

    // ------------------------------------------------------------------------
    // 切 priv mode + 保存 mstatus / sstatus 字段 (按 deliver_priv 分流)
    //
    // deliver M (RV Privileged Spec §3.1.6.1 trap entry to M-mode):
    //   MPP  = caller priv (hart->priv at trap entry; 2-bit)
    //   MPIE = MIE
    //   MIE  = 0
    // deliver S (RV Privileged Spec §5.1.1 trap entry to S-mode):
    //   SPP  = caller priv (hart->priv at trap entry; 1-bit, 0=U, 1=S; M-mode trap 不
    //          delegate, caller 必为 U/S)
    //   SPIE = SIE
    //   SIE  = 0
    //
    // 操作 _mstatus 的低 32 位 (mstatus 入口跟 sstatus 入口共用同字段, mask 不同);
    // 高 32 位 (mstatush) 不动。
    // ------------------------------------------------------------------------
    {
        uint32_t mstatus_lo = (uint32_t)(hart->trap._mstatus & 0xFFFFFFFFu);

        if (deliver_priv == PRIV_M) {
            /* MPP = caller priv */
            mstatus_lo &= ~MSTATUS_MPP;
            mstatus_lo |= ((uint32_t)hart->priv << MSTATUS_MPP_SHIFT)
                          & MSTATUS_MPP;

            /* MPIE = MIE */
            if (mstatus_lo & MSTATUS_MIE) mstatus_lo |=  MSTATUS_MPIE;
            else                          mstatus_lo &= ~MSTATUS_MPIE;

            /* MIE = 0 (trap 入口 disable interrupt) */
            mstatus_lo &= ~MSTATUS_MIE;
        } else {
            /* deliver_priv == PRIV_S */
            /* SPP = caller priv (1-bit; PRIV_U=0, PRIV_S=1; caller 必为 U/S) */
            if (hart->priv == PRIV_S) mstatus_lo |=  MSTATUS_SPP;
            else                      mstatus_lo &= ~MSTATUS_SPP;

            /* SPIE = SIE */
            if (mstatus_lo & MSTATUS_SIE) mstatus_lo |=  MSTATUS_SPIE;
            else                          mstatus_lo &= ~MSTATUS_SPIE;

            /* SIE = 0 */
            mstatus_lo &= ~MSTATUS_SIE;
        }

        hart->trap._mstatus = (hart->trap._mstatus & 0xFFFFFFFF00000000ULL)
                            | (uint64_t)mstatus_lo;
    }

    hart->priv    = deliver_priv;                            /* 切到 deliver priv */
    hart->regs[0] = hart->trap.xtvec[deliver_priv];          /* 跳 trap vector (handler 起点) */

    return hart->trap.in_trap;
}


// ----------------------------------------------------------------------------
// trap_raise_exception —— interpreter / JIT block 内 helper 长跳入口; 详见 trap.h doc
//
// _Noreturn, 内部 trap_set_state + siglongjmp(*hart->jmp_buf_ptr, 1) 跳回 dispatcher 入口
// 的一次性 sigsetjmp 落点。caller (interpreter) 内 goto out 变 unreachable 但保留无害。
// ----------------------------------------------------------------------------
_Noreturn void trap_raise_exception(cpu_t *hart, uint32_t cause, uint32_t tval) {
    (void)trap_set_state(hart, cause, tval);
    siglongjmp(*hart->jmp_buf_ptr, 1);
    // unreachable; siglongjmp 不返回。GCC 在 _Noreturn 函数末尾不需要 return 语句, 但若
    // 漏调 siglongjmp 会触发 -Wreturn-local-addr / 调用方 UB; 此处留空依赖 _Noreturn 标识。
}
