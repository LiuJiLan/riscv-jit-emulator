//
// Created by liujilan on 2026/5/4.
// a_01_5_c → a_01_7 trap 模块实现 (架构语义层 + 双 raise 入口接通 sigsetjmp 协议 +
//           a_01_7 起真切 priv mode + 写 mstatus.MPP/MPIE/MIE)。
//
// 顶部接口 doc 见 trap.h; 跨文件协议见 src/dummy.txt §1。
//
// 阶段演进:
//   a_01_5_b: trap_set_state 候选 A 早 return; trap_raise_exception 普通 return (不 longjmp)
//   a_01_5_c: trap_raise_exception 标 _Noreturn + siglongjmp 跳回 dispatcher 落点
//   a_01_7:   trap_set_state 真切 priv (写 _mstatus 的 MPP/MPIE/MIE; 改 hart->priv = deliver_priv);
//             OP_MRET 真切 priv 在 interpreter.c case 内做 (反操作)
//
// 未实现 (留 a_03+):
//   - mideleg / medeleg-driven deliver_priv (现在仍 hard-code PRIV_M)
//

#include "trap.h"

#include "cpu.h"        // cpu_t 完整定义 (trap.h 只 forward, 这里要访问字段)
#include "riscv.h"      // PRIV_M

#include <setjmp.h>     // siglongjmp (a_01_5_c)
#include <stdint.h>


// ----------------------------------------------------------------------------
// trap_set_state —— 架构语义层, 不长跳; 详见 trap.h doc
// ----------------------------------------------------------------------------
uint8_t trap_set_state(cpu_t *hart, uint32_t cause, uint32_t tval) {
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

    // a_01_5_b v0: deliver_priv hard-code PRIV_M。
    // 未来 (真接 OS / mideleg/medeleg 路径): 按 cause 查 mideleg/medeleg 决定 deliver 到 M 还是 S。
    const uint8_t deliver_priv = PRIV_M;

    hart->trap.xcause[deliver_priv] = cause;
    hart->trap.xtval [deliver_priv] = tval;
    hart->trap.xepc  [deliver_priv] = hart->regs[0];         // 当前指令 PC; mmu_translate_pc
                                                              // 调本 helper 时 regs[0] 是 fetch
                                                              // GVA, 也是该指令 PC, 含义一致

    // ------------------------------------------------------------------------
    // a_01_7: 切 priv mode + 保存 mstatus 字段
    //
    // RV Privileged Spec §3.1.6.1 trap entry:
    //   MPP  = caller priv (hart->priv at trap entry)
    //   MPIE = MIE
    //   MIE  = 0
    //   priv = deliver_priv
    //
    // 操作 _mstatus 的低 32 位 (mstatus 入口); 高 32 位 (mstatush) 不动 (RV32 spec mstatush 是
    // SBE/MBE 等 endian 控制位, trap 不影响)。
    // ------------------------------------------------------------------------
    {
        uint32_t mstatus_lo = (uint32_t)(hart->trap._mstatus & 0xFFFFFFFFu);

        /* MPP = caller priv (hart->priv 在切之前的值) */
        mstatus_lo &= ~MSTATUS_MPP;                                     /* 清旧 MPP */
        mstatus_lo |= ((uint32_t)hart->priv << MSTATUS_MPP_SHIFT)       /* 放新 MPP */
                      & MSTATUS_MPP;

        /* MPIE = MIE */
        if (mstatus_lo & MSTATUS_MIE) mstatus_lo |=  MSTATUS_MPIE;
        else                          mstatus_lo &= ~MSTATUS_MPIE;

        /* MIE = 0 (trap 入口 disable interrupt) */
        mstatus_lo &= ~MSTATUS_MIE;

        hart->trap._mstatus = (hart->trap._mstatus & 0xFFFFFFFF00000000ULL)
                            | (uint64_t)mstatus_lo;
    }

    hart->priv    = deliver_priv;                            /* 切到 deliver priv (a_01 = M) */
    hart->regs[0] = hart->trap.xtvec[deliver_priv];          /* 跳 trap vector (handler 起点) */

    return hart->trap.in_trap;
}


// ----------------------------------------------------------------------------
// trap_raise_exception —— interpreter / JIT block 内 helper 长跳入口; 详见 trap.h doc
//
// a_01_5_c 形态: _Noreturn, 内部 trap_set_state + siglongjmp(*hart->jmp_buf_ptr, 1) 跳回
//                dispatcher 入口的一次性 sigsetjmp 落点。caller (interpreter) 内 goto out
//                变 unreachable 但保留无害。
// ----------------------------------------------------------------------------
_Noreturn void trap_raise_exception(cpu_t *hart, uint32_t cause, uint32_t tval) {
    (void)trap_set_state(hart, cause, tval);
    siglongjmp(*hart->jmp_buf_ptr, 1);
    // unreachable; siglongjmp 不返回。GCC 在 _Noreturn 函数末尾不需要 return 语句, 但若
    // 漏调 siglongjmp 会触发 -Wreturn-local-addr / 调用方 UB; 此处留空依赖 _Noreturn 标识。
}
