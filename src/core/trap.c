//
// Created by liujilan on 2026/5/4.
// a_01_5_c trap 模块实现 (架构语义层 + 双 raise 入口接通 sigsetjmp 协议)。
//
// 顶部接口 doc 见 trap.h; 跨文件协议见 src/dummy.txt §1。
//
// 阶段演进:
//   a_01_5_b: trap_set_state 候选 A 早 return; trap_raise_exception 普通 return (不 longjmp)
//   a_01_5_c: trap_raise_exception 标 _Noreturn + siglongjmp 跳回 dispatcher 落点
//
// 未实现 (留 a_03+):
//   - 切 priv mode (写 _mstatus.MPP / 当前 hart->priv 等)
//   - mideleg / medeleg-driven deliver_priv (现在 hard-code PRIV_M)
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
    // TODO a_01_5_c: 切 priv mode
    //   _mstatus.MPP[1:0] = hart->priv;     /* 保存 caller priv */
    //   _mstatus.MPIE     = _mstatus.MIE;   /* 保存 interrupt-enable */
    //   _mstatus.MIE      = 0;               /* 进 trap 时 disable interrupt */
    //   hart->priv        = deliver_priv;
    // 当前不动 _mstatus / hart->priv, fixture 不读这些字段 (trap dump 看 in_trap/cause/tval/epc/tvec)

    hart->regs[0] = hart->trap.xtvec[deliver_priv];           // 跳 trap vector (handler 起点)

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
