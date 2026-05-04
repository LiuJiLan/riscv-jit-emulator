//
// Created by liujilan on 2026/5/4.
// a_01_5_b trap 模块对外接口: trap_csrs_t 物理存储 + 两层 raise 接口
//   (trap_set_state 不长跳, trap_raise_exception 含长跳; a_01_5_b 后者还不接 longjmp,
//    a_01_5_c 才标 _Noreturn + siglongjmp)。
//
// 跨文件协议见 src/dummy.txt §1; 本模块涉及该协议的两处:
//   - 机制 (2a) interpreter helper 经 trap_raise_exception 长跳 (a_01_5_c 真激活)
//   - 机制 (2b) dispatcher fetch 路径 (mmu_translate_pc) 直调 trap_set_state, 不长跳
//
// trap_csrs_t 字段分类 (按 dummy.txt §1 用户拆出的四类 CSR 镜像):
//   - 第四类 (按 priv 索引数组): xcause / xtval / xepc / xtvec, 4 槽 (PRIV_M / PRIV_S /
//     PRIV_VS-slot / PRIV_U), a_01_5_b v0 只用 [PRIV_M]
//   - 第一类 (RV32 物理 64 位, csr 入口拆访问): _mstatus, csr.c 通过 mstatus / mstatush
//     两个 csr 入口分别访问低/高 32 位 (sstatus 是 _mstatus 的 masked view, a_01_5_b
//     不实现, 等 S-mode 时加)
//   - host 状态 (非 RV CSR): in_trap, double fault detection 计数器, mret 复位为 0
//
// 命名约定 (与 RV 手册 + dummy.txt §1 用户拍板一致):
//   - "x" 前缀: priv 索引数组 (xcause / xtval / xepc / xtvec); 跟 RV 手册风格一致 (手册
//     用 xepc 同时指代 mepc 和 sepc, 由 deliver priv 决定具体哪个)。csr 大 switch 的
//     read/write helper 把 mepc/sepc 这些 csr 名映射到 xepc[PRIV_M]/xepc[PRIV_S]。
//   - "_" 前缀: 物理存储是 64 位但 csr 入口拆 32 位访问的字段 (_mstatus); csr 大 switch
//     的 mstatus / mstatush 入口映射到 _mstatus 的低/高半边。
//   - 不带前缀: 既不按 priv 索引也不拆 64 位的字段 (in_trap; host 流程状态)。
//
// helper 形态:
//   trap_set_state(hart, cause, tval) -> uint8_t
//     设架构状态 (in_trap++; 写 xcause/xtval/xepc[deliver_priv]; regs[0] = xtvec[deliver_priv]);
//     候选 A 早 return: in_trap >= 3 时不 deliver (字段保留第二次状态作 root cause), 静默
//     in_trap++ 后返回。返回 in_trap 当前值, 给 mmu_translate_pc 那种 caller 用作 0/非0
//     状态传给 dispatcher (省一次读 hart->trap.in_trap)。
//
//   trap_raise_exception(hart, cause, tval) -> _Noreturn (a_01_5_c 起)
//     内部 trap_set_state + siglongjmp(*hart->jmp_buf_ptr, 1) 跳回 dispatcher 入口
//     一次性 sigsetjmp 落点。caller (interpreter) 内 goto out 变 unreachable 但保留无害
//     (GCC -Wunreachable-code 默认 disabled, 不警告)。
//

#ifndef CORE_TRAP_H
#define CORE_TRAP_H

#include <stdint.h>

// ----------------------------------------------------------------------------
// forward decl cpu_t —— 因为 cpu.h 内嵌 trap_csrs_t 字段时要 #include "trap.h", 而本头
// 文件的 helper 签名又要用 cpu_t*。如果两边互 include 会形成循环 include + 跳过 → cpu_t
// 引用未定义。
//
// 解决: cpu.h 改用具名 struct (typedef struct cpu_s {...} cpu_t;), 本头文件 forward
// typedef 同名:
//   typedef struct cpu_s cpu_t;
// C11 §6.7p3 允许同一名字 typedef 多次, 只要 underlying type 一致 (这里两处都是
// struct cpu_s, 一致)。GCC -std=c11 -Wall 不报。
//
// 注: 这条 forward typedef 是 trap.h 的"工程实现细节", 不是项目设计意图; cpu_s 这个
// tag 名也是仅为本 forward decl 而存在, 没人用 struct cpu_s 这个名字直接做事。
// ----------------------------------------------------------------------------
typedef struct cpu_s cpu_t;


// ----------------------------------------------------------------------------
// trap_csrs_t —— trap-related CSR 物理存储 + host trap 流程状态
//
// 内嵌 cpu_t 末尾 (cpu.h), 与 jmp_buf_ptr 等"持指针"字段不同, 这是 guest 状态, 与 cpu_t
// 生死同源, 直接内嵌。 sizeof ≈ 80 B (4*4*4 + 8 + 1 + padding), 不强制 _Alignas(64)
// 因为 fast path 只在 cpu_t 前 128 B (regs[32]), trap_csrs_t 在后, 不冲突。
// ----------------------------------------------------------------------------
typedef struct {
    // 第四类: 按 priv 索引数组 (mepc/sepc, mtval/stval, mcause/scause, mtvec/stvec)
    // index = deliver priv: PRIV_M=3 / PRIV_S=1; PRIV_U=0 / PRIV_H=2 槽位不用但保 4 对齐
    // (cpu_t.tlb_table[4] 同风格, 直接 [priv] 索引免减法)。
    // 命名 "x" 前缀对应 RV 手册 xepc 风格 (xepc[PRIV_M] = mepc, xepc[PRIV_S] = sepc 等);
    // csr 大 switch 的 mepc / sepc / ... read/write helper 映射到对应槽位。
    // a_01_5_b v0 只写 [PRIV_M] 槽。
    uint32_t  xcause[4];
    uint32_t  xtval[4];
    uint32_t  xepc[4];
    uint32_t  xtvec[4];

    // 第一类: RV32 物理 64 位 (mstatus + mstatush 两个 csr 入口拆访问)。
    // sstatus 是 _mstatus 的 masked view, a_01_5_b 不实现。
    // a_01_5_b 内 csr.c 小 helper 仍是 fprintf 占位, 暂不真读写本字段; a_01_5_c csr.c
    // 改造时 csr_mstatus_read / csr_mstatush_read / csr_<*>_write 才真接本字段。
    uint64_t  _mstatus;

    // host trap 流程状态: 嵌套 trap 计数器 (0 / 1 / 2 / 3+)。
    //   0 = 正常 execution
    //   1 = 第一次 trap (handler 跑)
    //   2 = handler 内再 trap (double, 项目仍尝试 deliver)
    //   3+ = handler 又 trap (triple, 候选 A 早 return 不 deliver, dispatcher 看 while
    //        条件 in_trap < 3 失败退出)
    // mret 路径复位为 0 (a_01_5_c 加 OP_MRET 时实现)。
    uint8_t   in_trap;
} trap_csrs_t;


// ----------------------------------------------------------------------------
// trap_set_state —— 架构语义层, 不长跳
//
// 调用方:
//   - mmu_translate_pc (dummy.txt §1 路径 2b, 直接 control flow)
//   - trap_raise_exception 内部 (复用本 helper 的"写字段+计数")
//
// 行为:
//   in_trap++;
//   if (in_trap >= 3) {
//       /* 候选 A: 早 return, 不写 xcause/xtval/xepc, 不跳 xtvec; 字段保留第二次状态 */
//       return in_trap;
//   }
//   deliver_priv = PRIV_M  /* a_01_5_b v0: hard-code; 未来 mideleg/medeleg-driven */
//   xcause[deliver_priv] = cause;
//   xtval[deliver_priv]  = tval;
//   xepc[deliver_priv]   = hart->regs[0];
//   /* TODO a_01_5_c: 切 priv mode (写 _mstatus.MPP 等) */
//   hart->regs[0] = xtvec[deliver_priv];
//   return in_trap;
//
// 返回值: in_trap 当前值 (++ 后)。caller 用作"trap 是否已处理"的 0/非0 信号 (机制不依赖
// 返回值, dispatcher while 条件兜底; 但 mmu_translate_pc 等 caller 透传给 dispatcher 省
// 一次读 hart->trap.in_trap)。
uint8_t trap_set_state(cpu_t *hart, uint32_t cause, uint32_t tval);


// ----------------------------------------------------------------------------
// trap_raise_exception —— interpreter / JIT block 内 helper 长跳入口 (a_01_5_c 起 _Noreturn)
//
// 调用方: interpreter case (OP_UNSUPPORTED / WRITE_PC_OR_TRAP 内 misalign / csr 权限失败
// 等), 未来 JIT translator emit 出来的 host code 同样接本 helper。
//
// a_01_5_c 形态: _Noreturn, 内部 trap_set_state + siglongjmp(*hart->jmp_buf_ptr, 1) 跳回
//                dispatcher 入口的一次性 sigsetjmp 落点。caller 内 goto out 变 unreachable
//                但保留无害 (GCC -Wunreachable-code 默认 disabled, 不警告)。
//
// 接口形态稳定, caller 不动。
_Noreturn void trap_raise_exception(cpu_t *hart, uint32_t cause, uint32_t tval);

#endif //CORE_TRAP_H
