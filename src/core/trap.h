//
// Created by liujilan on 2026/5/4.
// trap 模块对外接口: trap_csrs_t 物理存储 + 三层 raise 接口
//   exception 路径: trap_set_exception_state (不长跳) + trap_raise_exception (含长跳)
//   interrupt 路径: trap_set_interrupt_state (不长跳) + trap_check_interrupt (dispatcher 主帧 polling 入口)
//
// 跨文件协议见 src/dummy.txt §1; 本模块涉及该协议的三处:
//   - 路径 2a interpreter/JIT helper 经 trap_raise_exception 长跳 (深栈)
//   - 路径 2b dispatcher fetch 路径 (mmu_translate_pc / IALIGN 兜底) 直调 trap_set_exception_state, 不长跳
//   - 路径 2b' dispatcher loop 顶 trap_check_interrupt → trap_set_interrupt_state, 不长跳
//     (跟 2b 同形态 dispatcher 主帧浅栈 return + continue, 详 dummy.txt §1 + §9)
//
// trap_csrs_t 字段分类 (按 dummy.txt §6 CSR 物理存储字段命名五类划分):
//   - 第四类 (按 priv 索引数组): xcause / xtval / xepc / xtvec / xscratch, 4 槽
//     (PRIV_M / PRIV_S / PRIV_VS-slot / PRIV_U)
//   - 第一类 (RV32 物理 64 位, csr 入口拆访问): _mstatus / _medeleg
//     (csr.c 通过对应 csr 入口分别访问)
//   - 第三类 (单字段, 单 csr 入口, 不带前缀): mideleg (MXLEN=32, spec 无 midelegh)
//   - 第 (2a) 类 (副本基本字段, 多 csr 入口同级看 mask 子集): _mie (mie + sie 看)
//   - 第 (5) 类 (软件可写子集 + 异步源 OR 合成读): _mip_sw (mip + sip 入口看;
//     csrr 时跟 CLINT.msip / mtime≥mtimecmp / 未来 PLIC 合成)
//   - host 状态 (非 RV CSR): in_trap, 嵌套 trap 计数器 + 内部停机位段 (位段编码见
//     dispatcher.c 末尾 in_trap 位段编码段); mret 路径只清 bit 0-1
//
// 命名约定 (与 RV 手册 + dummy.txt §6 一致):
//   - "x" 前缀: priv 索引数组 (xcause / xtval / xepc / xtvec); 跟 RV 手册风格一致 (手册
//     用 xepc 同时指代 mepc 和 sepc, 由 deliver priv 决定具体哪个)。csr 大 switch 的
//     read/write helper 把 mepc/sepc 这些 csr 名映射到 xepc[PRIV_M]/xepc[PRIV_S]。
//   - "_" 前缀: 物理存储是 64 位但 csr 入口拆 32 位访问的字段 (_mstatus); csr 大 switch
//     的 mstatus / mstatush 入口映射到 _mstatus 的低/高半边。
//   - 不带前缀: 既不按 priv 索引也不拆 64 位的字段 (in_trap; host 流程状态)。
//
// helper 形态:
//   trap_set_exception_state(hart, cause, tval) -> uint8_t
//     设架构状态 (in_trap++; 写 xcause/xtval/xepc[deliver_priv]; regs[0] = xtvec[deliver_priv]
//     & ~0x3 跳 base 忽略 mode; cause < 32 不含 CAUSE_INTERRUPT_BIT);
//     候选 A 早 return: in_trap >= 3 时不 deliver (字段保留第二次状态作 root cause), 静默
//     in_trap++ 后返回。返回 in_trap 当前值, 给 mmu_translate_pc 那种 caller 用作 0/非0
//     状态传给 dispatcher (省一次读 hart->trap.in_trap)。
//
//   trap_set_interrupt_state(hart, cause_low) -> uint8_t
//     设架构状态 (in_trap++; xcause = cause_low | CAUSE_INTERRUPT_BIT; xtval = 0;
//     xepc = hart->regs[0]; mstatus xPIE/xIE/xPP 翻转 + 切 priv; jump-to = xtvec base
//     (mode=0 direct) 或 base + 4*cause_low (mode=1 vectored)). cause_low ∈ [0, 32)
//     = IRQ_* 位号. mideleg 查 deliver_priv. tval 强制 0 (RV spec §3.1.16 interrupt tval=0).
//     候选 A 早 return 同 trap_set_exception_state.
//
//   trap_check_interrupt(hart) -> int
//     dispatcher 主帧 polling 入口 (每 loop 顶调). 内部 csr_mip_read 合成读 (含 CLINT
//     pending) → mip & mie & 按 priv 全局 IE / mideleg 算 deliver_mask → ready 非 0
//     按 RV Priv Spec §3.1.9 优先级 (M_EXT > M_SOFT > M_TIMER > S_EXT > S_SOFT > S_TIMER)
//     选第一非零位 → 调 trap_set_interrupt_state. 返 0 = 无 fire, dispatcher 继续 fetch;
//     返非 0 = 已 trap_set_interrupt_state, dispatcher 应 continue. trace 互斥协议见
//     debug.h 顶段 (ready==0 时本函数打 'c'; ready!=0 时 trap_set_interrupt_state 内打 t/s/e).
//
//   trap_raise_exception(hart, cause, tval) -> _Noreturn
//     内部 trap_set_exception_state + siglongjmp(*hart->jmp_buf_ptr, 1) 跳回 dispatcher 入口
//     一次性 sigsetjmp 落点。caller (interpreter) 内 goto out 变 unreachable 但保留无害
//     (GCC -Wunreachable-code 默认 disabled, 不警告)。
//

#ifndef CORE_TRAP_H
#define CORE_TRAP_H

#include <stdint.h>

#include "riscv.h"  // uxlen_t / u64_t (typedef family; dummy.txt §13)

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
    // 第四类: 按 priv 索引数组 (mepc/sepc, mtval/stval, mcause/scause, mtvec/stvec, mscratch/sscratch)
    // index = deliver priv: PRIV_M=3 / PRIV_S=1; PRIV_U=0 / PRIV_H=2 槽位不用但保 4 对齐
    // (cpu_t.tlb_table[4] 同风格, 直接 [priv] 索引免减法)。
    // 命名 "x" 前缀对应 RV 手册 xepc 风格 (xepc[PRIV_M] = mepc, xepc[PRIV_S] = sepc 等);
    // csr 大 switch 的 mepc / sepc / ... read/write helper 映射到对应槽位。
    // 详见 dummy.txt §6 CSR 物理存储字段命名四类划分。
    uxlen_t   xcause[4];
    uxlen_t   xtval[4];
    uxlen_t   xepc[4];
    uxlen_t   xtvec[4];
    uxlen_t   xscratch[4];     // mscratch=xscratch[PRIV_M], sscratch=xscratch[PRIV_S]

    // 第一类: RV32 物理 64 位, csr 入口拆 32 位访问 (dummy.txt §6)。
    // _mstatus: csr 入口 mstatus + mstatush 拆访问低/高半边 (RV32 ABI)。sstatus 是
    //   _mstatus 的 masked view (SSTATUS_MASK)。
    // _medeleg: priv spec 1.12 定义 medelegh (0x312) 为 RV32 高 32 位入口, 物理 64 位
    //   存 cause bitmap (RV64 单入口整体访问)。项目当前只实装 medeleg 低 32 位入口
    //   (medelegh 未实装, 跟 mstatush 平行 future), 字段类型 uint64_t 已 RV64-ready。
    //   trap_set_exception_state 按 _medeleg.bit(cause) 派发 deliver_priv (U/S-mode trap +
    //   bit=1 → deliver S, 否则 deliver M; M-mode trap 总 M)。
    u64_t     _mstatus;
    u64_t     _medeleg;        // csr 入口 medeleg (0x302); medelegh (0x312) 未实装

    // 第三类: 单字段, 单 csr 入口, 不带前缀 (dummy.txt §6)。
    // mideleg: MXLEN-bit (RV32 = 32 位; spec 未定义 midelegh, 中断 cause 不会超 32 位)。
    //   trap_set_interrupt_state 按 mideleg.bit(cause_low) 派发 deliver_priv
    //   (U/S-mode 时 bit=1 → S, bit=0 → M; M-mode 总 M)。trap_check_interrupt 算
    //   deliver_mask 时也读 mideleg 决定哪些 IRQ 在当前 priv 下接受。
    uxlen_t   mideleg;         // csr 入口 mideleg (0x303)

    // 第 (2a) 类: 副本基本字段 (mie / sie 两 csr 入口同级看不同 mask 子集, dummy.txt §6)。
    // mie 入口看全部 32 位 (项目用 bit 1/3/5/7/9/11 = IRQ_S/M × SOFT/TIMER/EXT 六位,
    //   见 riscv.h IRQ_* 宏); sie 入口看 32 位 ∩ SIE_MASK = IRQ_S × SOFT/TIMER/EXT
    //   三位。两入口实现独立, 关系是"同级看不同 mask"。
    uxlen_t   _mie;            // csr 入口 mie (0x304) / sie (0x104) 同级 mask view

    // 第 (5) 类: 软件可写子集 + 异步源 OR 合成读 (dummy.txt §6 第 5 类)。
    // mip 入口的软件可写子集物理存储:
    //   bit 1 = SSIP        (IRQ_S_SOFT;  M/S csrw 都可 inject)
    //   bit 5 = STIP        (IRQ_S_TIMER; M csrw inject 用; 项目无 Sstc 无硬件源)
    //   bit 9 = SEIP_sw     (IRQ_S_EXT;   M csrw inject; csrr SEIP 时跟 PLIC hw_seip OR)
    // 其他位永远 0 (csrw 截 MIP_SW_WRITABLE_MASK, csrr mip 由 csr_mip_read 合成).
    //
    // csrr mip 时跟以下异步源 OR 合成完整 mip readout:
    //   bit 3  MSIP ← CLINT.msip[hartid]          (atomic_load; 跨 hart MMIO writer)
    //   bit 7  MTIP ← (mtime ≥ mtimecmp[hartid]) (is_clint_timer_pending compute)
    //   bit 9  SEIP_hw ← PLIC s_pending           (未来; v1 永远 0)
    //   bit 11 MEIP ← PLIC m_pending              (未来; v1 永远 0)
    // csrw mip 只动本字段 MIP_SW_WRITABLE_MASK 对应位; 其他位 RO 写忽略。
    //
    // 并发: 本字段 plain uint32_t (本 hart 单线程访问 — M-mode csrw mip / S-mode
    // csrw sip 都是本 hart guest 软件; dispatcher 读也本 hart loop 顶). 跨 hart
    // inject SSIP 走 IPI 路径 (远 hart 写 clint.msip → 目标 hart MSIP trap →
    // 目标 hart M-handler 自己 csrw mip 设 SSIP), 仍是本 hart 写自己 _mip_sw。
    // SMP/H/AIA: H 扩展 (跨虚拟 hart inject) / AIA (IMSIC MSI 直接写远 hart) 会
    // 破坏 "本 hart 单线程访问 _mip_sw" 前提, 真做时需改 _Atomic + atomic 路径。
    uxlen_t   _mip_sw;         // csr 入口 mip (0x344) / sip (0x144) 软件可写子集

    // host trap 流程状态: 嵌套 trap 计数器 + 内部停机位段 (位段编码详见 dispatcher.c
    // 末尾 in_trap 位段编码段)。
    //   bit 0-1: 嵌套深度
    //     0 = 正常 execution
    //     1 = 第一次 trap (handler 跑)
    //     2 = handler 内再 trap (double, 项目仍尝试 deliver)
    //     3 = handler 又 trap (triple, 候选 A 早 return 不 deliver, dispatcher 看 while
    //         条件 in_trap < 3 失败退出)
    //   bit 3+ : 内部异常 / 内部正常停机 (host 端协议, 由 dispatcher 自己设, 解释器 / JIT
    //           不碰)
    // mret 路径只清 bit 0-1 (高位停机标记由 reset 流程决定是否清; 当前在 OP_MRET case 内
    // 实现复位)。
    //
    // 注意: 这是项目自定义协议, 跟 RV Smdbltrp 扩展定义的 double trap 不同:
    //   - RV Smdbltrp (Machine Double-Trap, riscv.h CAUSE_DOUBLE_TRAP=16): 标准扩展, trap
    //     entry 时硬件检查 mstatus.MDT 字段, 若 MDT=1 (上次 trap 还没退) 触发 cause=16 trap
    //     由硬件 deliver。这是规范定义的 trap delivery 机制。
    //   - 本字段 in_trap: 项目自定义计数器, 跟 mstatus.MDT 不是同一个东西; 候选 A 早 return
    //     是 host emulator 自己实现的 "triple fault halt" 退出协议, 跟 cause=16 没关系。
    //   项目不实现 Smdbltrp 扩展, mcause 字段不会出现 16 这个值; CAUSE_DOUBLE_TRAP 宏先列在
    //   riscv.h 是为了 Exception Code 表完整 + 未来真做扩展时直接引用, 当前代码不引用此宏。
    uint8_t   in_trap;
} trap_csrs_t;


// ----------------------------------------------------------------------------
// trap_set_exception_state —— 架构语义层 (sync exception 路径), 不长跳
//
// 调用方:
//   - mmu_translate_pc (dummy.txt §1 路径 2b, 直接 control flow)
//   - dispatcher.c IALIGN 兜底 (dummy.txt §9 单一源)
//   - trap_raise_exception 内部 (复用本 helper 的"写字段+计数")
//
// 行为:
//   in_trap++;
//   if (in_trap >= 3) return in_trap;     /* 候选 A: 早 return 不 deliver */
//   deliver_priv = (caller == M) ? M : (_medeleg.bit(cause) ? S : M);
//   xcause[deliver_priv] = cause;          /* cause < 32, 不含 CAUSE_INTERRUPT_BIT */
//   xtval[deliver_priv]  = tval;
//   xepc[deliver_priv]   = hart->regs[0];
//   /* 切 priv mode + 翻 mstatus xPIE/xIE/xPP */
//   hart->priv    = deliver_priv;
//   hart->regs[0] = xtvec[deliver_priv] & ~0x3u;  /* exception 永走 base, 忽略 MODE */
//   return in_trap;
uint8_t trap_set_exception_state(cpu_t *hart, uint32_t cause, uxlen_t tval);


// ----------------------------------------------------------------------------
// trap_set_interrupt_state —— 架构语义层 (async interrupt 路径), 不长跳
//
// 调用方:
//   - trap_check_interrupt 内部 (dispatcher 主帧 polling 入口, dummy.txt §1 路径 2b')
//
// 跟 trap_set_exception_state 的分歧 (按"对偶不教条"原则拆函数):
//   1. cause 高位由本函数加 (caller 传 cause_low = IRQ_* 位号, 函数内 OR
//      CAUSE_INTERRUPT_BIT 写 xcause)
//   2. deliver_priv 查 mideleg (替 _medeleg)
//   3. tval 强制 0 (RV spec §3.1.16; 中断 tval 永远 0)
//   4. jump-to 处理 vectored mode (mtvec/stvec MODE=1 时 base + 4*cause_low)
//   5. DEBUG 字符按 cause_low 分流 ('t'/'s'/'e')
// mstatus xPIE/xIE/xPP 翻转 / in_trap 协议 / 候选 A 早 return 完全跟 exception 路径一样
// (30 行重复 by design, 不抽 common helper; 未来 H 扩展 hstatus 增量铺路时再抽).
//
// 行为:
//   in_trap++;
//   if (in_trap >= 3) return in_trap;
//   deliver_priv = (caller == M) ? M : (mideleg.bit(cause_low) ? S : M);
//   xcause[deliver_priv] = cause_low | CAUSE_INTERRUPT_BIT;
//   xtval[deliver_priv]  = 0;
//   xepc[deliver_priv]   = hart->regs[0];
//   /* 切 priv mode + 翻 mstatus xPIE/xIE/xPP */
//   hart->priv    = deliver_priv;
//   /* jump-to: mode==1 vectored 跳 base+4*cause_low; 否则 (0 direct / 2/3 reserved 落 direct) 跳 base */
//   uint32_t tvec = xtvec[deliver_priv];
//   hart->regs[0] = (tvec & 0x3u) == 1u ? (tvec & ~0x3u) + 4u*cause_low : (tvec & ~0x3u);
//   return in_trap;
uint8_t trap_set_interrupt_state(cpu_t *hart, uint32_t cause_low);


// ----------------------------------------------------------------------------
// trap_check_interrupt —— dispatcher 主帧 polling 入口, 不长跳
//
// 调用方: dispatcher.c loop 顶 (每轮 while 体进入一次).
//
// 返 0  = 无中断 deliver, dispatcher 继续 fetch
// 返非 0 = 已调 trap_set_interrupt_state, dispatcher 应 continue (重派发 from xtvec)
//
// 内部步骤:
//   1. mip_view = csr_mip_read(hart)                        // 合成读: _mip_sw | clint pending
//   2. enabled = mip_view & hart->trap._mie
//   3. deliver_mask = 按当前 priv + mstatus.MIE/SIE + mideleg 决定哪些 IRQ 在本 priv 接受
//        priv=M: mstatus.MIE ? MIE_VALID_MASK : 0
//        priv=S: (~mideleg & MIE_VALID_MASK) | (mstatus.SIE ? (mideleg & SIE_MASK) : 0)
//        priv=U: MIE_VALID_MASK
//   4. ready = enabled & deliver_mask;  ready == 0 → DEBUG_INT_CHECK 'c' + return 0
//   5. RV Priv Spec §3.1.9 优先级 M_EXT > M_SOFT > M_TIMER > S_EXT > S_SOFT > S_TIMER 选第一非零位
//   6. trap_set_interrupt_state(hart, irq);  return 非 0
//
// trace 互斥协议 (debug.h 顶段): ready==0 时本函数打 'c'; ready!=0 时由
// trap_set_interrupt_state 内按 cause_low 打 't/s/e'. 每轮入口必出且仅出一个 ∈ {c,t,s,e}.
int trap_check_interrupt(cpu_t *hart);


// ----------------------------------------------------------------------------
// trap_raise_exception —— interpreter / JIT block 内 helper 长跳入口 (_Noreturn)
//
// 调用方: interpreter case (OP_UNSUPPORTED / WRITE_PC_OR_TRAP 内 misalign / csr 权限失败
// 等), 未来 JIT translator emit 出来的 host code 同样接本 helper。
//
// 内部 trap_set_exception_state + siglongjmp(*hart->jmp_buf_ptr, 1) 跳回 dispatcher 入口
// 的一次性 sigsetjmp 落点。caller 内 goto out 变 unreachable 但保留无害 (GCC
// -Wunreachable-code 默认 disabled, 不警告).
//
// 跟 interrupt 路径形态不对偶 by design: exception 在 helper 深栈必须 longjmp 才能跳回
// dispatcher 主帧; interrupt 在 dispatcher 主帧浅栈 return + continue 即可. 见
// dummy.txt §1 路径 2a vs 2b'. 按"对偶不教条" 原则承认形态差异, trap_set_*_state 内
// 核共用思想.
_Noreturn void trap_raise_exception(cpu_t *hart, uint32_t cause, uxlen_t tval);

#endif //CORE_TRAP_H
