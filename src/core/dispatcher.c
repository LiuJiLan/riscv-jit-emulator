//
// Created by liujilan on 2026/4/28.
// dispatcher 实现 (block 1+2+3 调度; sigsetjmp 永久落点 + while(in_trap<3) 多块循环)。
// 跨文件协议见 src/dummy.txt §1 (sigsetjmp) / §4 (TLB 分发机制)。
//

#include "dispatcher.h"

#include "config.h"
#include "cpu.h"
#include "debug.h"      // DEBUG_REFETCH (块边界 / 跨页 / longjmp 回 sigsetjmp 落点都触发)
#include "interpreter.h"
#include "mmu.h"
#include "tlb.h"
#include "trap.h"       // trap_set_exception_state (IALIGN 兜底); trap_check_interrupt (loop 顶 polling)
#include "riscv.h"
#include "runtime.h"    // system_reset_signal (主循环 check + 函数末 set 0 触发停机)

#include <inttypes.h>
#include <setjmp.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>       // perf: clock_gettime ([perf] 主循环计时; DEBUG_PERF_ON gate)


// ============================================================================
// dispatcher 的作用 (设计文档)
// ============================================================================
//
// I. 完成 block 级别的 pc → PA / HVA 翻译 (block 入口包准备)
//    - PA 给 JIT hash (jit_cache 用 PA 为 key 找 host_code_ptr)
//    - HVA 给解释器直接读字节取指 (依赖块边界保证不跨 4K page)
//
// II. 在 helper longjmp 跳回时承接控制流 (sigsetjmp landing)
//
// III. 迭代头扫尾 (count 累加 / 未来 mtime 推进 / 中断检查 / perf_advance)
//
//
// 形态: sigsetjmp 一次性 + while(hart->trap.in_trap < 3) 多块循环。
//
//   - sigsetjmp(*hart->jmp_buf_ptr, 1) 在 dispatcher 入口一次性建立永久落点 (见 dummy.txt §1
//     机制 (1) "为什么 sigsetjmp 在 while 外"段)。落点同时承接两种路径:
//       (i)  初次进入 dispatcher (sigsetjmp 返回 0, 顺序到 while 顶判条件)
//       (ii) helper longjmp 跳来 (siglongjmp 返回非 0, 控制流到 sigsetjmp 落点, 顺序到
//            while 顶重新判 in_trap; trap_set_exception_state 内已设 hart->regs[0]=xtvec, 自然
//            从 trap handler 继续)
//     sigsetjmp 返回值不被分流 — longjmp 不携带"trap 错误"语义, 只是无条件控制流原语。
//
//   - 退出条件: hart->trap.in_trap < 3 不再成立 — bit 0-1 进 triple fault (值=3) 或高位
//     (bit 3+) 被 dispatcher 自己设 (内部异常 / 未来停机)。具体编码见本文件末尾 in_trap
//     位段编码段。trap_set_exception_state / trap_set_interrupt_state 内 in_trap >= 3 时早
//     return 不 deliver (候选 A); main 端拿回控制后 dump halt 状态, 未来 reset 由 dispatcher
//     自己处理。
//
//   - mmu_translate_pc 路径 (dummy.txt §1 路径 C, 不长跳): rc != 0 → continue, 让 while
//     条件接管。
//
//   - total_count + local_count 必须 volatile (跨 longjmp 不被编译器放 callee-saved 寄存
//     器丢值, 见 dummy.txt §1 末段); 必须在 sigsetjmp 之前声明 + 初始化, 否则 longjmp
//     跳回时重新执行初始化, 累计丢失。
//
//
// count 同步契约 (跟 JIT prologue/epilogue 哲学一致):
//   dispatcher 持 local_count 接 interpret_one_block 出参 count_out; interpreter 内
//   count 是栈局部变量, longjmp 跳走时 *count_out = count 必须已写过, 否则 dispatcher
//   收到 local_count = 0, total_count 累加 0 = 丢一个 block 的 count。
//
//   interpreter 端协议 (interpret_one_block 内 SYNC_COUNT 宏 doc + switch 顶部协议段):
//     (a) may-trap 路径: case 内 SYNC_COUNT() 在 trap_raise / helper-call 之前
//     (b) boundary 路径: case 末 break + fetch loop 末段 boundary check → goto out →
//                        out 段 SYNC_COUNT() 托底 (csr/sfence/mret/branch/jal/jalr
//                        走这条 — decode.h is_block_boundary_inst 列表)
//     (c) pure case: 不需要同步 (走完 fetch loop 末段 count++ 后下轮 while 继续;
//                     count 直到 (b) boundary out 才传出)
//
//   dispatcher 端 (扫尾搬迭代头哲学):
//     (1) local_count 必须 volatile + 提到 sigsetjmp 之前声明 — 跟 total_count 同处理,
//         跨 longjmp 持久 (longjmp 不动栈, sigsetjmp 之前声明的栈上变量值保留)
//     (2) 累加 + reset 搬到迭代头 (while 体顶, block 1 之前):
//           total_count += local_count;
//           local_count = 0;
//         不能放迭代尾 (block 3 之后) — longjmp 跳回 setjmp 落点会跳过迭代尾,
//         上一轮 SYNC_COUNT 写好的 local_count 会被下一轮 reset 覆盖, 总值丢失
//     (3) longjmp 路径跟正常 continue 路径都经过迭代头, 两条路径同形态扫尾
//
//   RV precise trap 语义对齐: trap 触发指令本身不算入 count_out (count++ 在 case 末,
//   但 SYNC_COUNT 在 trap_raise 之前; 所以 trap 那条 count 没 ++)。
//
//   未来扫尾扩展 (mtime 推进 / 中断检查 / perf_advance) 也搬迭代头, 不放迭代尾 (同 (2)
//   原因)。当前未接, 占位等接入。
// ============================================================================


void dispatcher(cpu_t *hart) {
    // sigsetjmp / siglongjmp 协议见 src/dummy.txt §1。
    sigjmp_buf dispatch_env;
    hart->jmp_buf_ptr = &dispatch_env;

    // total_count + local_count: 跨 longjmp 累加 / 持久, volatile 强制放栈 (dummy.txt §1
    // 末段)。两个变量都必须在 sigsetjmp 之前声明 + 初始化, 否则 longjmp 跳回时重新执行初始化,
    // local_count 丢失上一轮 SYNC_COUNT 写好的值, total_count 累加丢失。
    // total_count / local_count 类型统一 uint64_t: RV spec mtime/time 是 64 位无符号,
    // 跟 perf_counters / cycle / instret 等 64-bit 字段铺路。32 位 ~4G 指令在 1GHz host
    // 下几秒就溢出, 不可用。local_count 也 64 bit, 跟 interpret_one_block 的 count_out
    // 签名 + 内部 count 类型一致 — "count 一律 64 bit" 心智。x86-64 host 上 32/64 ALU
    // 单 cycle 等价, .text +REX prefix 几字节可忽略 (性能差 ≈ 0)。
    volatile uint64_t total_count = 0;
    volatile uint64_t local_count = 0;

    // === perf timing (DEBUG_PERF_ON gate; 见 debug.h) ===
    // 紧挨主循环前打 t_start —— 必须在 sigsetjmp 之前 (longjmp 跳回不重跑此行,
    // 跟 total_count 同处理)。t_start 在 setjmp/longjmp 窗口内不被修改, 无需 volatile。
    // Release 配置仍开 DEBUG_PERF_ON (自动化 perf 套件读 [perf]); 是 trace 类标志在
    // Release 关掉, 所以 [perf] 计时不被 trace 的 stderr 写入污染。
#ifdef DEBUG_PERF_ON
    struct timespec t_start, t_end;
    clock_gettime(CLOCK_MONOTONIC, &t_start);
#endif
    // === end perf timing ===

    // 一次性 sigsetjmp 建立永久落点; 返回值不分流 (dummy.txt §1 机制 (3) "dispatcher 视角"段)。
    sigsetjmp(*hart->jmp_buf_ptr, 1);

    // 主循环条件: in_trap < 3 (hart 自身未 triple fault) AND system_reset_signal=1
    // (cross-hart / 全机停机协议未触发)。后者跟 main while 同一 flag, 让"任一 hart
    // 触发 system reset 时所有 hart 一起退" 成为单 hart 即可铺路的协议 (dummy.txt
    // §12 + runtime.h doc 段)。memory_order_relaxed 内层 hot path 不付 acquire 代价
    // (set 路径用 release, 见本文件函数末段 + main.c)。
    while (hart->trap.in_trap < 3 &&
           atomic_load_explicit(&system_reset_signal, memory_order_relaxed)) {

    // ========================================================================
    // 迭代头扫尾:
    //   - 累加上一轮 block 的 local_count 进 total_count
    //   - 重置 local_count, 准备下一轮 block 写入
    //
    // 必须放迭代头 (而不是迭代尾 block 3 之后): longjmp 跳回 setjmp 落点会跳过迭代尾,
    // 但跳到 while 顶后顺序进 while 体, 必经过迭代头; 跟正常 continue (mmu_translate_pc
    // fail 走 path 2b return rc → continue) 路径一样必经过迭代头。两条路径同形态扫尾。
    //
    // 未来 mtime 推进 / 中断检查 / perf_advance 等扫尾工作一并放这里, 不放迭代尾。
    // ========================================================================
    total_count += local_count;
    local_count = 0;

    // ========================================================================
    // 中断检查 (trap_check_interrupt; 详 dummy.txt §1 路径 2b' / trap.h doc 段)
    //
    // 必须在 DEBUG_REFETCH 之前: check 可能调 trap_set_interrupt_state 改 pc → xtvec,
    // 'f' trace 应反映"新 pc 即将 fetch", 不是"check 前的旧 pc". 中断 fire 路径 trace
    // 序列 = 't'/'s'/'e' (set_interrupt_state 打) → 'f' (本 DEBUG_REFETCH 打),
    // 跟 exception 路径 longjmp 跳回 sigsetjmp 落点后 trace = 'E' → 'f' 同形态.
    //
    // 返非 0 = 已 set_interrupt_state, dispatcher continue 跳回 while 顶 (走 dummy.txt
    // §9 路径 2b 浅栈 return-based, 不长跳 — 跟 mmu_translate_pc 失败路径同形态).
    // ========================================================================
    if (trap_check_interrupt(hart) != 0) continue;

    // 每轮 while 体进入 = 一次"重新派发取指" (块边界自然推进 / 跨页退块重派 / helper
    // longjmp 跳回 sigsetjmp 落点都走这里)。fixture 跨页 / 中断密度人工观察 (debug.h)。
    // trap_check_interrupt 之后才打 'f' = pc 已敲定 (互斥协议见 debug.h 顶段).
    DEBUG_REFETCH();

    // ========================================================================
    // pc IALIGN 兜底 (single source; 详 dummy.txt §9)
    //
    // 不管 pc 怎么来 (cpu_create / 上轮 block 出口 / sigsetjmp 跳回后的 xtvec /
    // 未来 reset_vector / mret/sret 写的 mepc/sepc / 中断 deliver 后 xtvec), fetch 前
    // 统一兜底。pc 不对齐时 regime / TLB / mmu_translate_pc 都没必要算 — 直接
    // trap_set_exception_state + continue 让 while 条件接管. dispatcher 主帧内, 走返回机
    // 制不长跳 (§1 路径 2b).
    //
    // tval = pc 自身 (跟 mmu_translate_pc fetch fault 写 tval = gva = pc 同形态)。
    // 转跳指令 (jal/jalr/branch/mret/sret) 内的 IALIGN 自检占位实际是 dead code
    // (IALIGN=16 + encoding/mask 强制对齐), 这里是 single source。
    // ========================================================================
    if ((hart->regs[0] & IALIGN_MASK) != 0u) {
        trap_set_exception_state(hart, CAUSE_INST_ADDR_MISALIGNED, /*tval*/hart->regs[0]);
        continue;
    }

    // ========================================================================
    // block 1: 算派发包 (regime, current_tlb)
    //
    // 派发包概念上是两件:
    //   regime       : 执行 regime, 决定 "用哪套 PTE 检查规则"
    //                    REGIME_BARE  = Trust  (M-mode 或任何 priv 带 bare satp)
    //                    REGIME_SV32  = Checked (S/U + Sv32)
    //   current_tlb  : 走 TLB 时用哪个叶 TLB
    //                    NULL          = Trust 不需要 TLB
    //                    非 NULL       = Sv32 用 hart->tlb_table[priv][asid] 选定的叶
    //
    // 接口层简化: regime 与 current_tlb 是否为 NULL 现阶段严格 1:1 一致 (NULL ↔ BARE,
    //   非 NULL ↔ SV32), 故下游 mmu_translate_pc / interpret_one_block 只吃 current_tlb
    //   即可 (NULL 编码 regime); 详见 mmu.h regime_t doc 段。
    //
    // 落地修订: dispatcher 内部仍把 regime 显式算出, 表达 "这是两个独立的派发概念, 现阶段
    //   恰好一致" + 未来 H 扩展真打破 1:1 时这里不需要重新引入变量。两个本地变量在同一
    //   if/else 内一同赋值, 没有 inconsistent state 风险。
    //
    // xatp 抽象层 (有意保留): 初版 = satp; 未来 H 扩展 V=1 时 = vsatp。
    //
    // U 副本语义在 cpu_create 阶段已按 misa 派发 (cpu.c [PRIV_U] 段; MSU 副本 [1] S,
    // MU-only 副本 [3] M=NULL)。dispatcher 这里只看 priv + xatp.mode 选 regime, 不需要
    // 再 misa 分支 — MU-only 时 U 副本 M=NULL, satp.MODE=bare 自然走 BARE 路径
    // current_tlb=NULL; satp.MODE=Sv32 时查 tlb_table[PRIV_U][...]=NULL 走懒分配 (但
    // MU-only 实际不会让 satp 写非 bare 值, csr 层应卡)。
    // ========================================================================
    uxlen_t  xatp = hart->satp;
    regime_t regime;
    tlb_t *current_tlb;

    if (hart->priv == PRIV_M || (xatp >> 31) == 0 /* satp.mode == bare */) {
        // Trust regime: bypass TLB
        regime      = REGIME_BARE;
        current_tlb = NULL;
    } else {
        // Checked regime: 走 [priv][asid]
        // satp.ASID 字段在 Sv32 中位于 bit 22..30; csr.c 的 satp 写 helper 已 WARL 截断到
        // ASID_MASK 位 (dummy.txt §3 satp 合法性契约), 所以这里直接索引安全。
        regime      = REGIME_SV32;
        uint32_t asid = (xatp >> 22) & ASID_MASK;
        current_tlb = hart->tlb_table[hart->priv][asid];
        // SV32 路径需 current_tlb 非 NULL 才走 walker; NULL 时走懒分配 (tlb_alloc 写回)。
        // tlb_alloc 失败 = host 内存耗尽 (host 错, 跟 guest 无关), 走内部异常硬停机
        // (in_trap bit 3 = 1; 见本文件末尾 in_trap 位段编码段)。
        if (current_tlb == NULL) {
            current_tlb = tlb_alloc();
            if (current_tlb == NULL) {
                fprintf(stderr,
                        "[dispatcher] tlb_alloc failed for priv=%u asid=%u\n",
                        (uint32_t)hart->priv, asid);
                hart->trap.in_trap |= 0x8;  // 内部异常硬停机 (in_trap bit 3)
                break;
            }
            hart->tlb_table[hart->priv][asid] = current_tlb;
        }
    }
    // regime 显式算出表达"两个独立的派发概念, 现阶段恰好一致, 未来 H 扩展真打破 1:1 时
    // 这里不重新引入变量" 的设计意图; 当前下游只吃 current_tlb (NULL 编码 regime),
    // regime 在本函数没有运行时消费者, 抑制 unused 警告。
    (void)regime;

    // ========================================================================
    // block 2: 取指路径 (mmu_translate_pc)
    //
    // dispatcher 内部得出 (regime, current_tlb) 两件; 下游只吃 current_tlb (NULL 编码
    // regime)。这是 mmu_translate_pc / interpret_one_block 的接口形态; JIT 一侧不同
    // (block 编译时 baked regime, jit_cache key = (PA, regime))。
    // ========================================================================
    uxlen_t  pa;
    uint8_t *hva;
    int rc = mmu_translate_pc(hart, current_tlb, &pa, &hva);
    (void)pa;       // 未来给 JIT 查 jit_cache 用, 当前 interpreter 不消费

    // ========================================================================
    // mmu_translate_pc 已在内部直调 trap_set_exception_state 设好 xcause/xtval/xepc/regs[0]=xtvec,
    // rc 是 in_trap 当前值 (0 / 非 0 状态)。非 0 → continue 让 while(in_trap < 3) 接管
    // (in_trap 已 ≥ 1, 第 3 次时退出 dispatcher)。
    // dummy.txt §1 路径 C (mmu fetch trap 不长跳)。
    // ========================================================================
    if (rc != 0) continue;

    // ========================================================================
    // block 3: 派发到 jit / interpreter
    //
    // 完整形态 (jit_cache 接入后启用; 当前直接走 interpreter 一路):
    //   if (jit_cache_hit) {
    //       jit_block(hart, current_tlb, &local_counter);  // jit_cache 注册的 host_code_ptr
    //   } else {
    //       counter[pc]++;                                  // 热度计数 hash
    //       if (达阈值) trigger_translate(...);              // 触发翻译 (后期改 thread 非阻塞)
    //       interpret_one_block(hart, current_tlb, hva, &local_counter);
    //                                                       // 解释器从 hva 直接读字节取指,
    //                                                       // 依赖块边界保证不跨 4K page
    //   }
    // 出块后所有扫尾搬到迭代头 (helper longjmp 跳回 sigsetjmp 落点会跳过迭代尾;
    // 只有迭代头才能保证一定执行)。
    //
    // 当前调用 (jit_cache / 热度计数未接): 直接 interpret_one_block, 块边界由解释器
    // 末尾 is_block_boundary_inst 自然产生 (branch/jal/jalr 命中后退出 fetch loop)。
    // ========================================================================
    /* local_count 跨 longjmp 持久 (volatile + sigsetjmp 之前声明), 累加 + reset 在迭代头
     * 处理 (block 1 之前); 这里只做 interpret_one_block 调用本身。(uint32_t *) cast 因
     * interpret_one_block 接口非 volatile 指针; 调用期间 volatile 语义不影响 (helper 内部
     * 本地操作), 调用前后 dispatcher 端 volatile 读写仍生效。 */
    interpret_one_block(hart, current_tlb, hva, (uint64_t *)&local_count);
    // perf_advance(hart, local_count);  // 占位, dispatcher 不消费; 真做时也搬迭代头

    }  /* while (in_trap < 3) */

    // === perf timing (DEBUG_PERF_ON gate) ===
    // 紧挨主循环后打 t_end。
#ifdef DEBUG_PERF_ON
    clock_gettime(CLOCK_MONOTONIC, &t_end);
#endif
    // === end perf timing ===

    // DEBUG trace 流尾部换行: dispatcher 在 while 体内通过 DEBUG_XXX 宏写入单字符流
    // (无换行)。while 退出 = trace 流到此为止, 立刻打一次 \n 收尾, 让后面的 [perf] /
    // [dispatcher] halted 各占干净一行。放 t_end 之后 (fputc 的 I/O 不算进 [perf] 计时
    // 窗口)。受 DEBUG_TRACE_ON gate (DEBUG_NEWLINE; trace 关时无字符流, 换行同步退化 no-op)。
    DEBUG_NEWLINE();

    // 退出循环后再做一次扫尾累加 — 最后一轮 block 的 local_count 还没进 total_count
    // (迭代头扫尾负责的是"上一轮", 最后一轮没下一轮迭代头来扫)。
    total_count += local_count;

    // === perf timing (DEBUG_PERF_ON gate; 直接 fprintf, 不走 debug 模块) ===
#ifdef DEBUG_PERF_ON
    {
        double perf_elapsed = (double)(t_end.tv_sec  - t_start.tv_sec)
                            + (double)(t_end.tv_nsec - t_start.tv_nsec) / 1e9;
        fprintf(stderr,
                "[perf] elapsed=%.6f s  total_count=%" PRIu64 "  MIPS=%.3f\n",
                perf_elapsed, total_count,
                perf_elapsed > 0.0
                    ? (double)total_count / perf_elapsed / 1e6
                    : 0.0);
    }
#endif
    // === end perf timing ===

    fprintf(stderr,
            "[dispatcher] halted: in_trap=%u total_count=%" PRIu64 " pc=0x%08" PRIx32 "\n",
            hart->trap.in_trap, total_count, hart->regs[0]);

    // ========================================================================
    // in_trap 位段编码语义 (host 端协议, 多种信号可叠加)
    //
    //   bit 0-1  (值 0..3)   : 实际 trap 嵌套深度
    //                            0/1/2 = 普通 trap nesting
    //                            3     = triple fault (项目内部停机协议; 跟 RV/SiFive 风格一致)
    //   bit 2    (值 4..7)   : 留白, 防止 trap 嵌套位将来扩展时跟下面位段冲突
    //   bit 3    (值 8..15)  : 内部异常 / 内部正常 (host 端协议, 不是 RV trap)
    //                            tlb_alloc fail (host 内存耗尽) 设此位 → halt
    //                            未来 MMIO sifive_test finisher 触发的"正常停机"也走这条
    //   bit 4    (值 16..31) : 留白
    //   bit 5+   (值 32+)    : 未来停机扩展 (设计预留)
    //
    // 设计哲学 (位表示叠加, 同时传多种信息):
    //   - 解释器 / JIT 内部只看 bit 0-1 (trap nesting 视角); 看到 in_trap < 3 即继续
    //   - 高位 (bit 3+) 仅由 dispatcher 写, 解释器 / JIT 不碰
    //   - while (in_trap < 3) 的判断作为安全闸: 高位一被设值就 ≥ 8 > 3, while 自动失败,
    //     dispatcher 退出 → main 端 dump 状态 (未来 reset 由 dispatcher 自己处理)
    //   - reset 路径只清 bit 0-1; 高位 bit 3+ 一旦设, reset 流程显式按停机类型决定是否清
    //
    // 注: 这是项目自定义 host-side 协议; 跟 RV Smdbltrp 扩展 (riscv.h CAUSE_DOUBLE_TRAP=16,
    // 硬件检查 mstatus.MDT 字段触发 cause=16) 无关。项目当前不实现 Smdbltrp; 未来若实现,
    // trap delivery 路径会按 spec 走 cause=16, 跟这里的 in_trap 位段协议是两条独立机制
    // (会并存但语义不重叠 — Smdbltrp 是规范层 trap 投递机制, in_trap 是 host emulator
    // 退出协议)。
    // ========================================================================
    // 未来 reset 扩展占位
    //
    // reset 是 per-hart 级 (跟 SMP-ready 一致, 跟 RV Smdbltrp 扩展 + SiFive 等 commercial
    // cores 的实际行为一致): 单 hart reset 不影响其他 hart, 跟 x86 triple fault 整 CPU
    // reset 不同。
    //
    // 伪码:
    //   if (misa 支持 reset 扩展 && hart->trap.in_trap == 3) {
    //       cpu_reset(hart);     // cpu.c 加, 跟 cpu_create / cpu_destroy 接口对称;
    //                             // 重置 regs[1..31] / pc=reset_vector / mstatus /
    //                             // xtvec/xepc/xcause/xtval / in_trap=0; 保留 hartid
    //                             // 等"硬件标识"字段 (真 hardware reset 后 hartid 不变)
    //       siglongjmp(*hart->jmp_buf_ptr, 1);  // 重入 dispatcher 入口的 sigsetjmp 落点 +
    //                                            // while 顶判 (in_trap=0 < 3 true 进 while)
    //                                            // 复用现有控制流原语, 不需要新协议
    //   }
    //   // else: dispatcher 退出, main 端处理 (高位 bit 3+ 已设走硬停机; misa 不支持
    //   //        reset 也走 main exit)
    //
    // 规范: cpu 的分配 + 初始化 + 销毁都在 dispatcher 之外 (cpu_create / cpu_destroy 由 main
    // 调用)。cpu_reset 是状态重置 (不是 alloc/destroy), 是规范的合法例外, 允许由 dispatcher
    // 调用; 它在 cpu.c 实现, 跟 cpu_create / cpu_destroy 接口对称。
    // ========================================================================

    // ========================================================================
    // 通知 main while 退出: dispatcher 退出 (in_trap >= 3 tri-fault 或其他 break
    // 路径) 后 set SRS=0。release-store 让 main 的 acquire-load 跟随看到。
    //
    // 跟上部 "未来 reset 扩展占位" 不冲突 — 未来真做 per-hart reset 时:
    //   - hart 参与 reset 路径: 不 set SRS, 走 cpu_reset + siglongjmp 重入 while
    //   - hart 不参与 reset / 走全机停机路径: set SRS, main while 退
    // 当前简化下 dispatcher 退出 = 全机停机, 一律 set。未来真分流时本行加判断。
    //
    // SDS 不在这里 set — dispatcher tri-fault 不一定意味全机 shutdown (例如未来
    // user 触发的 SR-only reset); SDS 由 main 在 while 退出后 cleanup 段 set 0
    // (谁 spawn 谁 join 协议见 dummy.txt §12 + runtime.h "SDS 蕴含 SRS" 段)。
    // ========================================================================
    atomic_store_explicit(&system_reset_signal, 0, memory_order_release);
}
