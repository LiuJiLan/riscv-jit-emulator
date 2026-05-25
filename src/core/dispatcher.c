//
// Created by liujilan on 2026/4/28.
// dispatcher 实现 (block 1+2+3 调度; sigsetjmp 永久落点 + while(SRS==0) 多块循环)。
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
#include "runtime.h"    // system_reset_signal (主循环 check + HART_MDT 兜底)

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
// 形态: sigsetjmp 一次性 + while(system_reset_signal == 0) 多块循环。
//
//   - sigsetjmp(*hart->jmp_buf_ptr, 1) 在 dispatcher 入口一次性建立永久落点 (见 dummy.txt §1
//     机制 (1) "为什么 sigsetjmp 在 while 外"段)。落点同时承接两种路径:
//       (i)  初次进入 dispatcher (sigsetjmp 返回 0, 顺序到 while 顶判条件)
//       (ii) helper longjmp 跳来 (siglongjmp 返回非 0, 控制流到 sigsetjmp 落点, 顺序到
//            while 顶重新判 SRS; trap_set_exception_state 内已设 hart->regs[0]=xtvec, 自然
//            从 trap handler 继续)
//     sigsetjmp 返回值不被分流 — longjmp 不携带"trap 错误"语义, 只是无条件控制流原语。
//
//   - 退出条件: system_reset_signal 非 0 (任一 bit 设了即退; bitmap 编码见 runtime.h)。
//     M-mode double trap (mstatus.MDT=1 时再来 trap) 在 trap.c 内部检查到, 自己 set
//     SYSRESET_BIT_HART_MDT — dispatcher 不主动写 SRS。a_03_session_011 起 in_trap
//     字段废除, 改走 spec-defined MDT/SDT (Smdbltrp/Ssdbltrp); 历史段见本文件末尾。
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

    // 主循环条件: system_reset_signal == 0 (允许执行); 任一 bit (HART_MDT /
    // TEST_RESET / SHUTDOWN_TRIGGER / DEVICE_FAIL) 设了都退出 (bitmap 极性 0=允许 /
    // 非0=触发, 详 runtime.h)。跟 main while 同一 bitmap, 让"任一 hart 触发
    // system reset 时所有 hart 一起退" 成为单 hart 即可铺路的协议 (dummy.txt §12 +
    // runtime.h doc)。memory_order_relaxed 内层 hot path 不付 acquire 代价 (set
    // 路径用 release, 见 runtime.c set_bit / main.c try_clear)。
    //
    // a_03_session_011 起: in_trap 字段废除, while 条件不再含 in_trap < 3; M-mode
    // critical-error (mstatus.MDT=1 时 trap_set_*_state 检) 在 trap.c 内自己
    // set HART_MDT, dispatcher 退出原因统一通过 system_reset_signal 表达。
    while (atomic_load_explicit(&system_reset_signal, memory_order_relaxed) == 0u) {

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
        // tlb_alloc 失败 = host 内存耗尽 (host 错, 跟 guest 无关), 走 hart-internal
        // hard halt — 视同 M-mode double fault, set SYSRESET_BIT_HART_MDT 让 main 走
        // ABORT_MASK 路径 cleanup return 非 0。in_trap bit 3 兜底位段已废除, 直接
        // 出 dispatcher。
        if (current_tlb == NULL) {
            current_tlb = tlb_alloc();
            if (current_tlb == NULL) {
                fprintf(stderr,
                        "[dispatcher] tlb_alloc failed for priv=%u asid=%u\n",
                        (uint32_t)hart->priv, asid);
                system_reset_signal_set_bit(SYSRESET_BIT_HART_MDT);
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
    // rc 是 trap_set_exception_state 返值 (0 / 非 0)。非 0 → continue 让 while(SRS==0)
    // 接管 (trap 已 deliver 或 M-mode critical-error 已 set HART_MDT, while 退出)。
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

    }  /* while (system_reset_signal == 0) */

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
            "[dispatcher] halted: mstatus.MDT=%u sstatus.SDT=%u total_count=%" PRIu64 " pc=0x%08" PRIx32 "\n",
            (uint32_t)((hart->trap._mstatus & MSTATUS_MDT_BIT64) != 0u),
            (uint32_t)((hart->trap._mstatus & (uint64_t)MSTATUS_SDT) != 0u),
            total_count, hart->regs[0]);

    // ========================================================================
    // 历史: in_trap 字段 → SRS bitmap → spec MDT/SDT
    //
    // 最先 (a_02 末 / a_03 初): 提出 cpu_t.trap.in_trap (uint8_t) 字段是为了
    // 跟踪 trap 嵌套深度 — 名字 "in_trap" 即由此而来。位段最初设计:
    //   bit 0-1  实际 trap 嵌套深度 (0/1/2 普通 nesting; 3 = triple fault)
    //
    // 方案落实时顺手扩展: 同一字段还有冗余高位 (bit 3+, 值 ≥ 8 远大于 3,
    // dispatcher while(in_trap < 3) 自动失败退出), 顺便用作内部停机位段 —
    // 一字段兼两用:
    //   bit 2    留白 (跟下面停机段拉开间距)
    //   bit 3    内部异常硬停机 (tlb_alloc fail 等 host 错; 跟 RV trap 无关)
    //   bit 4+   留白 / 未来扩展
    // mret/sret 路径只清 bit 0-1; 高位停机标记一旦设, while 自然退出。
    //
    // 当时 (in_trap 字段一字两用方案确定时) 设想: sifive_test 兼容设备触发
    // 整机停机, 可以直接写 cpu_t.trap.in_trap 高位完成 — 单核下不需要 atomic,
    // 写法简单。但同时也意识到, 真到多核时跨 hart 写 in_trap 就必须用 atomic
    // 变量。项目当时还是单核, 这条想法就搁置了。
    //
    // 后来 CLINT 实装时, 引入 runtime SRS / SDS 信号 (早期 _Atomic int 形态)。
    // CLINT timer 是真多线程异步源 (跨 hart 写 mtime, 主帧读), 必须走 atomic;
    // 顺便把"sifive_test 触发整机停机" 这条早期设想接上 SRS — 跨设备停机通道
    // 终于真做出来, 但走的是新加的 SRS 变量, 不是 in_trap 高位。
    //
    // in_trap 字段从这时起定位转为"为未来 double trap 服务" — 把 spec MDT/SDT
    // 的"上次 trap 没退又来一次 = 不可恢复" 语义用 host 计数器近似 (候选 A 早
    // return 不 deliver)。这套机制本质是不实装 Smdbltrp / Ssdbltrp 扩展时的
    // 简化兜底。
    //
    // a_03_session_011 真做 Smdbltrp / Ssdbltrp 时, 根据手册发现 in_trap 字段
    // 本身没必要 — spec MDT 在 mstatus, SDT 在 sstatus, trap entry 检字段 +
    // 没退就是 unexpected, 跟踪效果跟 in_trap 计数器等价。同时 SRS 已升级为
    // 32-bit bitmap (详 runtime.h)。in_trap 字段彻底清, M-mode critical-error
    // 走 system_reset_signal SYSRESET_BIT_HART_MDT, 跟 ABORT_MASK 通道统一。
    //
    // 现在 (a_03_session_011 起): in_trap 字段废除, 改走 spec-defined 路径:
    //   - S-trap entry 检查 sstatus.SDT — SDT=1 时升级 deliver_priv=M,
    //     cause=CAUSE_DOUBLE_TRAP (Ssdbltrp §4.1.1.5 unexpected trap)
    //   - M-trap entry 检查 mstatus.MDT — MDT=1 时不 deliver, hart 进 critical-
    //     error state, set system_reset_signal SYSRESET_BIT_HART_MDT 通知 main
    //     (Smdbltrp §3.1.6.2 unexpected trap; 项目不实装 NMI, MDT 无 RNMI handler
    //     接管, 必走 critical-error)
    //   - mret 清 MDT=0 (按 spec §3.1.6.2); sret 清 SDT=0 (按 spec §4.1.1.5)
    //
    // ------------------------------------------------------------------------
    // 为什么 MDT critical-error 选 "abort 整个模拟器" 而不是 per-hart restart
    //
    // spec §3.1.6.2 末段: "The actions performed by the platform when a hart
    // asserts a critical-error signal are platform-specific. The range of
    // possible actions include restarting the affected hart or restarting the
    // entire platform, among others." — 平台自由选择, 重启单 hart 或整机皆合规。
    //
    // 项目选 "整机 abort + cleanup chain return 非 0", 理由:
    //   1. 跟 QEMU 实践一致 — QEMU 不实装 NMI 时 MDT critical-error 走 abort,
    //      减少对照行为差异; 简化基准对照测试。
    //   2. 方便模拟器停机 — 走现有 ABORT_MASK 通道 (SYSRESET_BIT_HART_MDT 在
    //      mask 内), main 端统一 cleanup + return exit_code, 跟 sifive_test
    //      PASS/FAIL / 设备 fail 出口完全同路径; 不需要新增 "per-hart restart"
    //      生命周期 (那是真 SMP 的事, 单 hart 下 restart 整 hart ≡ restart 整机)。
    //
    // 未来真 SMP 落地时, 可以为 SYSRESET 新增一个 "per-hart MDT" bit (放
    // ABORT_MASK 外, 跟 TEST_RESET 同 reset-allowed 体例), 单 hart MDT 走 hart
    // 局部 reset 不影响别的 hart, 跟 spec "可重启单 hart" 选项对应。当前单 hart
    // 简化下两条路径等价。
    // ========================================================================
    // 底下这个 if 留着 (而不是删) 的理由:
    //   spec §3.1.6.2 末段说 platform 在 hart critical-error 时的行为可以是
    //   "restarting the affected hart" (重启单 hart, 不影响别的 hart) 或
    //   "restarting the entire platform" (重启整机)。HART 在手册里是有资格被
    //   单独 reset 的, 只是项目当前没实现这条路径 (单 hart 简化下 per-hart
    //   reset ≡ 整机 reset)。
    //   未来真做 SMP + per-hart reset 时, 这个 if 是入口 — 按 hart-internal 错
    //   类型分流: 跟整机有关的设 ABORT bit, 仅本 hart 的设新 "per-hart MDT" bit
    //   (放 ABORT_MASK 外, 走 reset continue 路径)。
    //
    // 当前 condition 0 = dead branch (in_trap 字段废除, MDT critical-error 由 trap.c
    // 自己 set HART_MDT, tlb_alloc fail 路径也自己 set, 不需要 dispatcher 末段补刀;
    // 整段保留是给未来 per-hart reset 触发改 condition 用)。
    //
    // /* 未来实现某种方法转跳到这里, 或者, 其他方法调用 cpu_reset, 甚至在外面
    //    由 main 来 reset 单核; 三条路径都让 hart 单独重启不影响别的 hart */
    // if (M-mode Double Fault) {
    //     仅 reset 本 hart();
    // }
    if (0) {
        system_reset_signal_set_bit(SYSRESET_BIT_HART_MDT);
    }
}
