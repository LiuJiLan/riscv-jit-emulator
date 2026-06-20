//
// Created by liujilan on 2026/4/28.
// dispatcher 实现 (block 1+2+3 调度; sigsetjmp 永久落点 + while(SRS==0) 多块循环)。
// 跨文件协议见 src/dummy.txt §1 (sigsetjmp) / §4 (TLB 分发机制)。
//

#include "dispatcher.h"

#include "api/jit_api.h"   // jit_compile_block / jit_status_t (fork 点 miss 路径调)
#include "config.h"
#include "cpu.h"
#include "debug.h"      // DEBUG_REFETCH (块边界 / 跨页 / longjmp 回 sigsetjmp 落点都触发)
#include "interpreter.h"
#include "jit/backend.h"   // jit_block_func_t cast (fork 点 hit 路径调 host_code)
#include "jit/jit_cache.h" // jit_cache_lookup / jit_rcu_read_lock/unlock + entry struct
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
//     SYSRESET_BIT_HART_MDT — dispatcher 不主动写 SRS。Double Trap 走 spec-defined
//     MDT/SDT (Smdbltrp/Ssdbltrp); 详本文件末尾段。
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
    // while 条件不含 in_trap (无独立 in_trap 字段); M-mode critical-error
    // (mstatus.MDT=1 时 trap_set_*_state 检) 在 trap.c 内自己 set HART_MDT,
    // dispatcher 退出原因统一通过 system_reset_signal 表达。
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

    // JIT 状态灯兜底清 (§11 (a) 候选 c).
    //   语义同上"迭代头必经"协议 — host_code 内 helper 撞 trap_raise_exception
    //   longjmp 时跳过 host_code 后续清状态灯, 由本处兜底清; 正常路径 (hit 调
    //   host_code 跑完 + 块 3 显式清) 是冗余清 NULL release, 重复清不撞 race.
    //   字段从 cpu_create init 起就是 NULL, hot path (正常 iteration 不跑 JIT 块)
    //   也是清 NULL → NULL, 单次 atomic store release ~ns 量级开销, 不显著。
    atomic_store_explicit(&hart->jit_executing_host_code, NULL,
                          memory_order_release);

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
    if (trap_check_interrupt(hart) != 0) { continue; }

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
    //   regime       : 执行 regime + priv 视角, 决定 "用哪套 PTE 检查规则" (b_01 起 3 状态)
    //                    REGIME_BARE   = Trust  (M-mode 或任何 priv 带 bare satp)
    //                    REGIME_SV32_S = Checked S (priv=S + Sv32; PTE_U=0 默认; SUM/MXR 联动)
    //                    REGIME_SV32_U = Checked U (priv=U + Sv32; PTE_U=1 强制)
    //   current_tlb  : 走 TLB 时用哪个叶 TLB
    //                    NULL          = Trust 不需要 TLB
    //                    非 NULL       = Sv32 用 hart->tlb_table[priv][asid] 选定的叶
    //
    // 接口层 by design 拍法分裂 (详 mmu.h 顶段 "执行 regime" 段):
    //   interpreter 路径 (mmu_translate_pc / interpret_one_block): 只吃 current_tlb,
    //     NULL 编码 BARE / 非 NULL 编码 SV32_S 或 SV32_U; interpreter case 内按
    //     cpu->priv 分支判 PTE_U / SUM/MXR。
    //   JIT 路径 (jit_cache key = (PA, regime); fork 点真做): 真显式吃
    //     regime_t enum; (PA, SV32_S) 跟 (PA, SV32_U) 是两个独立槽; 块体编译时
    //     baked priv 视角的 PTE_U / SUM/MXR check。
    //
    // 落地修订: dispatcher 内部仍把 regime 显式算出 (3 状态), 即使当前 interpreter 接口只
    //   消费 current_tlb —— 是为 T4 jit_cache fork 点接入时不需要重新引入变量。
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
        // Checked regime: priv == S → REGIME_SV32_S; priv == U → REGIME_SV32_U;
        // 两 regime 共享 current_tlb 算法 (走 [priv][asid] 选叶); regime 三元区分用于
        // JIT 一侧 baked priv (interpreter 路径仍只看 current_tlb, 不感知 S/U)。
        // satp.ASID 字段在 Sv32 中位于 bit 22..30; csr.c 的 satp 写 helper 已 WARL 截断到
        // ASID_MASK 位 (dummy.txt §3 satp 合法性契约), 所以这里直接索引安全。
        regime      = (hart->priv == PRIV_S) ? REGIME_SV32_S : REGIME_SV32_U;
        uint32_t asid = (xatp >> 22) & ASID_MASK;
        current_tlb = hart->tlb_table[hart->priv][asid];
        // SV32 路径需 current_tlb 非 NULL 才走 walker; NULL 时走懒分配 (tlb_alloc 写回)。
        // tlb_alloc 失败 = host 内存耗尽 (host 错, 跟 guest 无关), 走 hart-internal
        // hard halt — 视同 M-mode double fault, set SYSRESET_BIT_HART_MDT 让 main 走
        // ABORT_MASK 路径 cleanup return 非 0, 直接出 dispatcher。
        if (current_tlb == NULL) {
            current_tlb = tlb_alloc();
            if (current_tlb == NULL) {
                fprintf(stderr,
                        "[hart%u dispatcher] tlb_alloc failed for priv=%u asid=%u" EOL,
                        hart->hartid, (uint32_t)hart->priv, asid);
                system_reset_signal_set_bit(SYSRESET_BIT_HART_MDT);
                break;
            }
            hart->tlb_table[hart->priv][asid] = current_tlb;
        }
    }
    // regime 显式算出 (3 状态: BARE / SV32_S / SV32_U); interpreter 路径下游只吃
    // current_tlb (NULL 编码 BARE, 非 NULL 编码 SV32_S/_U; interpreter case 内按
    // cpu->priv 分支 S/U). JIT 一侧 fork 点 (block 3) 真消费 regime 作 jit_cache
    // key 第二维.

    // ========================================================================
    // block 2: 取指路径 (mmu_translate_pc)
    //
    // dispatcher 内部得出 (regime, current_tlb) 两件; 下游 (interpreter 路径) 只吃
    // current_tlb (NULL = BARE; 非 NULL = SV32_S 或 SV32_U, interpreter case 内按
    // cpu->priv 分支)。这是 mmu_translate_pc / interpret_one_block 的接口形态;
    // JIT 一侧不同 (block 编译时 baked regime, jit_cache key = (PA, regime),
    // (PA, SV32_S) 跟 (PA, SV32_U) 是两个独立槽)。
    // ========================================================================
    uxlen_t  pa;
    uint8_t *hva;
    int rc = mmu_translate_pc(hart, current_tlb, &pa, &hva);
    // pa: block 3 fork 点真消费 (jit_cache key 第一维); hva: interpreter 路径取指起点

    // ========================================================================
    // mmu_translate_pc 已在内部直调 trap_set_exception_state 设好 xcause/xtval/xepc/regs[0]=xtvec,
    // rc 是 trap_set_exception_state 返值 (0 / 非 0)。非 0 → continue 让 while(SRS==0)
    // 接管 (trap 已 deliver 或 M-mode critical-error 已 set HART_MDT, while 退出)。
    // dummy.txt §1 路径 C (mmu fetch trap 不长跳)。
    // ========================================================================
    if (rc != 0) { continue; }

    // ========================================================================
    // block 3: 派发到 jit / interpreter (fork 点; b_02 T5 接 counter + invalidate)
    //
    // 协议 (jit_framework_overview §3 (a) caller 视角 + §11 (a) 候选 c 状态灯方案;
    //       b_02 T5 加 heat counter 真消费):
    //   1. jit_rcu_read_lock(hartid) — 进 RCU read critical section
    //   2. jit_cache_lookup_or_init(pa, regime) → entry 或 NULL
    //      - miss EMPTY 时内部 CAS install COUNTING entry counter=0
    //      - 命中 任意非 EMPTY status (COMPILED/COUNTING/BLACK) 返 entry
    //      - 探测耗尽 (hash table 满) 返 NULL
    //   3. entry 4 状态分流:
    //      - COMPILED → 取 host_code + 点亮 cpu->jit_executing_host_code 状态灯,
    //                   出 RCU 后调 host_code; 跑完清状态灯 (longjmp 路径迭代头兜底清)
    //      - COUNTING → atomic_fetch_add(&counter, 1); old+1 == COMPILE_THRESHOLD
    //                   时 needs_compile=1, 出 RCU 后调 jit_compile_block; 本次仍走
    //                   interpret 兜底 (编译后下次 lookup_or_init hit COMPILED)
    //      - BLACK   → interpret 不增 counter (永不编)
    //      - NULL (cache 满) → interpret 兜底 (后续 jit_flush_all 路径自然走通)
    //   4. jit_rcu_read_unlock(hartid) — 立即退 RCU (避免延伸到 host_code 执行期撞
    //                   longjmp 不安全)
    //   5. needs_compile 时 jit_compile_block — promote COUNTING → COMPILED 或失败
    //                   set_blacklist (Q11 a 组合层处理 Flush+retry)
    //
    // local_count 跨 longjmp 持久 (volatile + sigsetjmp 之前声明), 累加 + reset 在
    // 迭代头处理 (block 1 之前); 这里只做 block 执行调用. (uint64_t *) cast 因
    // interpret_one_block / jit_block_func_t 接口非 volatile 指针; 调用期间 volatile
    // 语义不影响 (helper 内部本地操作), 调用前后 dispatcher 端 volatile 读写仍生效.
    // ========================================================================
    jit_rcu_read_lock(hart->hartid);
    jit_cache_entry_t *e = jit_cache_lookup_or_init(pa, regime);
    void *host_code = NULL;
    uint32_t needs_compile = 0u;
    if (e != NULL) {
        uint32_t s = atomic_load_explicit(&e->status, memory_order_acquire);
        if (s == (uint32_t)JIT_CACHE_COMPILED) {
            host_code = e->host_code_ptr;
            atomic_store_explicit(&hart->jit_executing_host_code, host_code,
                                  memory_order_release);
        } else if (s == (uint32_t)JIT_CACHE_COUNTING) {
            uint32_t old = atomic_fetch_add_explicit(&e->counter, 1u,
                                                    memory_order_relaxed);
            /* old+1 == THRESHOLD: 恰好一个 hart 触发 compile (atomic fetch_add
             * 保证 RMW 顺序唯一性, 后续 hart fetch_add 拿 old >= THRESHOLD 不再
             * 重触发). 用 == 而非 >= 节省 — compile 完成 entry 切 COMPILED,
             * 下次 lookup_or_init 直接 hit COMPILED 走 host_code. */
            if (old + 1u == COMPILE_THRESHOLD) {
                needs_compile = 1u;
            }
        }
        /* BLACK 不处理 — 走 interpret 兜底 (host_code 留 NULL) */
    }
    jit_rcu_read_unlock(hart->hartid);

    if (host_code != NULL) {
        DEBUG_JIT_HIT();   /* 'J' trace — JIT 路径走通的功能信号 (Q5=a) */
        jit_block_func_t fn = (jit_block_func_t)host_code;
        fn(hart, current_tlb, (uint64_t *)&local_count);
        atomic_store_explicit(&hart->jit_executing_host_code, NULL,
                              memory_order_release);
    } else {
        if (needs_compile) {
            /* jit_compile_block 内部 Q11 a 组合层: 成功 install promote COUNTING →
             * COMPILED; 失败 (NOT_IMPLEMENTED / 二次 FULL) set_blacklist → BLACK.
             * 返码本身 dispatcher 不消费 (interpret 兜底逻辑相同), void cast. */
            (void)jit_compile_block(pa, regime);
        }
        interpret_one_block(hart, current_tlb, hva, (uint64_t *)&local_count);
    }
    // perf_advance(hart, local_count);  // 占位, dispatcher 不消费; 真做时也搬迭代头

    }  /* while (system_reset_signal == 0) */

    // === perf timing (DEBUG_PERF_ON gate) ===
    // 紧挨主循环后打 t_end。
#ifdef DEBUG_PERF_ON
    clock_gettime(CLOCK_MONOTONIC, &t_end);
#endif
    // === end perf timing ===

    // DEBUG trace flush: dispatcher 在 while 体内通过 DEBUG_XXX 宏 append 字符到
    // per-hart trace_buf (__thread)。while 退出 = trace 流到此为止, 立刻 flush 整个
    // buffer 到 stderr + EOL 收尾, 让后面的 [perf] / [dispatcher] halted 各占干净
    // 一行。放 t_end 之后 (fwrite 的 I/O 不算进 [perf] 计时窗口)。受 DEBUG_TRACE_ON
    // gate (DEBUG_NEWLINE; trace 关时退化 no-op)。
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
                "[hart%u perf] elapsed=%.6f s  total_count=%" PRIu64 "  MIPS=%.3f" EOL,
                hart->hartid, perf_elapsed, total_count,
                perf_elapsed > 0.0
                    ? (double)total_count / perf_elapsed / 1e6
                    : 0.0);
    }
#endif
    // === end perf timing ===

    fprintf(stderr,
            "[hart%u halted] mstatus.MDT=%u sstatus.SDT=%u total_count=%" PRIu64 " pc=0x%08" PRIx32 "" EOL,
            hart->hartid,
            (uint32_t)((hart->trap._mstatus & MSTATUS_MDT_BIT64) != 0u),
            (uint32_t)((hart->trap._mstatus & (uint64_t)MSTATUS_SDT) != 0u),
            total_count, hart->regs[0]);
    fputs(EOL, stderr);   /* 大块尾部空行: 让 [perf] / [halted] 跟下一大块视觉分开 */

    // ========================================================================
    // M-mode critical-error (Smdbltrp) → 整机 abort, 不 per-hart restart
    //
    // Double Trap 走 spec-defined MDT/SDT (无独立 in_trap 嵌套计数字段):
    //   - S-trap entry 检 sstatus.SDT — SDT=1 升级 deliver_priv=M,
    //     cause=CAUSE_DOUBLE_TRAP (Ssdbltrp §4.1.1.5 unexpected trap)
    //   - M-trap entry 检 mstatus.MDT — MDT=1 不 deliver, hart 进 critical-error,
    //     set SYSRESET_BIT_HART_MDT 通知 main (Smdbltrp §3.1.6.2; 项目不实装 NMI,
    //     无 RNMI handler 接管, 必走 critical-error)
    //   - mret 清 MDT=0 (spec §3.1.6.2); sret 清 SDT=0 (spec §4.1.1.5)
    //
    // spec §3.1.6.2 末段: critical-error 时 platform 行为自由 ("restarting the
    // affected hart or restarting the entire platform, among others")。项目选
    // "整机 abort + cleanup chain return 非 0":
    //   1. 跟 QEMU 实践一致 (不实装 NMI 时 MDT critical-error 走 abort), 减少
    //      对照行为差异。
    //   2. 走现有 ABORT_MASK 通道 (SYSRESET_BIT_HART_MDT 在 mask 内), main 统一
    //      cleanup + return exit_code, 跟 sifive_test PASS/FAIL / 设备 fail 出口
    //      同路径; 不需要新增 per-hart restart 生命周期 (单 hart 下 restart 整
    //      hart ≡ restart 整机)。
    //
    // 底下 if(0) 保留 (不删) 的理由: 未来真 SMP + per-hart reset 时这是入口 —
    // 按 hart-internal 错类型分流, 仅本 hart 的设新 "per-hart MDT" bit (放
    // ABORT_MASK 外走 reset continue 路径), 对应 spec "可重启单 hart" 选项。
    // 当前 condition 0 = dead branch (MDT critical-error 由 trap.c 自己 set
    // HART_MDT, tlb_alloc fail 路径也自己 set, dispatcher 末段不需补刀)。
    // ========================================================================
    if (0) {
        system_reset_signal_set_bit(SYSRESET_BIT_HART_MDT);
    }
}
