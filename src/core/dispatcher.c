//
// Created by liujilan on 2026/4/28.
// dispatcher 实现 (block 1+2+3 调度; a_01_5_b: while(in_trap < 3) + mmu rc continue)。
// 跨文件协议见 src/dummy.txt §1 (sigsetjmp) / §4 (TLB 分发机制)。
//

#include "dispatcher.h"

#include "config.h"
#include "cpu.h"
#include "interpreter.h"
#include "mmu.h"
#include "tlb.h"
#include "riscv.h"

#include <inttypes.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>


// ============================================================================
// dispatcher 的作用 (设计文档)
// ============================================================================
//
// I. 完成 block 级别的 pc → PA / HVA 翻译 (block 入口包准备)
//    - PA 给 JIT hash (jit_cache 用 PA 为 key 找 host_code_ptr)
//    - HVA 给解释器直接读字节取指 (依赖块边界保证不跨 4K page)
//
// II. 在 helper longjmp 跳回时承接控制流 (sigsetjmp landing, a_01_5_c 真激活)
//
// III. 迭代头 perf 同步 / mtime 推进 / 中断检查 (a_03 真接入)
//
// a_01_5_c 当前形态: sigsetjmp 一次性 + while(hart->trap.in_trap < 3) 多块循环。
//   - sigsetjmp(*hart->jmp_buf_ptr, 1) 在 dispatcher 入口一次性建立永久落点 (见 dummy.txt §1
//     机制 (1) "为什么 sigsetjmp 在 while 外"段)。落点同时承接两种路径:
//       (i)  初次进入 dispatcher (sigsetjmp 返回 0, 顺序到 while 顶判条件)
//       (ii) helper longjmp 跳来 (siglongjmp 返回非 0, 控制流到 sigsetjmp 落点, 顺序到
//            while 顶重新判 in_trap; trap_set_state 内已设 hart->regs[0]=xtvec, 自然
//            从 trap handler 继续)
//     sigsetjmp 返回值不被分流 — longjmp 不携带"trap 错误"语义, 只是无条件控制流原语。
//   - 退出条件: hart->trap.in_trap >= 3 (triple fault, 项目内部停机协议)。
//     trap_set_state 内 in_trap >= 3 时早 return 不 deliver (候选 A); main 端拿回控制后
//     fprintf 表 halt + 未来 reset 接入。
//   - mmu_translate_pc 路径 (dummy.txt §1 路径 C, 不长跳): rc != 0 → continue, 让 while
//     条件接管。
//   - total_count 必须 volatile (跨 longjmp 不被编译器放 callee-saved 寄存器丢值, 见
//     dummy.txt §1 末段)。声明 + 初始化要在 sigsetjmp 调用之前, 否则 longjmp 跳回时会
//     重新执行初始化, 累计丢失。
//
//   - count 同步契约 (a_01_10 (7) step 1 加, 跟 JIT prologue/epilogue 哲学一致):
//     dispatcher 持 local_count 接 interpret_one_block 出参 count_out; interpreter 内
//     count 是栈局部变量, longjmp 跳走时 *count_out = count 必须已写过, 否则 dispatcher
//     收到 local_count = 0 (初始值), total_count 累加 0 = 丢一个 block 的 count。
//
//     interpreter 端协议 (interpret_one_block 内 SYNC_COUNT 宏 doc + switch 顶部协议段):
//       (a) may-trap 路径: case 内 SYNC_COUNT() 在 trap_raise / helper-call 之前
//       (b) boundary 路径: case 末 break + fetch loop 末段 boundary check → goto out →
//                          out 段 SYNC_COUNT() 托底 (csr/sfence/mret/branch/jal/jalr
//                          走这条 — decode.h is_block_boundary_inst 列表)
//       (c) pure case: 不需要同步 (走完 fetch loop 末段 count++ 后下轮 while 继续;
//                       count 直到 (b) boundary out 才传出)
//
//     dispatcher 端 (session_002 D1 项 4 关键: 扫尾搬迭代头):
//       (1) local_count 必须 volatile + 提到 sigsetjmp 之前声明 — 跟 total_count 同
//           处理, 跨 longjmp 持久 (longjmp 不动栈, sigsetjmp 之前声明的栈上变量值保留)
//       (2) 累加 + reset 搬到迭代头 (while 体顶, block 1 之前):
//             total_count += local_count;
//             local_count = 0;
//           不能放迭代尾 (block 3 之后) — longjmp 跳回 setjmp 落点会跳过迭代尾,
//           上一轮 SYNC_COUNT 写好的 local_count 会被下一轮 reset 覆盖, 总值丢失
//       (3) longjmp 路径跟正常 continue 路径都经过迭代头, 两条路径同形态扫尾
//
//     RV precise trap 语义对齐: trap 触发指令本身不算入 count_out (count++ 在 case 末,
//     但 SYNC_COUNT 在 trap_raise 之前; 所以 trap 那条 count 没 ++)。
//
//     未来扫尾搬头的同类工作 (session_002 D1 项 4 一并搬): mtime 推进 / 中断检查 /
//     等也应放迭代头, 不放迭代尾。当前 a_01 没接 mtime / 中断, 占位等 a_03+。
// ============================================================================


int dispatcher(cpu_t *hart) {
    // sigsetjmp / siglongjmp 协议见 src/dummy.txt §1。
    sigjmp_buf dispatch_env;
    hart->jmp_buf_ptr = &dispatch_env;

    // total_count + local_count: 跨 longjmp 累加 / 持久, volatile 强制放栈 (dummy.txt §1
    // 末段)。两个变量都必须在 sigsetjmp 之前声明 + 初始化, 否则 longjmp 跳回时重新执行初始化,
    // local_count 丢失上一轮 SYNC_COUNT 写好的值, total_count 累加丢失。
    //
    // local_count 之前 (a_01_5_b ~ a_01_10) 在 while 体内每轮重新声明 (块作用域局部), 触发
    // session_002 D1 项 4 的 bug — longjmp 跳回 setjmp 落点 → 进 while 体 → local_count
    // 重新声明初始化 0, 上一轮 SYNC_COUNT 写好的 27 被覆盖。a_01_10 (7) step 1 修: 提到
    // sigsetjmp 之前 + volatile (跟 total_count 同形态)。
    volatile uint32_t total_count = 0;
    volatile uint32_t local_count = 0;

    // 一次性 sigsetjmp 建立永久落点; 返回值不分流 (dummy.txt §1 机制 (3) "dispatcher 视角"段)。
    sigsetjmp(*hart->jmp_buf_ptr, 1);

    while (hart->trap.in_trap < 3) {

    // ========================================================================
    // 迭代头扫尾 (session_002 D1 项 4 关键; a_01_10 (7) step 1 实施):
    //   - 累加上一轮 block 的 local_count 进 total_count
    //   - 重置 local_count, 准备下一轮 block 写入
    //
    // 必须放迭代头 (而不是迭代尾 block 3 之后): longjmp 跳回 setjmp 落点会跳过迭代尾,
    // 但跳到 while 顶后顺序进 while 体, 必经过迭代头; 跟正常 continue (mmu_translate_pc
    // fail 走 path 2b return rc → continue) 路径一样必经过迭代头。两条路径同形态扫尾。
    //
    // 未来 mtime 推进 / 中断检查 / perf_advance 等扫尾工作一并放迭代头 (session_002 D1
    // 项 4); 当前 a_01 没接, 占位等 a_03+。
    // ========================================================================
    total_count += local_count;
    local_count = 0;

    // ========================================================================
    // block 1: 算派发包 (regime, current_tlb) [D23 + D25 + D25.1 路线]
    //
    // 派发包概念上是两件 (file_plan §1.dispatcher 设计意图):
    //   regime       : 执行 regime, 决定 "用哪套 PTE 检查规则"
    //                    REGIME_BARE  = Trust  (M-mode 或任何 priv 带 bare satp)
    //                    REGIME_SV32  = Checked (S/U + Sv32)
    //   current_tlb  : 走 TLB 时用哪个叶 TLB
    //                    NULL          = Trust 不需要 TLB (D23 路线)
    //                    非 NULL       = Sv32 用 hart->tlb_table[priv][asid] 选定的叶
    //
    // D25 (接口层简化): 现阶段 regime 与 current_tlb 是否为 NULL 严格 1:1 一致
    //   (NULL ↔ BARE, 非 NULL ↔ SV32), 故下游 mmu_translate_pc / interpret_one_block
    //   只吃 current_tlb 即可 (NULL 编码 regime); 详见 mmu.h regime_t doc 段。
    // D25.1 (落地修订): dispatcher 内部仍把 regime 显式算出, 表达 "这是两个独立的派发概念,
    //   现阶段恰好一致" + 服务 dispatcher 末尾报告 label + 未来若 H 扩展真打破 1:1 时
    //   这里不需要重新引入变量。两个本地变量在同一 if/else 内一同赋值, 没有 inconsistent
    //   state 风险 (D25 担心的是接口层调用方传不一致, 不是同一函数内本地变量)。
    //
    // xatp 抽象层 (有意保留): 初版 = satp; 未来 H 扩展 V=1 时 = vsatp。
    //
    // 未来 misa 驱动派发 (不实现, 占位注释):
    //   - MU-only ISA: 没有 S-mode, U-mode 即使 satp.MODE = Sv32 也无意义 (没 PTE 设施),
    //     dispatcher 应识别 misa 把 U 也路由到 BARE。
    //   - 真要做时, dispatcher block 1 加 misa 检查:
    //       if (misa is MU-only && hart->priv == PRIV_U) {
    //           regime = REGIME_BARE; current_tlb = NULL;
    //       }
    // ========================================================================
    uint32_t xatp = hart->satp;
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
        // a_01_8 Step 6 (新) 末段解开 (从 Step 8 提前的"必要部分"): SV32 路径需 current_tlb
        // 非 NULL 才走 walker; 不解开的话切 S 后 current_tlb=NULL 让 mmu_translate_pc 走
        // BARE 路径, 不验 SV32 walker (fixture (g) 跑不通)。tlb_alloc 失败简化处理 (return
        // -1 + fprintf, 跟 ram_init / cpu_create 失败同形态); 不接 trap 路径 (host 内存
        // 耗尽是致命 host 错, 跟 guest 行为无关)。
        if (current_tlb == NULL) {
            current_tlb = tlb_alloc();
            if (current_tlb == NULL) {
                fprintf(stderr,
                        "[dispatcher] tlb_alloc failed for priv=%u asid=%u\n",
                        (uint32_t)hart->priv, asid);
                return -1;
            }
            hart->tlb_table[hart->priv][asid] = current_tlb;
        }
    }
    // regime 显式算出表达 D25.1 设计意图 ("两个独立的派发概念, 现阶段恰好一致, 未来 H 扩展
    // 真打破 1:1 时这里不重新引入变量"); 当前下游只吃 current_tlb (NULL 编码 regime), regime
    // 在本函数没有运行时消费者, 抑制 unused 警告。
    (void)regime;

    // ========================================================================
    // block 2: 取指路径 (mmu_translate_pc)
    //
    // dispatcher 内部得出 (regime, current_tlb) 两件; D25 后下游只吃 current_tlb
    // (NULL 编码 regime)。这是 mmu_translate_pc / interpret_one_block 的接口形态;
    // JIT 一侧不同 (block 编译时 baked regime, jit_cache key = (PA, regime))。
    // ========================================================================
    uint32_t pa;
    uint8_t *hva;
    int rc = mmu_translate_pc(hart, current_tlb, &pa, &hva);
    (void)pa;       // 未来给 JIT 查 jit_cache 用, a_01_5_b 不消费

    // ========================================================================
    // a_01_5_b: mmu_translate_pc 已在内部直调 trap_set_state 设好 xcause/xtval/xepc/
    // regs[0]=xtvec, rc 是 in_trap 当前值 (0/非0 状态)。
    // 非 0 → continue 让 while(in_trap < 3) 接管 (in_trap 已 ≥ 1, 第 3 次时退出 dispatcher)。
    // dummy.txt §1 路径 C (mmu fetch trap 不长跳)。
    // ========================================================================
    if (rc != 0) continue;

    // ========================================================================
    // block 3: 派发到 jit / interpreter
    //
    // 完整形态注释 (a_01_5+ 激活时把下方临时调用删掉, 解开这段):
    //   if (jit_cache_hit) {
    //       jit_block(hart, current_tlb, &local_counter);  // jit_cache 注册的 host_code_ptr
    //   } else {
    //       counter[pc]++;                                  // 热度计数 hash
    //       if (达阈值) trigger_translate(...);              // 触发翻译 (a_01 后改 thread 非阻塞)
    //       interpret_one_block(hart, current_tlb, hva, &local_counter);
    //                                                       // 解释器从 hva 直接读字节取指,
    //                                                       // 依赖块边界保证不跨 4K page
    //   }
    // 出块后所有扫尾搬到迭代头 (helper longjmp 跳回 sigsetjmp 落点会跳过迭代尾;
    // 只有迭代头才能保证一定执行)。
    //
    // a_01_4 临时调用 (jit_cache 没接 / 热度计数没接): 直接 interp 一次, 块边界由解释器
    // 末尾 is_block_boundary_inst 自然产生 (branch/jal/jalr 命中后退出 fetch loop)。
    // ========================================================================
    /* local_count 提到 sigsetjmp 之前 + volatile (a_01_10 (7) step 1 修, session_002 D1 项 4):
     * 跨 longjmp 持久, 不再每轮 while 体重新声明 (避免上一轮 SYNC_COUNT 写好的值被 0 覆盖)。
     * 累加 + reset 已搬到迭代头 (block 1 之前); 这里只做 interpret_one_block 调用本身。
     * (uint32_t *) cast 因 interpret_one_block 接口非 volatile 指针; 调用期间 volatile 语义
     * 不影响 (helper 内部本地操作), 调用前后 dispatcher 端 volatile 读写仍生效。 */
    interpret_one_block(hart, current_tlb, hva, (uint32_t *)&local_count);
    // perf_advance(hart, local_count);  // 占位, dispatcher 不消费 (a_03 接入); 真做时也搬迭代头

    // a_01_4 的 count==0 break 已删: OP_UNSUPPORTED 改 trap_raise_exception 后, count 永远
    // > 0 (trap 是发生在 case 头部, 之前的 boundary 指令至少一条已计入); 退出条件统一靠
    // while 顶 in_trap < 3。

    }  /* while (in_trap < 3) */

    // 退出循环后再做一次扫尾累加 — 最后一轮 block 的 local_count 还没进 total_count
    // (迭代头扫尾负责的是"上一轮", 最后一轮没下一轮迭代头来扫)。
    total_count += local_count;

    fprintf(stderr,
            "[dispatcher] halted: in_trap=%u total_count=%u pc=0x%08" PRIx32 "\n",
            hart->trap.in_trap, total_count, hart->regs[0]);

    // ========================================================================
    // 未来 reset 扩展占位 (a_03+ 真接入时填; 当前 dispatcher 直接 return 给 main 处理)
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
    //   // else: dispatcher 退出, main 端处理 (in_trap >= 10 走 main 的硬停机分支;
    //   //        misa 不支持 reset 也走 main exit)
    //
    // in_trap 编码语义 (扩展):
    //   0/1/2     普通 trap 嵌套深度
    //   3         标准 triple fault = reset 信号 (跟 RV/SiFive 风格一致)
    //   4..9      留作未来扩展 (远离 3 避免跟标准 reset 信号撞)
    //   >= 10     项目内部硬停机编码 (退出 hart, 不 reset; 例如 MMIO sifive_test finisher
    //              触发时, trap_set_state 直接把 in_trap 设到 10+ 而不是 ++ 到 3)
    //
    // 注: 这是项目自定义 host-side 计数协议; 跟 RV Smdbltrp 扩展 (riscv.h CAUSE_DOUBLE_TRAP=16,
    // 硬件检查 mstatus.MDT 字段触发 cause=16) 无关。项目当前不实现 Smdbltrp; 未来若实现, trap
    // delivery 路径会按 spec 走 cause=16, 跟这里的 in_trap 嵌套计数协议是两条独立机制 (会并存
    // 但语义不重叠 — Smdbltrp 是规范层 trap 投递机制, in_trap 是 host emulator 退出协议)。
    //
    // 规范: cpu 的分配 + 初始化 + 销毁都在 dispatcher 之外 (cpu_create / cpu_destroy 由 main
    // 调用)。cpu_reset 是状态重置 (不是 alloc/destroy), 是规范的合法例外, 允许由 dispatcher
    // 调用; 它在 cpu.c 实现, 跟 cpu_create / cpu_destroy 接口对称。
    // ========================================================================

    return 0;
}
