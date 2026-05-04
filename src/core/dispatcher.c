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
// a_01_5_b 当前形态: while(hart->trap.in_trap < 3) 多块循环。
//   - sigsetjmp 仍不调 (helper 还不长跳, 等 a_01_5_c 加 _Noreturn 真激活)
//   - 退出条件: hart->trap.in_trap >= 3 (triple fault, 项目内部停机协议; 见 dummy.txt §1)
//     trap_set_state 内部 in_trap >= 3 时早 return 不 deliver (候选 A); main 端拿回控制后
//     fprintf 表 halt + 未来 reset 接入。这是项目当前的"标准停机机制", 直到未来 MMIO
//     bus / sifive_test 起来后改 MMIO finisher 路径。
//   - mmu_translate_pc 改造后 (a_01_5_b): rc != 0 → continue (trap_set_state 已设字段 +
//     regs[0]=xtvec, while 条件接管)。a_01_4 的 fetch fprintf + return 1 路径删除。
//   - count==0 break 删除: a_01_4 该路径用于 ecall→OP_UNSUPPORTED→silent halt;
//     a_01_5_b 起 OP_UNSUPPORTED 改 trap_raise_exception, count 永远 > 0 不再走 break。
//
// 完整形态 (a_01_5_c 后):
//   sigsetjmp(*hart->jmp_buf_ptr, 1);   /* 一次性, while 外, 见 dummy.txt §1 */
//   while (hart->trap.in_trap < 3) {
//       /* perf_advance / mtime 推进 / 中断检查 (a_03) */
//       /* block 1 + 2 + 3 */
//   }
// ============================================================================


int dispatcher(cpu_t *hart) {
    // sigsetjmp / siglongjmp 协议见 src/dummy.txt §1。
    // a_01_5_b: jmp_buf 实体声明 + jmp_buf_ptr 赋值, 但**不调 sigsetjmp**
    //           (helper 还不 longjmp; a_01_5_c 加 _Noreturn 真激活时 sigsetjmp 一次性
    //            放 while 外)。实体在 dispatcher 栈帧上 (cpu_t 只持指针, 见 dummy.txt §1)。
    sigjmp_buf dispatch_env;
    hart->jmp_buf_ptr = &dispatch_env;

    uint32_t total_count = 0;     // 累加 local_count, halted 时一并报告
                                   // (a_01_5_c 加 sigsetjmp 后改 volatile, 见 dummy.txt §1 末段)

    while (hart->trap.in_trap < 3) {

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
        // future 懒分配 (a_01 不会到这, SV32 在 a_01 不触发):
        //   if (current_tlb == NULL) {
        //       current_tlb = tlb_alloc();
        //       hart->tlb_table[hart->priv][asid] = current_tlb;
        //   }
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
    uint32_t local_count = 0;
    interpret_one_block(hart, current_tlb, hva, &local_count);
    // perf_advance(hart, local_count);  // 占位, dispatcher 不消费 (a_03 接入)

    total_count += local_count;

    // a_01_4 的 count==0 break 已删: OP_UNSUPPORTED 改 trap_raise_exception 后, count 永远
    // > 0 (trap 是发生在 case 头部, 之前的 boundary 指令至少一条已计入); 退出条件统一靠
    // while 顶 in_trap < 3。

    }  /* while (in_trap < 3) */

    fprintf(stderr,
            "[dispatcher] halted: in_trap=%u total_count=%u pc=0x%08" PRIx32 "\n",
            hart->trap.in_trap, total_count, hart->regs[0]);

    return 0;
}
