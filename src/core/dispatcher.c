//
// Created by liujilan on 2026/4/28.
// a01_3 dispatcher 实现 (block 1+2+3 真调度起步; 临时 if(1) 限制单次, 等 a_03 trap.c
// + a01_5 完整 dispatcher 时改 while(1) + sigsetjmp)。
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
// II. 在 helper longjmp 跳回时承接控制流 (sigsetjmp landing) - 等 a_03 trap.c 真接入
//
// III. 迭代头 perf 同步 / mtime 推进 / 中断检查 - 等 a_03 真接入
//
// 完整形态 (a01_5 后):
//   while (1) {
//       sigsetjmp(*hart->jmp_buf_ptr, 1);
//       /* 迭代头扫尾: hart->cycle += local_counter; local_counter = 0;
//                       mtime 推进 + 中断检查 */
//       /* block 1 + 2 + 3 */
//   }
//
// a01_3 当前形态: if(1) 包 block 1+2+3, 跑一次。
//   - while(1) → if(1): 一次性执行, 不引入临时 "已跑过" 标记变量
//   - sigsetjmp 仍不调 (a01_3 没有 helper 会 longjmp, 等 a_03 trap.c 接入)
//   - block 3: 完整 jit_cache_hit/else 派发结构作为注释保留 (设计意图);
//              其下方临时调 interpret_one_block 一次, 完整派发 a01_5+ 激活时换上
// ============================================================================


int dispatcher(cpu_t *hart) {
    // sigsetjmp / siglongjmp 协议见 src/dummy.txt §1。
    // a01_3: jmp_buf 实体声明 + jmp_buf_ptr 赋值, 但**不调 sigsetjmp**
    //         (没 helper 会 longjmp; 真激活等 a_03 trap.c + a01_5 完整 dispatcher)。
    //         实体在 dispatcher 栈帧上 (cpu_t 只持指针, 紧凑布局, 见 dummy.txt §1)。
    sigjmp_buf dispatch_env;
    hart->jmp_buf_ptr = &dispatch_env;

    if (1) {  // a01_3 临时形态 (while(1) → if(1), 跑一次, 不需要 hack 变量)

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

    // ========================================================================
    // block 2: 取指路径 (mmu_translate_pc)
    //
    // dispatcher 内部得出 (regime, current_tlb) 两件; D25 后下游只吃 current_tlb
    // (NULL 编码 regime)。这是 mmu_translate_pc / interpret_one_block 的接口形态;
    // JIT 一侧不同 (block 编译时 baked regime, jit_cache key = (PA, regime))。
    // ========================================================================
    uint32_t pa;
    uint8_t *hva;
    int cause = mmu_translate_pc(hart, current_tlb, &pa, &hva);

    // ========================================================================
    // a01_3 取指 trap 临时处理 (a_03 trap.c 接入后改为 trap_raise(hart, cause)):
    //
    // 真接入后 dispatcher 不在这里 fprintf + return; 而是 trap_raise + 走 sigsetjmp
    // landing 重入下一轮迭代 (mmu.h 顶部 trap_raise 段 + dispatcher 预期使用伪码)。
    // 当前 a01_3 fixture 全部走 RAM 内 BARE 路径, cause = 0; 这条仅为防 fixture 写错时
    // 给个有用错误信息。pa 仅在未来给 JIT 查 jit_cache 用, a01_3 不消费 (cast to void)。
    // ========================================================================
    (void)pa;

    if (cause != 0) {
        fprintf(stderr, "[dispatcher] fetch trap: cause=%d pc=0x%08" PRIx32 "\n",
                cause, hart->regs[0]);
        return 1;
    }

    // ========================================================================
    // block 3: 派发到 jit / interpreter
    //
    // 完整形态注释 (a01_5+ 激活时把下方临时调用删掉, 解开这段):
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
    // a01_3 临时调用 (jit_cache 没接 / 热度计数没接 / 块边界没真做): 直接 interp 一次。
    // ========================================================================
    uint32_t local_count = 0;
    interpret_one_block(hart, current_tlb, hva, &local_count);
    // perf_advance(hart, local_count);  // 占位, dispatcher 不消费 (a_03 接入)

    fprintf(stderr,
            "[dispatcher] block done: regime=%s count=%u pc=0x%08" PRIx32 "\n",
            regime == REGIME_BARE ? "BARE" : "SV32",
            local_count, hart->regs[0]);

    }  /* if (1) */

    return 0;
}
