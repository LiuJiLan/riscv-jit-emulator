//
// Created by liujilan on 2026/4/28.
// a_01 dispatcher 伪代码
// 跨文件协议见 src/dummy.txt §1(sigsetjmp)/ §4(TLB 分发机制)
//

#include "dispatcher.h"

// a_01: 以下整段是伪代码占位,等 a01_5(interpreter / dispatcher / main 翻译)
//        入场再激活。当前用 #if 0 包住以保证整个项目可编译。
//        协议见 src/dummy.txt §1(sigsetjmp)/ §4(TLB 分发机制)。
#if 0

// dispatcher的作用:
// I. 完成block级别的pc的mmu translation工作
//    直接翻译结果为GPA
//      JIT hash机制依赖PA得到 -> Host JIT PC ADDR
//      解释器依靠PA和ram中的偏移信息得到 -> GPA_PC



int dispatcher(cpu_t *hart) {  // 返回值 a_01 范围内随便,a_01 后 helper 走 exit 走不到这
    // sigsetjmp / siglongjmp 协议见 src/dummy.txt §1
    sigjmp_buf dispatch_env                         // TODO(a_01: 注释位,a_01 不实现)
    hart->jmp_buf_ptr = &dispatch_env               // TODO(a_01: 注释位)

    volatile uint32_t local_counter = 0             // volatile: helper 通过 &local_counter 写;
                                                    //           跨 sigsetjmp/siglongjmp 必须 volatile,见 dummy.txt §1 附段
    int loop_count = 0                              // a_01 临时退出机制,a_01 后删

    while (1) {
        sigsetjmp(*hart->jmp_buf_ptr, 1)            // TODO(a_01: 注释位)
                                                    // longjmp 跳回这里,接着进迭代体

        // === 迭代头扫尾 ===
        hart->cycle += local_counter                // TODO(a_01 后): 累加到共享区 _cycle / _instret
                                                    // TODO(a_01 后): mtime 推进 + 中断检查
        // 未来设计备注: cycle CSR 的 helper 程序需要传递 local_counter 的指针,
        //              因为方便调用时返回正确的值; helper 会将已有的 local_counter
        //              累加到 cycle 并清空 local_counter。未来备注, 不实现。
        local_counter = 0

        // === a_01 临时退出 ===
        if (loop_count >= 1) break                  // TODO(a_01 后): 删除, 真退出 = helper 走 exit
        loop_count += 1

        // === block 1: 选叶 TLB(见 dummy.txt §4 TLB 分发机制) ===
        block:  // 意图, 每次 dispatcher 要拿到指令的 gpa(guest_virtual_address),
                // 但是, 此时可能还没有 tlb, 检查的工作, 由 dispatcher 做
        xatp = hart->satp           // xatp 抽象: 别直接用 hart->satp;
                                    // 留给 H 扩展时切到 hart->vsatp 等接口位

        if hart->priv == 3 (M 态) or xatp.mode == bare:    # 实际位运算
            current_tlb_slot = &tlb_table[3]        # M 态 / bare 模式都走单 TLB
                                                    # (本质都是恒等映射, 无 ASID 概念)
        else:
            current_tlb_slot = &tlb_table[priv][xatp.ASID]   # S/U 二级索引

        if *current_tlb_slot == NULL:
            *current_tlb_slot = tlb_alloc()         # tlb 模块函数, 见 §2.tlb H1
        current_tlb = *current_tlb_slot             # 不论新旧都从槽里取
                                                    # (D1 修正: 原 if 内赋值会让命中分支用未初始化值)
        block_end

        // === block 2: fetch 路径(取 pc 对应的 host pointer)===
        block:
        // TODO(a_01: 等 tlb / mmu 实现完成后再填,interpret_one_block 也是)
        //   - 查 TLB 命中: 直接拿 HA
        //   - miss: gpa = mmu_translate_fetch(hart->pc, current_tlb)
        //           → 装载 TLB → 拿 HA
        // 注: walker 在 a_01 = M/bare short-circuit(gva == pa, 不走真页表)
        // 注: TLB 是 load/store 基础;fetch 路径(pc 推进)同样走这条 —— dummy.txt §4
        // 问题, mmu 家族所有的函数命名还没有想好, 但是也可以先定 mmu_translate_fetch
        block_end

        // === block 3: 派发(jit 或者解释器, 传递都是 cpu_t, current_tlb, &local_counter)===
        block:
        if (0) {                                    // a_01 永远不进(jit 命中分支占位, 保留结构)
            // 拿到 jit_host_code
            // 调用 jit_host_code(hart, current_tlb, &local_counter)
        } else {                                    // 未命中
            // counter[pc]++                        // 这个用的是类似 python dict() 的外代码, 表达 hash 意图
                                                    // TODO(a_01 后): hash 表查热度
            // if 达到阈值:
            //     触发翻译 + 编译                  // TODO(a_01 后): 后续改为 thread 非阻塞
            interpret_one_block(hart, current_tlb, &local_counter)  // 函数签名中缺乏一个块用的
            // 注: 解释器内部从 hart->pc 派生 host pointer,
            //     a_01 简化下 = host_ram_base + (hart->pc - RAM_START_GPA);
            //     依赖块边界保证不跨页(由项目大机制定调)
        }
        block_end

        // 出块后没有"扫尾"工作 —— 所有扫尾搬到迭代头
        // (helper longjmp 跳回设置点会跳过迭代尾;只有迭代头才能保证一定执行)
    }

    return 0
}

#endif // a_01 伪代码占位
