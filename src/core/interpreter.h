//
// Created by liujilan on 2026/4/28.
// a01_3 interpreter 模块对外接口。
//
// interpret_one_block: 解释执行一段 RV32 代码, 从 hva_pc 开始顺序取指 + decode + dispatch
//   + 执行, 直到:
//     (a) 命中 OP_UNSUPPORTED — break 出 fetch loop。a01_3 用作 fixture 末尾停止信号
//         (在 .S 末尾放 ecall, decode 归 OP_UNSUPPORTED); 未来真接 a_03 trap.c 后, 这里
//         改为 trap_raise(2, mtval=d.raw_inst) + siglongjmp, dispatcher 主循环不返回这里。
//     (b) 已执行指令数达到 a01_3 失控保护 hard limit (256 条; a01_4+ 真做软边界时移到
//         config.h, 并加跨 4K page 检查 + 真边界 op (branch/jal/jalr/...) 截断)。
//
// pc 维护:
//   - 算术 / 逻辑 / 立即数 op 不动 pc, 解释器在 op switch 后统一 hart->regs[0] += 4。
//   - OP_UNSUPPORTED 触发 break 时, hart->regs[0] 停在触发指令本身 (RV trap 语义对齐:
//     trap 时 mepc = 触发指令的 PC; 未来真接 trap.c 时, trap_raise 直接读 regs[0] 写
//     mepc 就对)。
//   - 控制流 op (branch/jal/jalr) a01_4+ 接入时, 它们 case 内部自己改 hart->regs[0],
//     需要重构 fetch loop 的 +4 逻辑 (用 continue 或者把 +4 提前到默认路径)。
//
// 参数:
//   hart        - 调用 hart (regs / x0 特殊路径都通过它)
//   current_tlb - dispatcher 算好的派发包 (NULL = REGIME_BARE; 非 NULL = REGIME_SV32);
//                 a01_3 暂未真用 (没接 load/store, 也没碰 mmu_walker_*); load/store 接入
//                 (a01_4+) 时透传给 mmu_walker_load / store。
//   hva_pc      - dispatcher 通过 mmu_translate_pc 得出的本块入口 HVA, 解释器从这里读
//                 第一条指令字节; 假设本块不跨 4K page (a01_3 fixture 满足; 边界判定
//                 a01_4+ 真做)。
//   count_out   - 出参; 本次实际执行的指令数 (不含触发 OP_UNSUPPORTED 的那条)。
//
// x0 处理见 dummy.txt §2: 解释器内部 READ_REG / WRITE_REG 宏分流, regs[0] (= pc) 物理
//   位置不被 x0 读写碰到。
//

#ifndef CORE_INTERPRETER_H
#define CORE_INTERPRETER_H

#include <stdint.h>

#include "cpu.h"
#include "tlb.h"

void interpret_one_block(cpu_t *hart, tlb_t *current_tlb,
                         uint8_t *hva_pc, uint32_t *count_out);

#endif //CORE_INTERPRETER_H
