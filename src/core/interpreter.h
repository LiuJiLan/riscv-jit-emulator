//
// Created by liujilan on 2026/4/28.
// a01_3 interpreter 模块对外接口。
//
// interpret_one_block: 解释执行一段 RV32 代码, 从 hva_pc 开始顺序取指 + decode + dispatch
//   + 执行, 直到:
//     (a) 命中 OP_UNSUPPORTED — goto out 出 fetch loop。fixture 末尾 ecall (a_01_4 仍未
//         实现 ecall 真语义, decode 归 OP_UNSUPPORTED) 用作停止信号; 未来真接 a_01_5
//         trap.c 后, 改为 trap_raise_exception(hart, 2, d.raw_inst) + siglongjmp,
//         dispatcher 主循环不返回这里。
//     (b) 命中 is_block_boundary_inst 硬边界 (branch/jal/jalr 已在 case 内自描述 pc);
//         fetch loop 末尾检查后 goto out, dispatcher 重新 mmu_translate_pc 进下一块。
//     (c) 命中 instruction-address-misaligned 等异常, WRITE_PC_OR_TRAP 内 goto out
//         (a_01_4 trap_raise_exception fprintf 占位; a_01_5 真接 trap.c 后改 longjmp,
//         不返回到 case)。
//     (d) 已执行指令数达 BLOCK_INST_LIMIT (config.h, 当前 64, 与 plan §1.23.2 软边界初版
//         默认对齐); 失控保护, 正常 fixture 不会撞到。a_05+ OS 场景再叠加跨 4K page 检查。
//
// pc 维护:
//   - 算术 / 逻辑 / 立即数 op (含 RVC): 不动 pc, fetch loop 末尾统一 hart->regs[0] +=
//     d.pc_step (PC_STEP_RV=4 / PC_STEP_RVC=2)。
//   - 控制流 op (branch/jal/jalr): case 内自描述 hart->regs[0] (走 WRITE_PC_OR_TRAP
//     宏, 内含 IALIGN 对齐检查 + trap_raise_exception 占位); pc_step=PC_STEP_NONE 使
//     fetch loop +=0 不覆盖 case 写入。
//   - OP_UNSUPPORTED / trap 路径: hart->regs[0] 停在触发指令本身 (RV precise trap 语义,
//     trap 时 mepc = 触发指令的 PC; trap_raise_exception 内部读 hart->regs[0] 当 mepc)。
//
// trap_raise_exception 接口 (interpreter / 未来 JIT 共用): 见 interpreter.c 内的 helper
// 块顶部注释 (cause + tval; mepc 隐式 = hart->regs[0]; xtval/xcause/xepc 中 x 由 helper
// 内部 mideleg/medeleg 决定, caller 不需要知道)。
//
// 参数:
//   hart        - 调用 hart (regs / x0 特殊路径都通过它)
//   current_tlb - dispatcher 算好的派发包 (NULL = REGIME_BARE; 非 NULL = REGIME_SV32);
//                 a01_3/a01_4 暂未真用 (没接 load/store, 也没碰 mmu_walker_*); load/store
//                 接入 (a01_6+) 时透传给 mmu_walker_load / store。
//   hva_pc      - dispatcher 通过 mmu_translate_pc 得出的本块入口 HVA, 解释器从这里读
//                 第一条指令字节; 假设本块不跨 4K page (当前 fixture 满足; 跨页边界判定
//                 a_05+ OS 场景再做)。
//   count_out   - 出参; 本次实际执行的指令数 (含 boundary 那条 = 已成功执行;
//                 不含触发 OP_UNSUPPORTED 或 trap 的那条 = 未执行)。
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
