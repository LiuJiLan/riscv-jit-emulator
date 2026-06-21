//
// interpreter 模块对外接口。
//
// interpret_one_block: 解释执行一段 RV32 代码, 从 hva_pc 开始顺序取指 + decode + dispatch
//   + 执行, 直到:
//     (a) 命中 OP_UNSUPPORTED — 调 trap_raise_exception(hart, 2, d.raw_inst) _Noreturn
//         longjmp 跳回 dispatcher sigsetjmp 落点, 不返回这里。
//     (b) 命中 is_block_boundary_inst 硬边界 (branch/jal/jalr 已在 case 内自描述 pc);
//         fetch loop 末尾检查后 goto out, dispatcher 重新 mmu_translate_pc 进下一块。
//     (c) 命中 instruction-address-misaligned 等异常, WRITE_PC_OR_TRAP 内
//         trap_raise_exception _Noreturn longjmp 跳回 dispatcher (不返回到 case)。
//     (d) 已执行指令数达 BLOCK_INST_LIMIT (config.h, 当前 64; 失控保护, 正常 fixture 不会
//         撞到)。
//     (e) 推进后 hva_pc 跨 4K page (软边界, 跟 BLOCK_INST_LIMIT 同性质; 利用
//         host_ram_base 4K 对齐 → (hva_pc & 0xFFF) == (gva_pc & 0xFFF) invariant,
//         不需单独保留 gva)。块入口指令必跑一次 — entry_page 由块入口算, 推进后才判跨页。
//
// pc 维护:
//   - 算术 / 逻辑 / 立即数 op (含 RVC): 不动 pc, fetch loop 末尾统一 hart->regs[0] +=
//     d.pc_step (PC_STEP_RV=4 / PC_STEP_RVC=2)。
//   - 控制流 op (branch/jal/jalr): case 内自描述 hart->regs[0] (走 WRITE_PC_OR_TRAP
//     宏, 内含 IALIGN 对齐检查 + trap_raise_exception); pc_step 设实际指令长度
//     (RV=4 / RVC=2), branch taken / jal / jalr 走 goto out 跳过 fetch loop 末段,
//     branch not-taken 让末段 += d.pc_step 自动顺序推进 (decode.h pc_step doc)。
//   - OP_UNSUPPORTED / trap 路径: hart->regs[0] 停在触发指令本身 (RV precise trap 语义,
//     trap 时 mepc = 触发指令的 PC; trap_raise_exception 内部读 hart->regs[0] 当 mepc)。
//
// trap_raise_exception 接口 (interpreter / 未来 JIT 共用): 见 trap.h 顶部 doc (cause +
// tval; mepc 隐式 = hart->regs[0]; xtval/xcause/xepc 中 x 由 helper 内部按 medeleg 决定,
// caller 不需要知道)。
//
// count_out 同步契约 (跟 dispatcher 顶部 doc 对齐): SYNC_COUNT 宏在 may-trap 路径 case 内
// trap_raise / helper-call 之前同步; boundary 路径在 out 段托底同步; pure case 不需同步
// (count 在栈帧上累加, block 末传出)。详见 dispatcher.c 顶部 "count 同步契约" 段。
//
// 参数:
//   hart        - 调用 hart (regs / x0 特殊路径都通过它)
//   current_tlb - dispatcher 算好的派发包 (NULL = REGIME_BARE; 非 NULL = REGIME_SV32_S 或 _U);
//                 透传给 lsu_load_helper / lsu_store_helper (interpreter 自感知 priv,
//                 通过 current_tlb NULL/非NULL 化简表达)。
//   hva_pc      - dispatcher 通过 mmu_translate_pc 得出的本块入口 HVA, 解释器从这里读
//                 第一条指令字节; 块内每条指令完整在同一 4K page 内 (推进后跨页时退出
//                 block, 见上方 (e))。
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
                         uint8_t *hva_pc, uint64_t *count_out);

#endif //CORE_INTERPRETER_H
