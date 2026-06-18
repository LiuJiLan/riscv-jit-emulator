//
// Created by liujilan on 2026/6/5.
// jit/ir.h —— JIT 自制 IR 定义 (POD; backend 视角 host-agnostic + ISA-agnostic).
//
// ============================================================================
// 设计依据 (plan §1.21 三层架构 + IR 选择 = 选项 B)
// ============================================================================
//
// 三层职责 (Dispatcher / Translator / JitBackend):
//   - Translator 知道 RISC-V, 不知道 host  → 产出 IR (后端无关)
//   - JitBackend 知道 host, 不知道 RISC-V  → 消费 IR
//   - 三层之间的接口 = 自制 IR, 这是分层成立的关键
//
// IR vs decoded_inst_t (重要区分):
//   - decoded_inst_t (core/decode.h) 是 **RV 视角** 三地址码 — op_kind_t 名字
//     OP_LUI / OP_AUIPC / OP_BEQ / ... 都是 RV opcode; interpreter / translator
//     共享 decode
//   - ir_inst_t (本头) 是 **backend 视角** host-agnostic + ISA-agnostic 中间形式
//     — backend 不感知 RV opcode, 只看 IR op (例 IR_OP_BRANCH_LT 合并 BLT/BGE
//     signed 版, 参数标 signed)
//   - 两者形态相似但定位不同; backend 直接消费 decoded_inst_t = plan §1.21
//     否决的选项 A "无 IR / backend 被迫 RV 化"
//
// ============================================================================
// 当前骨架范围
// ============================================================================
//
// 当前只放最小骨架: ir_op_kind_t enum 两条占位 + ir_inst_t 最少必要字段.
// 详细 IR op 列表 / 字段细节 / 三地址码完整形态 b_02 真做 emit 时拍.
//
// 命名:
//   - 类型 ir_*_t (ir_op_kind_t / ir_inst_t)
//   - enum 值 IR_OP_* (IR_OP_UNSUPPORTED / IR_OP_DISPATCH_EXIT)
//   - 跟 decode.h op_kind_t / OP_* 平行, 区分 RV 视角 vs backend 视角
//

#ifndef JIT_IR_H
#define JIT_IR_H

#include "riscv.h"   // uxlen_t (target_pc 字段; dummy.txt §13 typedef family)


// ----------------------------------------------------------------------------
// ir_op_kind_t —— IR op 分类 (当前 2 条占位; b_02 真做 emit 时扩)
//
// 当前两条占位是块出口必需的 (plan §1.22.6 + §1.23.3; audit 拍法 Q4 a):
//   IR_OP_UNSUPPORTED   — ir_op_kind_t enum 完整性占位 (-Wswitch-enum 跟
//                          decode / translate default case 用); Translator 撞到
//                          OP_UNSUPPORTED 时**不进 IR buffer**, 只在 buffer 末尾
//                          emit 一条 IR_OP_DISPATCH_EXIT(target_pc =
//                          unsupported_inst.pc) 作块出口; 块前缀部分编译成功
//                          (前 N 条支持指令进 IR + 末加 DISPATCH_EXIT); 运行时
//                          dispatcher 下一轮 PC = unsupported_inst.pc 时
//                          jit_cache miss → 调 interpret_one_inst →
//                          OP_UNSUPPORTED case → trap_raise_exception(cause 2)
//   IR_OP_DISPATCH_EXIT — 块出口标记, target_pc 字段填新 pc; backend emit 写
//                          cpu->pc + ret; dispatcher 主循环重派发新 pc
//
// b_02 真做 emit 时扩 (示意, 真做时拍):
//   IR_OP_ADD / SUB / SLT / XOR / AND / OR / SLL / SRL / SRA / ...
//   IR_OP_LOAD_8/16/32_S/U (按 size + sext/zext 分; 不按 RV LB/LH/LW 分)
//   IR_OP_STORE_8/16/32
//   IR_OP_BRANCH_EQ / LT / LT_U (合并 RV BLT/BGE signed 版, 参数标 signed)
//   IR_OP_CALL_HELPER (slow path helper call; ID + 参数)
//   ...
// ----------------------------------------------------------------------------
typedef enum {
    IR_OP_UNSUPPORTED   = 0,
    IR_OP_DISPATCH_EXIT = 1,
    // 详细 op_kind 列表 b_02 真做时拍 (参考 plan §1.21 / §1.22 / §1.23 + 真做时
    // backend 角度真需要的最小集)
} ir_op_kind_t;


// ----------------------------------------------------------------------------
// ir_inst_t —— 单条 IR 指令 (POD; 当前最少必要字段)
//
// 当前只放 kind + target_pc (DISPATCH_EXIT 用); b_02 真做 emit 时扩 dst /
// src1 / src2 / imm 等三地址码字段.
// ----------------------------------------------------------------------------
typedef struct {
    ir_op_kind_t kind;
    uxlen_t      target_pc;   // IR_OP_DISPATCH_EXIT 时 = 下一块入口 pc;
                              // IR_OP_UNSUPPORTED  时 = 触发指令 pc;
                              // 其他 op b_02 真做时按需用 (未来可能改成 union /
                              // 三地址码字段)
} ir_inst_t;


#endif //JIT_IR_H
