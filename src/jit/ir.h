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
// T1+T2 真做范围 (RV32I 算术全集 = U-type 2 + I-imm 9 + R-type 10 = 21 op)
// ============================================================================
//
// IR_OP_UNSUPPORTED   - enum 完整性哨兵 (永不真出现在 IR buffer; -Wswitch-enum
//                        + -Werror 强制 backend switch 完整, default case 写
//                        __builtin_unreachable / abort 兜底). 跟 decode.h
//                        OP_UNSUPPORTED 同名陷阱: decode 端真撞, IR 端哨兵不撞.
// IR_OP_DISPATCH_EXIT - 块出口模板 (= interpreter is_block_boundary_inst 后
//                        goto out 的 JIT 对偶); target_pc 字段填下一块入口 pc.
//                        backend emit 写 cpu->pc + 写 count_out + epilogue + ret.
//                        每个 IR 流末尾必有一条 IR_OP_DISPATCH_EXIT.
//
// RV32I 算术全集 21 op (按 decode.h op_kind_t 同分段顺序):
//   U-type 2:
//     IR_OP_LUI    rd = (uint32_t)imm                  (decode 已 sext20 + <<12)
//     IR_OP_AUIPC  rd = (uint32_t)imm                  (translator 端常量折叠;
//                                                       imm 字段存 baked_pc +
//                                                       d.imm 合并后常数; 跟
//                                                       QEMU 默认 + rv8 一致)
//   I-imm 9 (rs1 + imm; SLLI/SRLI/SRAI 的 shamt 在 imm 低 5 位):
//     IR_OP_ADDI   rd = rs1 + imm
//     IR_OP_SLTI   rd = ((int32_t)rs1 < imm) ? 1 : 0   (signed 比较)
//     IR_OP_SLTIU  rd = (rs1 < (uint32_t)imm) ? 1 : 0  (imm 先 sext 到 32, 再
//                                                       unsigned 比较)
//     IR_OP_XORI   rd = rs1 ^ imm
//     IR_OP_ORI    rd = rs1 | imm
//     IR_OP_ANDI   rd = rs1 & imm
//     IR_OP_SLLI   rd = rs1 << (imm & 0x1F)
//     IR_OP_SRLI   rd = rs1 >> (imm & 0x1F)
//     IR_OP_SRAI   rd = (int32_t)rs1 >> (imm & 0x1F)   (算术右移)
//   R-type 10 (rs1 + rs2):
//     IR_OP_ADD    rd = rs1 + rs2
//     IR_OP_SUB    rd = rs1 - rs2
//     IR_OP_SLL    rd = rs1 << (rs2 & 0x1F)
//     IR_OP_SLT    rd = ((int32_t)rs1 < (int32_t)rs2) ? 1 : 0
//     IR_OP_SLTU   rd = (rs1 < rs2) ? 1 : 0
//     IR_OP_XOR    rd = rs1 ^ rs2
//     IR_OP_SRL    rd = rs1 >> (rs2 & 0x1F)
//     IR_OP_SRA    rd = (int32_t)rs1 >> (rs2 & 0x1F)
//     IR_OP_OR     rd = rs1 | rs2
//     IR_OP_AND    rd = rs1 & rs2
//
// rd ∈ x1-x5 走固定 host reg 路径; 否则 load tmp/op/store tmp; rd = x0 直接
// 跳过写 (dummy.txt §2 dead store 体例).
//
// 后续 T3-T6 扩 enum:
//   T3 helper call:    LOAD/STORE 8 + CSR 6 + AMO 9 + LR/SC 2 + FENCE 2 (backend
//                       emit `call <static_addr>`; helpers.h re-export)
//   T4 控制流:         BRANCH 6 + JAL + JALR; backend emit 条件跳 / 块出口模板
//   T6 M+RVC:          MUL/DIV/REM 8 + RVC 压缩指令
//
// IR 流形态 (T1+T2):
//   [21 op 之一 前缀 N 条 (0 ≤ N ≤ BLOCK_INST_LIMIT - 1)]
//   [IR_OP_DISPATCH_EXIT 收尾 1 条 (target_pc = 下一块入口 pc = 截断指令 PC 或
//    软边界出块 PC)]
//
// 块前缀空 (N=0, 第一条就是非 RV32I 算术): translator 仍 emit DISPATCH_EXIT
// 收尾一条, 但 backend.compile_block 检测到 IR 流只有 DISPATCH_EXIT 时返
// JIT_ERR_NOT_IMPLEMENTED (块前缀无真翻译指令, 不值得 install), jit_entry.cc
// 走 set_blacklist 路径; dispatcher 下次 lookup BLACK miss → interpret 兜底真
// 执行那条 RV 指令.
//
// AUIPC PA != VA trail: V1 BARE only, translator cur_pc = pa = VA; SV32 上线
// (b_03+) 时 AUIPC baked_pc 来源 (用 pa 还是 cpu->pc VA) 单独议.
//
// 命名:
//   - 类型 ir_*_t (ir_op_kind_t / ir_inst_t)
//   - enum 值 IR_OP_* (IR_OP_UNSUPPORTED / IR_OP_DISPATCH_EXIT / 21 RV32I 算术 op)
//   - 跟 decode.h op_kind_t / OP_* 平行, 区分 RV 视角 vs backend 视角
//

#ifndef JIT_IR_H
#define JIT_IR_H

#include <stdint.h>

#include "riscv.h"   // uxlen_t (target_pc 字段; dummy.txt §13 typedef family)


// ----------------------------------------------------------------------------
// ir_op_kind_t —— IR op 分类 (T1+T2 范围: 2 哨兵/出口 + 21 RV32I 算术全集)
//
// IR_OP_UNSUPPORTED   - enum 完整性哨兵 (永不真出现在 IR buffer); backend
//                        default case 写 __builtin_unreachable / abort 兜底
// IR_OP_DISPATCH_EXIT - 块出口模板; target_pc 字段填下一 pc
//
// 21 RV32I 算术 op 按 decode.h op_kind_t 分段顺序排 (U-type → I-imm → R-type);
// 每条 op 语义见顶段 doc.
//
// IR_OP_UNSUPPORTED=0 / IR_OP_DISPATCH_EXIT=1 sentinel 显式值保留 (其他位置可
// 自由调整 enum 顺序而不影响 sentinel 语义); 21 算术 op 不指定显式值, switch
// 按名字访问.
//
// 后续 T3-T6 扩 enum 按 RV op 分组追加 (LOAD/STORE 跟 I-type/S-type 段; BRANCH/
// JAL/JALR 控制流段; CSR/AMO 等).
// ----------------------------------------------------------------------------
typedef enum {
    IR_OP_UNSUPPORTED   = 0,
    IR_OP_DISPATCH_EXIT = 1,

    /* ---- U-type (2) ---- */
    IR_OP_LUI,
    IR_OP_AUIPC,

    /* ---- I-type OP-IMM (9; SLLI/SRLI/SRAI shamt 在 imm 低 5 位) ---- */
    IR_OP_ADDI,
    IR_OP_SLTI,
    IR_OP_SLTIU,
    IR_OP_XORI,
    IR_OP_ORI,
    IR_OP_ANDI,
    IR_OP_SLLI,
    IR_OP_SRLI,
    IR_OP_SRAI,

    /* ---- R-type OP (10) ---- */
    IR_OP_ADD,
    IR_OP_SUB,
    IR_OP_SLL,
    IR_OP_SLT,
    IR_OP_SLTU,
    IR_OP_XOR,
    IR_OP_SRL,
    IR_OP_SRA,
    IR_OP_OR,
    IR_OP_AND,
} ir_op_kind_t;


// ----------------------------------------------------------------------------
// ir_inst_t —— 单条 IR 指令 (POD; 三地址码 + 块出口 target_pc 联合)
//
// 字段使用约定 (按 kind 不同):
//   kind = IR_OP_DISPATCH_EXIT:
//     target_pc - 下一块入口 PC (硬边界算 / 软边界出块 PC / 截断指令 PC)
//     rd/rs1/rs2/imm - 不用
//   kind ∈ U-type {IR_OP_LUI, IR_OP_AUIPC}:
//     rd  - 寄存器号 0..31
//     imm - 32 位常数 (translator 端: LUI = decode.imm 直传; AUIPC = baked_pc +
//           decode.imm 合并 — 选 a 常量折叠, 跟 QEMU 默认 + rv8 一致)
//     rs1 / rs2 / target_pc - 不用
//   kind ∈ I-imm {IR_OP_ADDI, IR_OP_SLTI, IR_OP_SLTIU, IR_OP_XORI, IR_OP_ORI,
//                  IR_OP_ANDI, IR_OP_SLLI, IR_OP_SRLI, IR_OP_SRAI}:
//     rd / rs1 - 寄存器号 0..31
//     imm      - 12 位符号扩展立即数 (跟 decoded_inst_t.imm 体例对偶; SLLI/SRLI/
//                SRAI 时低 5 位是 shamt, backend 端 & 0x1F)
//     rs2 / target_pc - 不用
//   kind ∈ R-type {IR_OP_ADD, IR_OP_SUB, IR_OP_SLL, IR_OP_SLT, IR_OP_SLTU,
//                   IR_OP_XOR, IR_OP_SRL, IR_OP_SRA, IR_OP_OR, IR_OP_AND}:
//     rd / rs1 / rs2 - 寄存器号 0..31
//     imm / target_pc - 不用
//   kind = IR_OP_UNSUPPORTED:
//     永不真出现 — 哨兵, 字段不读
//
// 字段类型选 uint8_t 寄存器号 (节省 IR 流内存; RV reg 0..31 占 5 bit, uint8_t
// 够) + int32_t imm (跟 decoded_inst_t.imm 同类型).
//
// future 字段扩 (T3 真翻译 LOAD/STORE/CSR/AMO 时):
//   funct3 / funct5 / amo aq/rl / csr_addr 等子分类字段; 真做时按需扩 union 或
//   新加字段 (跟 decoded_inst_t 字段不一一对应, IR 视角融合).
// ----------------------------------------------------------------------------
typedef struct {
    ir_op_kind_t kind;
    uxlen_t      target_pc;
    uint8_t      rd;
    uint8_t      rs1;
    uint8_t      rs2;
    int32_t      imm;
} ir_inst_t;


#endif //JIT_IR_H
