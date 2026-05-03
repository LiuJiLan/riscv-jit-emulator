//
// Created by liujilan on 2026/4/28.
// a01_3 decode 模块实现 (纯函数; 不读 / 不写 cpu_t)。
//
// 顶部模块文档见 decode.h。
//
// 算术语义对照表 (摘自 mini-rv32ima 比对):
//   - I-type 12 位 imm 符号扩展: (int32_t)inst >> 20 (依赖 C 算术右移对 signed 类型符号扩展)
//   - U-type imm: inst & 0xfffff000 (高 20 位即 imm[31:12], 低 12 位 0)
//   - SLLI/SRLI/SRAI shamt: rs2 字段 5 位 (RV32 规范要求 shamt[4:0])
//   - SUB/ADD 共享 funct3=0, 用 funct7=0x20 区分
//   - SRA/SRL 共享 funct3=5, 同样用 funct7=0x20 区分
//

#include "decode.h"

#include <stdint.h>

// ----------------------------------------------------------------------------
// decode_rvc —— 16-bit C 扩展 (compressed) 指令的 decode 路径
//
// a_01_3 起步 (用户拍 viii): 只翻译 C.LI + C.ADDI 两个最常用 RVC 算术指令到 RV32I
// op_kind (decoded_inst_t 复用 RV32I 的 OP_ADDI 等, 不为 RVC 单立 op_kind enum —
// RVC 是"指令长度变化", 不是"语义变化", 与 RV32I 同源)。
// 其他 RVC 指令暂时归 OP_UNSUPPORTED + pc_step=PC_STEP_RVC (fetch loop 仍 +2 推进
// 位置正确, 但解释器到这条立即 break)。
//
// RVC 编码 (RISC-V Spec Vol I §16):
//   inst[1:0] != 11 → 16-bit RVC; inst[1:0] = 11 → 32-bit (或更长)
//   inst[15:13] + inst[1:0] 是 RVC 的 opcode 分类 (C0/C1/C2 三个 quadrant × 8 个 funct3)
//
// 本 helper 拿 16-bit inst (低 16 位有效, 高 16 位是 fetch over-read 残值)。
// ----------------------------------------------------------------------------
static decoded_inst_t decode_rvc(uint16_t inst) {
    decoded_inst_t d;
    d.raw_inst = (uint32_t)inst;        // 低 16 位有效
    d.kind     = OP_UNSUPPORTED;
    d.rd       = 0;
    d.rs1      = 0;
    d.rs2      = 0;
    d.imm      = 0;
    d.pc_step  = PC_STEP_RVC;            // 16-bit 指令统一 pc 步进 2

    const uint32_t op    = inst & 0x3u;          // C 扩展 quadrant (0/1/2)
    const uint32_t funct3 = (inst >> 13) & 0x7u; // 类别

    // ---- C1 quadrant (op = 01) ----
    if (op == 0x1) {
        // C.ADDI (funct3=000): rd != 0; addi rd, rd, sign-ext(imm6)
        // C.LI   (funct3=010): rd != 0; addi rd, x0, sign-ext(imm6)
        // imm6 编码: bit[12]=imm[5] (sign), bit[6:2]=imm[4:0]
        if (funct3 == 0x0 || funct3 == 0x2) {
            const uint32_t rd = (inst >> 7) & 0x1Fu;
            if (rd == 0) return d;          // C.ADDI rd=0 是 C.NOP (这里也归 unsupported,
                                            // 真要支持 C.NOP 需 funct3=0 + rd=0 + imm=0
                                            // 翻译为 OP_ADDI x0, x0, 0; 留未来)
            // imm6 sign-ext to 32-bit
            uint32_t imm5 = (inst >> 12) & 0x1u;        // bit 5
            uint32_t imm4_0 = (inst >> 2) & 0x1Fu;      // bit 4..0
            int32_t imm = (int32_t)(imm5 << 5 | imm4_0);
            if (imm5) imm |= (int32_t)0xFFFFFFC0u;      // sign-ext bit 6+

            d.kind = OP_ADDI;
            d.rd   = rd;
            d.rs1  = (funct3 == 0x0) ? rd : 0u;         // C.ADDI: rs1=rd; C.LI: rs1=0
            d.imm  = imm;
            return d;
        }
        // 其他 C1 子类 (C.JAL / C.J / C.BEQZ / C.BNEZ / C.LUI / C.SRLI / C.SRAI / ...)
        // 留 OP_UNSUPPORTED, 真做时按 funct3 + 子位段加 case
        return d;
    }

    // C0 / C2 quadrant 全部留 OP_UNSUPPORTED (load/store/jr/jalr/mv/add 等真做时加)
    return d;
}

decoded_inst_t decode(uint32_t inst) {
    // RVC (16-bit) 分流: inst[1:0] != 11 → 走 decode_rvc 路径
    if ((inst & 0x3u) != 0x3u) {
        return decode_rvc((uint16_t)inst);
    }

    // 32-bit 普通 RV 路径 (a_01_3 现有)
    decoded_inst_t d;
    d.raw_inst = inst;
    d.kind     = OP_UNSUPPORTED;     // 默认; 各 case 命中再覆盖
    d.rd       = (inst >> 7)  & 0x1Fu;
    d.rs1      = (inst >> 15) & 0x1Fu;
    d.rs2      = (inst >> 20) & 0x1Fu;
    d.imm      = 0;                  // 默认; 各 type 命中再覆盖
    d.pc_step  = PC_STEP_RV;          // 32-bit 普通指令默认 +4; control flow 在 case override

    const uint32_t opcode = inst & 0x7Fu;
    const uint32_t funct3 = (inst >> 12) & 0x7u;
    const uint32_t funct7 = (inst >> 25) & 0x7Fu;

    switch (opcode) {
        // ---- U-type ----
        case 0x37:  // LUI
            d.kind = OP_LUI;
            d.imm  = (int32_t)(inst & 0xFFFFF000u);
            break;

        case 0x17:  // AUIPC
            d.kind = OP_AUIPC;
            d.imm  = (int32_t)(inst & 0xFFFFF000u);
            break;

        // ---- I-type OP-IMM ----
        case 0x13: {
            // I-type 12 位 imm 符号扩展到 32 位
            // ((int32_t)inst) >> 20 利用 signed 算术右移做 sign-extend; C 标准 6.5.7p5
            // 对 signed >> 是 implementation-defined, 但 GCC / Clang 在 -fsanitize=undefined
            // 下都按算术右移实现。CMakeLists 里 GCC 编译, 这条 portable enough。
            const int32_t simm = ((int32_t)inst) >> 20;
            d.imm = simm;
            switch (funct3) {
                case 0: d.kind = OP_ADDI;  break;
                case 2: d.kind = OP_SLTI;  break;
                case 3: d.kind = OP_SLTIU; break;
                case 4: d.kind = OP_XORI;  break;
                case 6: d.kind = OP_ORI;   break;
                case 7: d.kind = OP_ANDI;  break;
                case 1:  // SLLI: shamt 占 imm 低 5 位 (= rs2 字段)
                    d.kind = OP_SLLI;
                    d.imm  = (int32_t)d.rs2;
                    break;
                case 5:  // SRLI / SRAI 共享 funct3=5, 用 funct7=0x20 区分
                    d.kind = (funct7 == 0x20u) ? OP_SRAI : OP_SRLI;
                    d.imm  = (int32_t)d.rs2;
                    break;
                default:
                    // funct3 是 3 位 (0..7), 上面 0/1/2/3/4/5/6/7 全部覆盖, 不可达。
                    // 仅作防御。switch on uint32_t 不受 -Wswitch-enum 约束。
                    d.kind = OP_UNSUPPORTED;
                    break;
            }
            break;
        }

        // ---- R-type OP ----
        case 0x33:
            switch (funct3) {
                case 0: d.kind = (funct7 == 0x20u) ? OP_SUB : OP_ADD; break;  // ADD/SUB 共 funct3=0
                case 1: d.kind = OP_SLL;  break;
                case 2: d.kind = OP_SLT;  break;
                case 3: d.kind = OP_SLTU; break;
                case 4: d.kind = OP_XOR;  break;
                case 5: d.kind = (funct7 == 0x20u) ? OP_SRA : OP_SRL; break;  // SRL/SRA 共 funct3=5
                case 6: d.kind = OP_OR;   break;
                case 7: d.kind = OP_AND;  break;
                default:
                    // 同 OP-IMM, funct3 0..7 全覆盖, 不可达, 仅防御。
                    d.kind = OP_UNSUPPORTED;
                    break;
            }
            break;

        // ---- 其他 opcode ----
        // 0x03 LOAD / 0x23 STORE / 0x63 BRANCH / 0x67 JALR / 0x6F JAL / 0x0F FENCE /
        // 0x73 SYSTEM (CSR / ECALL / EBREAK / MRET / SRET / WFI) / 0x2F AMO / 真非法 opcode
        // 全部走默认的 OP_UNSUPPORTED, 不覆盖 d.kind。
        default:
            // 已经初始化为 OP_UNSUPPORTED, 保持。
            break;
    }

    return d;
}
