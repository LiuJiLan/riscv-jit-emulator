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
// 演进:
//   a_01_3 起步: 只翻译 C.LI + C.ADDI 两个 RVC 算术 case (用户拍 viii)
//   a_01_10 (7) step 1: 加完整 RVC 算术 + sp 设置 case (~16 case 含展开 SUB/XOR/OR/AND);
//                       控制流 (C.JAL/J/JR/JALR/BEQZ/BNEZ/EBREAK) 留 step 2-3, load/store
//                       (C.LW/SW/LWSP/SWSP) 留 step 4
//
// 所有 RVC 复用现有 RV32I op_kind, 不为 RVC 单立 op_kind enum — RVC 是"指令长度变化",
// 不是"语义变化", 与 RV32I 同源 (decoded_inst_t.pc_step = PC_STEP_RVC = 2 区分 fetch
// loop 推进; OP_xxx 跟 32-bit 路径一致, interpreter / 未来 translator 不感知 RVC)。
//
// RVC 编码 (RISC-V Spec Vol I §16):
//   inst[1:0] != 11 → 16-bit RVC; inst[1:0] = 11 → 32-bit (或更长)
//   inst[15:13] + inst[1:0] 是 RVC 的 opcode 分类 (C0/C1/C2 三个 quadrant × 8 个 funct3)
//
// 3-bit 寄存器编码 (rd' / rs1' / rs2' = inst 子段 + 8, 范围 x8-x15 子集):
//   inst[9:7]  → rd' / rs1' (CIW/CL/CS/CB-format)
//   inst[4:2]  → rs2' (CL/CS/CA-format) 或 rd' (CIW-format)
//   解码: ((inst >> 9) & 0x7) + 8 / ((inst >> 2) & 0x7) + 8
//
// imm 解码: 每条 RVC 指令 imm 的位段拼装规则不同, 需仔细对照 spec § 16 各 format
//   (CI/CIW/CL/CS/CB/CA/CJ/CR), 详见每 case 内 spec 引用注释。
//
// 边界 case 处理:
//   - C.NOP (funct3=000+rd=0+imm=0): 显式 NOP 语义 → OP_ADDI x0, x0, 0
//   - rd=0 hint reserved (C.LI / C.SRLI 等): 留 OP_UNSUPPORTED (项目不实现 hint)
//   - nzimm/nzuimm = 0 reserved (C.LUI / C.ADDI4SPN / C.ADDI16SP): 留 OP_UNSUPPORTED
//   - RV32 shamt[5]=1 reserved (C.SLLI / C.SRLI / C.SRAI inst[12]=1): 留 OP_UNSUPPORTED
//   - C.SUB/XOR/OR/AND inst[12]=1 是 RV64 SUBW/ADDW: RV32 reserved, 留 OP_UNSUPPORTED
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

    const uint32_t op     = inst & 0x3u;            // C 扩展 quadrant (0/1/2)
    const uint32_t funct3 = (inst >> 13) & 0x7u;    // 类别

    // ========================================================================
    // C0 quadrant (op = 00) —— C.ADDI4SPN / (C.LW step 4) / (C.SW step 4)
    // ========================================================================
    if (op == 0x0) {
        // C.ADDI4SPN (funct3=000, CIW-format): addi rd', x2, nzuimm10
        //   inst[12:11] = nzuimm[5:4]
        //   inst[10:7]  = nzuimm[9:6]
        //   inst[6]     = nzuimm[2]
        //   inst[5]     = nzuimm[3]
        //   inst[4:2]   = rd' (rd = rd' + 8)
        //   nzuimm = 0 reserved (spec § 16.5)
        if (funct3 == 0x0) {
            const uint32_t nzuimm =
                  ((inst >> 11) & 0x3u) << 4    // nzuimm[5:4]
                | ((inst >> 7)  & 0xFu) << 6    // nzuimm[9:6]
                | ((inst >> 6)  & 0x1u) << 2    // nzuimm[2]
                | ((inst >> 5)  & 0x1u) << 3;   // nzuimm[3]
            if (nzuimm == 0) return d;          // reserved
            const uint32_t rd_p = ((inst >> 2) & 0x7u) + 8;
            d.kind = OP_ADDI;
            d.rd   = rd_p;
            d.rs1  = 2;                          // x2 = sp
            d.imm  = (int32_t)nzuimm;            // unsigned 10-bit, 高位 0 自然
            return d;
        }
        // 其他 C0 子段: C.LW (funct3=010) / C.SW (funct3=110) — step 4 加
        return d;
    }

    // ========================================================================
    // C1 quadrant (op = 01) —— 算术 + sp + 控制流 (控制流 step 2/3 加)
    // ========================================================================
    if (op == 0x1) {
        const uint32_t rd_full = (inst >> 7) & 0x1Fu;
        const uint32_t imm5    = (inst >> 12) & 0x1u;     // CI imm[5] = inst[12]
        const uint32_t imm4_0  = (inst >> 2)  & 0x1Fu;    // CI imm[4:0] = inst[6:2]

        // C.NOP / C.ADDI / C.LI (funct3=000 / 010, CI-format):
        //   funct3=000: rd=0+imm=0 → C.NOP (OP_ADDI x0,x0,0); rd=0+imm!=0 hint reserved;
        //               rd!=0 → C.ADDI rd, rd, sign-ext(imm6)
        //   funct3=010: rd=0 hint reserved; rd!=0 → C.LI: addi rd, x0, sign-ext(imm6)
        if (funct3 == 0x0 || funct3 == 0x2) {
            int32_t imm = (int32_t)(imm5 << 5 | imm4_0);
            if (imm5) imm |= (int32_t)0xFFFFFFC0u;        // sign-ext bit 6+

            if (funct3 == 0x0 && rd_full == 0) {
                // C.NOP 边界 (spec § 16.5: rd=0+imm=0 显式 NOP; rd=0+imm!=0 hint reserved)
                if (imm != 0) return d;                   // hint reserved
                d.kind = OP_ADDI;
                d.rd   = 0;
                d.rs1  = 0;
                d.imm  = 0;
                return d;
            }
            if (rd_full == 0) return d;                   // C.LI rd=0 hint reserved
            d.kind = OP_ADDI;
            d.rd   = rd_full;
            d.rs1  = (funct3 == 0x0) ? rd_full : 0u;      // C.ADDI rs1=rd; C.LI rs1=0
            d.imm  = imm;
            return d;
        }

        // C.ADDI16SP / C.LUI (funct3=011, CI-format):
        //   rd = 2  → C.ADDI16SP: addi x2, x2, nzimm10
        //             imm[9]   = inst[12]
        //             imm[4]   = inst[6]
        //             imm[6]   = inst[5]
        //             imm[8:7] = inst[4:3]
        //             imm[5]   = inst[2]
        //             nzimm = 0 reserved
        //   rd != 0,2 → C.LUI: lui rd, nzimm18 (sign-ext)
        //             imm[17]    = inst[12]
        //             imm[16:12] = inst[6:2]
        //             nzimm = 0 reserved (项目不实现 hint)
        //   rd = 0 reserved
        if (funct3 == 0x3) {
            if (rd_full == 0) return d;                   // reserved
            if (rd_full == 2) {
                // C.ADDI16SP
                int32_t nzimm = (int32_t)(
                      ((inst >> 12) & 0x1u) << 9         // imm[9] (sign)
                    | ((inst >> 6)  & 0x1u) << 4         // imm[4]
                    | ((inst >> 5)  & 0x1u) << 6         // imm[6]
                    | ((inst >> 3)  & 0x3u) << 7         // imm[8:7]
                    | ((inst >> 2)  & 0x1u) << 5);       // imm[5]
                if (nzimm == 0) return d;                // reserved
                if (nzimm & (1 << 9)) nzimm |= (int32_t)0xFFFFFC00u;  // sign-ext bit 10+
                d.kind = OP_ADDI;
                d.rd   = 2;
                d.rs1  = 2;
                d.imm  = nzimm;
                return d;
            }
            // C.LUI
            int32_t nzimm = (int32_t)(
                  ((inst >> 12) & 0x1u) << 17            // imm[17] (sign)
                | ((inst >> 2)  & 0x1Fu) << 12);         // imm[16:12]
            if (nzimm == 0) return d;                    // reserved
            if (nzimm & (1 << 17)) nzimm |= (int32_t)0xFFFC0000u;  // sign-ext bit 18+
            d.kind = OP_LUI;
            d.rd   = rd_full;
            d.rs1  = 0;                                  // LUI 无 rs1
            d.imm  = nzimm;
            return d;
        }

        // C.MISC-ALU (funct3=100): 子段按 inst[11:10]
        //   00: C.SRLI (rd', rd', shamt6)
        //   01: C.SRAI (rd', rd', shamt6)
        //   10: C.ANDI (rd', rd', sign-ext imm6)
        //   11: C.SUB / C.XOR / C.OR / C.AND (rd', rd', rs2'; 由 inst[12]/[6:5] 区分)
        //              inst[12]=1 是 RV64 C.SUBW/C.ADDW reserved
        if (funct3 == 0x4) {
            const uint32_t sub  = (inst >> 10) & 0x3u;
            const uint32_t rd_p = ((inst >> 7) & 0x7u) + 8;

            if (sub == 0x0 || sub == 0x1) {
                // C.SRLI / C.SRAI (CB-format with shamt):
                //   shamt[5]   = inst[12]
                //   shamt[4:0] = inst[6:2]
                //   RV32: shamt[5] 必 0 (spec § 16.5 RV32 reserved)
                if (imm5 != 0) return d;                 // RV32 shamt[5]=1 reserved
                const uint32_t shamt = imm4_0;
                d.kind = (sub == 0x0) ? OP_SRLI : OP_SRAI;
                d.rd   = rd_p;
                d.rs1  = rd_p;
                d.imm  = (int32_t)shamt;
                return d;
            }
            if (sub == 0x2) {
                // C.ANDI (rd', rd', sign-ext imm6); imm 解码跟 C.ADDI 同
                int32_t imm = (int32_t)(imm5 << 5 | imm4_0);
                if (imm5) imm |= (int32_t)0xFFFFFFC0u;
                d.kind = OP_ANDI;
                d.rd   = rd_p;
                d.rs1  = rd_p;
                d.imm  = imm;
                return d;
            }
            // sub == 0x3: C.SUB / C.XOR / C.OR / C.AND (CA-format)
            //   inst[12]    = 0 (RV32) / 1 (RV64 SUBW/ADDW reserved 在 RV32)
            //   inst[6:5]   = funct (00 SUB / 01 XOR / 10 OR / 11 AND)
            //   inst[9:7]   = rd' = rs1'
            //   inst[4:2]   = rs2'
            if (imm5 != 0) return d;                     // RV32: inst[12]=1 reserved
            const uint32_t rs2_p  = ((inst >> 2) & 0x7u) + 8;
            const uint32_t op_sel = (inst >> 5) & 0x3u;
            switch (op_sel) {
                case 0x0: d.kind = OP_SUB; break;
                case 0x1: d.kind = OP_XOR; break;
                case 0x2: d.kind = OP_OR;  break;
                case 0x3: d.kind = OP_AND; break;
                default:  return d;                      // 不可达 (op_sel 2 位)
            }
            d.rd  = rd_p;
            d.rs1 = rd_p;
            d.rs2 = rs2_p;
            return d;
        }

        // 其他 C1 子类: C.JAL (001) / C.J (101) — step 3 (jump);
        //               C.BEQZ (110) / C.BNEZ (111) — step 2 (branch)
        return d;
    }

    // ========================================================================
    // C2 quadrant (op = 10) —— C.SLLI / 控制流 + 算术 / load/store (后两段后续 step 加)
    // ========================================================================
    if (op == 0x2) {
        const uint32_t rd_full  = (inst >> 7) & 0x1Fu;
        const uint32_t rs2_full = (inst >> 2) & 0x1Fu;

        // C.SLLI (funct3=000, CI-format with shamt): slli rd, rd, shamt6
        //   shamt[5]   = inst[12]
        //   shamt[4:0] = inst[6:2]
        //   inst[11:7] = rd
        //   RV32: shamt[5] 必 0 (reserved); rd = 0 reserved (项目不实现 hint)
        if (funct3 == 0x0) {
            if (rd_full == 0) return d;                  // hint reserved
            const uint32_t shamt5 = (inst >> 12) & 0x1u;
            if (shamt5 != 0) return d;                   // RV32 shamt[5]=1 reserved
            const uint32_t shamt = (inst >> 2) & 0x1Fu;
            d.kind = OP_SLLI;
            d.rd   = rd_full;
            d.rs1  = rd_full;
            d.imm  = (int32_t)shamt;
            return d;
        }

        // C.MV / C.ADD (funct3=100, CR-format): 共用 funct3 跟 C.JR/JALR/EBREAK
        //   inst[12] + rs2 区分:
        //     [12]=0 + rs2=0: C.JR        (step 3)
        //     [12]=0 + rs2!=0: C.MV       → ADD rd, x0, rs2 (rd!=0; rd=0 reserved)
        //     [12]=1 + rs1=rs2=0: C.EBREAK (step 3)
        //     [12]=1 + rs1!=0 + rs2=0: C.JALR (step 3)
        //     [12]=1 + rs2!=0: C.ADD     → ADD rd, rd, rs2 (rd!=0; rd=0 reserved)
        if (funct3 == 0x4) {
            const uint32_t bit12 = (inst >> 12) & 0x1u;
            if (bit12 == 0 && rs2_full != 0) {
                // C.MV
                if (rd_full == 0) return d;              // reserved
                d.kind = OP_ADD;
                d.rd   = rd_full;
                d.rs1  = 0;                              // mv = add rd, x0, rs2
                d.rs2  = rs2_full;
                return d;
            }
            if (bit12 == 1 && rs2_full != 0) {
                // C.ADD
                if (rd_full == 0) return d;              // reserved
                d.kind = OP_ADD;
                d.rd   = rd_full;
                d.rs1  = rd_full;
                d.rs2  = rs2_full;
                return d;
            }
            // bit12 + rs2=0 子段: C.JR / C.JALR / C.EBREAK — step 3 加
            return d;
        }

        // 其他 C2 子段: C.LWSP (010) / C.SWSP (110) — step 4 加
        return d;
    }

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

        // ---- B-type BRANCH ---- a_01_4
        case 0x63: {
            // B-type 13 位有符号立即数 (multiple of 2, imm[0]=0 编码强制), 4 段位拼接 (RV
            // spec Vol I, Conditional Branches 段; J-type 同型, 后面也 4 段):
            //   imm[12]   = inst[31]                 (sign bit)
            //   imm[10:5] = inst[30:25]
            //   imm[4:1]  = inst[11:8]
            //   imm[11]   = inst[7]                  (这一位"塞回去"是为了让 rs1/rs2 字段位置
            //                                         与所有其他指令类型保持一致 - RV 设计哲学)
            //   imm[0]    = 0
            //
            // sign-ext 用 ((int32_t)(inst & 0x80000000u)) >> 19 一次性把 bit 12 + 高位全部
            // 同时填好 (0x80000000 算术右移 19 位 = 0xFFFFF000 = 高 20 位全 1, 含 bit 12);
            // 其他段位置在 [11:0] 不与 sign-ext 部分重叠, 直接 OR。
            const int32_t imm =
                  (((int32_t)(inst & 0x80000000u)) >> 19)              /* sign-ext bit 12+ */
                | (int32_t)(((inst >> 25) & 0x3Fu) << 5)               /* bits 10:5 */
                | (int32_t)(((inst >> 8)  & 0xFu)  << 1)               /* bits 4:1 */
                | (int32_t)(((inst >> 7)  & 0x1u)  << 11);             /* bit 11 */
            d.imm     = imm;
            d.pc_step = PC_STEP_NONE;     /* control flow: case 自描述 pc, fetch loop +=0 NOP */
            switch (funct3) {
                case 0: d.kind = OP_BEQ;  break;
                case 1: d.kind = OP_BNE;  break;
                case 4: d.kind = OP_BLT;  break;
                case 5: d.kind = OP_BGE;  break;
                case 6: d.kind = OP_BLTU; break;
                case 7: d.kind = OP_BGEU; break;
                default:
                    // funct3 = 010 / 011 reserved by RV spec; 归 OP_UNSUPPORTED。
                    // 此时 pc_step 已被设为 PC_STEP_NONE, 但 OP_UNSUPPORTED 走 interpreter
                    // goto out 路径不依赖 pc_step (a_01_5 trap.c 接入后 pc_step 字段对 unsupp
                    // 也无意义), 不需要 reset。
                    d.kind = OP_UNSUPPORTED;
                    break;
            }
            break;
        }

        // ---- J-type JAL ---- a_01_4
        case 0x6F: {
            // J-type 21 位有符号立即数 (multiple of 2, imm[0]=0 编码强制), 4 段位拼接:
            //   imm[20]    = inst[31]                (sign bit)
            //   imm[10:1]  = inst[30:21]
            //   imm[11]    = inst[20]
            //   imm[19:12] = inst[19:12]              (天然在原位, 不 shift)
            //   imm[0]     = 0
            //
            // sign-ext: 0x80000000 算术右移 11 位 = 0xFFF00000 = 高 12 位全 1 (含 bit 20)。
            // imm[19:12] 段直接拿 inst & 0xFF000u (该段在 inst 中的位置就是 imm 中的位置)。
            const int32_t imm =
                  (((int32_t)(inst & 0x80000000u)) >> 11)              /* sign-ext bit 20+ */
                | (int32_t)(((inst >> 21) & 0x3FFu) << 1)              /* bits 10:1 */
                | (int32_t)(((inst >> 20) & 0x1u)   << 11)             /* bit 11 */
                | (int32_t)(inst & 0xFF000u);                          /* bits 19:12, 在原位 */
            d.kind    = OP_JAL;
            d.imm     = imm;
            d.pc_step = PC_STEP_NONE;
            break;
        }

        // ---- I-type JALR ---- a_01_4
        case 0x67:
            // funct3 != 0 reserved by RV spec; 归 OP_UNSUPPORTED。
            if (funct3 != 0) break;       // d.kind 默认 OP_UNSUPPORTED
            // 立即数 12 位有符号 (与 OP-IMM 同型), 同样用 ((int32_t)inst) >> 20 算术右移做
            // sign-ext。目标地址的 & ~1u mask 不在 decode 做 (decode 是纯函数, 不知 rs1 值);
            // 由 interpreter / translator 在 case 内做 (Step 3 WRITE_PC_OR_TRAP 的事)。
            d.kind    = OP_JALR;
            d.imm     = ((int32_t)inst) >> 20;
            d.pc_step = PC_STEP_NONE;
            break;

        // ---- I-type SYSTEM ---- a_01_5_a 加 csr 6 变体
        case 0x73: {
            // SYSTEM opcode 含两类指令:
            //   funct3 = 000: ECALL / EBREAK / MRET / SRET / WFI / SFENCE.VMA / ...
            //                 由 imm[11:0] 进一步区分 (a_01_5_c 加; 现在归 OP_UNSUPPORTED)
            //   funct3 ∈ {001, 010, 011, 101, 110, 111}: csr 6 变体
            //
            // csr 字段约定 (decode.h enum 段已 doc):
            //   d.imm = csr 12-bit address (inst[31:20]); 无符号扩展到 int32_t (高 20 位 0)
            //   d.rs1 = inst[19:15] (decode 顶部已统一提取):
            //             RW/RS/RC:   rs1 寄存器号
            //             RWI/RSI/RCI: 5-bit zimm (interpreter 不查 regs, 直接用数值)
            //   d.rd / d.pc_step: 走默认 (rd 顶部已提取; pc_step = PC_STEP_RV)
            //
            // csr 是硬边界 (decode.h is_block_boundary_inst Step 2 改 return 1, fetch loop
            // 末退出 → dispatcher 重派发 pc + 4 进下一块)。
            d.imm = (int32_t)((inst >> 20) & 0xFFFu);     /* 无符号 12 位放 imm */
            switch (funct3) {
                case 1: d.kind = OP_CSRRW;  break;
                case 2: d.kind = OP_CSRRS;  break;
                case 3: d.kind = OP_CSRRC;  break;
                case 5: d.kind = OP_CSRRWI; break;
                case 6: d.kind = OP_CSRRSI; break;
                case 7: d.kind = OP_CSRRCI; break;
                case 0:
                    // funct3=000 system 类: 两层区分:
                    //   funct7 = 0x09 (= 0b0001001) → SFENCE.VMA  (rs1=vaddr, rs2=asid; a_01_8)
                    //   funct7 = 0x00 / 0x18       → 由 imm[11:0] 区分 ECALL/EBREAK/MRET
                    //
                    // 为什么 sfence.vma 不能用 imm[11:0] switch 识别: imm[4:0] = rs2 是变量
                    // (寄存器号 0..31), imm[11:0] 不固定值, 32 种变种。所以先看 funct7。
                    //
                    // 其他 funct7 + funct3=0 的指令 (SRET=imm 0x102, WFI=imm 0x105 等):
                    // 走 imm switch 末 default → OP_UNSUPPORTED。a_01 不做 (S-mode / 中断 真做
                    // 时再加 op_kind + case)。
                    if (funct7 == 0x09u) {
                        // SFENCE.VMA rs1, rs2 (a_01_8)
                        // d.rs1, d.rs2 已由 decode 顶部统一提取 (rs1=vaddr 寄存器, rs2=asid 寄存器);
                        // d.rd, d.imm 不用 (RV spec 要求 rd=0; imm 字段被 funct7|rs2 复用, 但 case
                        // 不读, 仅供 raw_inst trap 路径备查)。
                        // d.pc_step = PC_STEP_RV (默认; sfence 不是 control flow, +4 推进; 块边界
                        // 由 is_block_boundary_inst → 1 + fetch loop 末 goto out 处理)。
                        d.kind = OP_SFENCE_VMA;
                        break;
                    }
                    switch ((uint32_t)d.imm) {
                        case 0x000:
                            d.kind    = OP_ECALL;
                            d.pc_step = PC_STEP_NONE;   /* trap 跳 xtvec, 不让 fetch loop 末 += 4 */
                            break;
                        case 0x001:
                            d.kind    = OP_EBREAK;
                            d.pc_step = PC_STEP_NONE;
                            break;
                        case 0x302:
                            d.kind    = OP_MRET;
                            d.pc_step = PC_STEP_NONE;   /* MRET 写 pc=xepc, 不让 fetch loop +4 */
                            break;
                        case 0x102:
                            d.kind    = OP_SRET;        /* a_01_8 Step 6; 跟 MRET 同形态 */
                            d.pc_step = PC_STEP_NONE;
                            break;
                        default:
                            d.kind = OP_UNSUPPORTED;
                            break;
                    }
                    break;
                case 4:
                default:
                    // funct3=100: reserved by RV spec; OP_UNSUPPORTED
                    d.kind = OP_UNSUPPORTED;
                    break;
            }
            break;
        }

        // ---- I-type LOAD ---- a_01_6
        case 0x03: {
            // I-type 12 位有符号立即数 (与 OP-IMM 同型, 复用 ((int32_t)inst) >> 20 算术右移)。
            // 字段 rd / rs1 已由 decode 顶部统一提取; rs2 是 garbage = imm 低 5 位 (顶部提取)。
            // pc_step 默认 PC_STEP_RV; 不动。
            d.imm = ((int32_t)inst) >> 20;
            switch (funct3) {
                case 0: d.kind = OP_LB;   break;
                case 1: d.kind = OP_LH;   break;
                case 2: d.kind = OP_LW;   break;
                case 4: d.kind = OP_LBU;  break;
                case 5: d.kind = OP_LHU;  break;
                case 3: case 6: case 7:
                default:
                    // funct3=011 (LD, RV64) / 110 (LWU, RV64) / 111 reserved by RV32 spec.
                    // 真上 RV64 时再加 op_kind; 现在 OP_UNSUPPORTED。
                    d.kind = OP_UNSUPPORTED;
                    break;
            }
            break;
        }

        // ---- S-type STORE ---- a_01_6
        case 0x23: {
            // S-type 12 位有符号立即数, 由 inst 中两段拼接:
            //   imm[11:5] = inst[31:25]    (高 7 位)
            //   imm[4:0]  = inst[11:7]     (低 5 位)
            // sign-ext 用 ((int32_t)(inst & 0x80000000u)) >> 19 → bit 31:12 全填 sign;
            // 然后 OR 上 inst[31:25] << 5 (覆盖 bit 11:5; bit 11 重复设置无害, sign bit 同值)
            // OR 上 inst[11:7] (bit 4:0)。
            // 这跟 decode.c B-type 的 4 段拼接风格一致。
            const int32_t imm =
                  (((int32_t)(inst & 0x80000000u)) >> 19)            /* sign-ext bit 31:12 */
                | (int32_t)(((inst >> 25) & 0x7Fu) << 5)              /* bits 11:5 */
                | (int32_t)((inst >> 7)  & 0x1Fu);                   /* bits 4:0 */
            d.imm = imm;
            // rs1 / rs2 已由 decode 顶部统一提取 (S-type rs2 在 inst[24:20], 跟通用提取一致);
            // rd 是 garbage = imm[4:0] = inst[11:7] (顶部提取)。pc_step = PC_STEP_RV。
            switch (funct3) {
                case 0: d.kind = OP_SB;   break;
                case 1: d.kind = OP_SH;   break;
                case 2: d.kind = OP_SW;   break;
                case 3: case 4: case 5: case 6: case 7:
                default:
                    // funct3=011 (SD, RV64) / 100..111 reserved by RV32 spec; OP_UNSUPPORTED。
                    d.kind = OP_UNSUPPORTED;
                    break;
            }
            break;
        }

        // ---- 其他 opcode ----
        // 0x0F FENCE / 0x2F AMO / 真非法 opcode 全部走默认的 OP_UNSUPPORTED。
        default:
            // 已经初始化为 OP_UNSUPPORTED, 保持。
            break;
    }

    return d;
}
