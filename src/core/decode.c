//
// Created by liujilan on 2026/4/28.
// decode 模块实现 (纯函数; 不读 / 不写 cpu_t)。
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
    // C0 quadrant (op = 00) —— C.ADDI4SPN / C.LW / C.SW
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
        // C.LW (funct3=010, CL-format) / C.SW (funct3=110, CS-format):
        //   inst[12:10] = uimm[5:3]
        //   inst[9:7]   = rs1' (rs1 = rs1' + 8)
        //   inst[6]     = uimm[2]
        //   inst[5]     = uimm[6]
        //   inst[4:2]   = rd' (LW) / rs2' (SW); +8 扩展到 x8-x15
        //   uimm 7-bit unsigned, [1:0]=0 (4-byte align), range [0, 124]
        //   翻译: OP_LW rd=rd', rs1=rs1', imm=uimm
        //        OP_SW rs1=rs1', rs2=rs2', imm=uimm
        //   pc_step = PC_STEP_RVC (load/store 不是 control flow, 顺序推进)
        if (funct3 == 0x2 || funct3 == 0x6) {
            const uint32_t uimm =
                  ((inst >> 10) & 0x7u) << 3      // uimm[5:3]
                | ((inst >> 6)  & 0x1u) << 2      // uimm[2]
                | ((inst >> 5)  & 0x1u) << 6;     // uimm[6]
            const uint32_t rs1_p = ((inst >> 7) & 0x7u) + 8;
            const uint32_t op_p  = ((inst >> 2) & 0x7u) + 8;   // rd' (LW) or rs2' (SW)
            if (funct3 == 0x2) {
                d.kind = OP_LW;
                d.rd   = op_p;
                d.rs1  = rs1_p;
                d.imm  = (int32_t)uimm;
            } else {
                d.kind = OP_SW;
                d.rs1  = rs1_p;
                d.rs2  = op_p;
                d.imm  = (int32_t)uimm;
            }
            return d;
        }

        // 其他 C0 子段: F/D 扩展 (RVC 不支持) 或 reserved, 全 OP_UNSUPPORTED
        return d;
    }

    // ========================================================================
    // C1 quadrant (op = 01) —— 算术 + sp + 控制流 (branch / jump)
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
                if (imm5) { imm |= (int32_t)0xFFFFFFC0u; }
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

        // C.JAL (funct3=001) / C.J (funct3=101, CJ-format): jump-and-link / jump
        //   inst[12]    = imm[11] (sign)
        //   inst[11]    = imm[4]
        //   inst[10:9]  = imm[9:8]
        //   inst[8]     = imm[10]
        //   inst[7]     = imm[6]
        //   inst[6]     = imm[7]
        //   inst[5:3]   = imm[3:1]
        //   inst[2]     = imm[5]
        //   imm[0]      = 0 (隐含 2-byte align)
        //   翻译: OP_JAL rd = (funct3==1 ? x1 : x0), imm12
        //         pc_step = PC_STEP_RVC (case 内 ra = pc + d.pc_step + goto out 跳过 fetch
        //         loop 末段, 跟 32-bit JAL 同形态; ra = pc + 2 for RVC, 跟 RV spec 一致)
        //   注: C.JAL 是 RV32-only (RV64 此位段是 C.ADDIW); 项目当前 RV32, 不冲突
        if (funct3 == 0x1 || funct3 == 0x5) {
            int32_t offset = (int32_t)(
                  ((inst >> 12) & 0x1u) << 11    // imm[11] (sign)
                | ((inst >> 11) & 0x1u) << 4     // imm[4]
                | ((inst >> 9)  & 0x3u) << 8     // imm[9:8]
                | ((inst >> 8)  & 0x1u) << 10    // imm[10]
                | ((inst >> 7)  & 0x1u) << 6     // imm[6]
                | ((inst >> 6)  & 0x1u) << 7     // imm[7]
                | ((inst >> 3)  & 0x7u) << 1     // imm[3:1]
                | ((inst >> 2)  & 0x1u) << 5);   // imm[5]
            if (offset & (1 << 11)) offset |= (int32_t)0xFFFFF000u;  // sign-ext bit 12+
            d.kind    = OP_JAL;
            d.rd      = (funct3 == 0x1) ? 1u : 0u;   // C.JAL: x1 (ra); C.J: x0
            d.imm     = offset;
            d.pc_step = PC_STEP_RVC;
            return d;
        }

        // C.BEQZ / C.BNEZ (funct3=110/111, CB-format): branch if rs1' == 0 / != 0
        //   inst[12]    = offset[8] (sign)
        //   inst[11:10] = offset[4:3]
        //   inst[9:7]   = rs1' (rs1 = rs1' + 8)
        //   inst[6:5]   = offset[7:6]
        //   inst[4:3]   = offset[2:1]
        //   inst[2]     = offset[5]
        //   offset[0]   = 0 (隐含 2-byte align, 跟 RV B-type imm 同形态)
        //   翻译: BEQ/BNE rs1', x0, offset (decode_rvc 复用 OP_BEQ/OP_BNE; pc_step =
        //   PC_STEP_NONE control flow case 自写 pc; BRANCH_IF 宏内 not-taken 根据 raw_inst
        //   [1:0] 区分 RVC pc+2 / 32-bit pc+4)
        if (funct3 == 0x6 || funct3 == 0x7) {
            int32_t offset = (int32_t)(
                  ((inst >> 12) & 0x1u) << 8     // offset[8] (sign)
                | ((inst >> 10) & 0x3u) << 3     // offset[4:3]
                | ((inst >> 5)  & 0x3u) << 6     // offset[7:6]
                | ((inst >> 3)  & 0x3u) << 1     // offset[2:1]
                | ((inst >> 2)  & 0x1u) << 5);   // offset[5]
            if (offset & (1 << 8)) offset |= (int32_t)0xFFFFFE00u;  // sign-ext bit 9+
            const uint32_t rs1_p = ((inst >> 7) & 0x7u) + 8;
            d.kind    = (funct3 == 0x6) ? OP_BEQ : OP_BNE;
            d.rs1     = rs1_p;
            d.rs2     = 0;                       // x0 (BEQZ/BNEZ 跟 0 比)
            d.imm     = offset;
            d.pc_step = PC_STEP_RVC;             // 实际指令长度 (跟 32-bit branch 同形态;
                                                 // BRANCH_IF 宏 taken goto out / not-taken
                                                 // 走 fetch loop 末段 += d.pc_step)
            return d;
        }

        // 其他 C1 子段未覆盖: 留 OP_UNSUPPORTED
        return d;
    }

    // ========================================================================
    // C2 quadrant (op = 10) —— C.SLLI / 控制流 + 算术 / load/store
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
        //     [12]=0 + rs2=0: C.JR
        //     [12]=0 + rs2!=0: C.MV       → ADD rd, x0, rs2 (rd!=0; rd=0 reserved)
        //     [12]=1 + rs1=rs2=0: C.EBREAK
        //     [12]=1 + rs1!=0 + rs2=0: C.JALR
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
            // bit12 + rs2=0 子段: C.JR / C.JALR / C.EBREAK
            //   [12]=0 + rs1!=0:        C.JR    → OP_JALR rd=x0, rs1, imm=0
            //   [12]=1 + rs1=0:         C.EBREAK → OP_EBREAK
            //   [12]=1 + rs1!=0:        C.JALR  → OP_JALR rd=x1, rs1, imm=0
            //   [12]=0 + rs1=0:         reserved
            // pc_step = PC_STEP_RVC for C.JR/C.JALR (跟 32-bit JALR 同形态, ra = pc + 2);
            // pc_step = PC_STEP_NONE for C.EBREAK (跟 32-bit EBREAK 同, trap 路径不顺序推进)
            if (bit12 == 0) {
                if (rd_full == 0) return d;              // C.JR rs1=0 reserved
                d.kind    = OP_JALR;
                d.rd      = 0;                           // C.JR: rd=x0 (不写 ra)
                d.rs1     = rd_full;                     // rs1 字段在 inst[11:7]
                d.imm     = 0;
                d.pc_step = PC_STEP_RVC;
                return d;
            }
            // bit12 == 1, rs2 == 0
            if (rd_full == 0) {
                // C.EBREAK
                d.kind    = OP_EBREAK;
                d.pc_step = PC_STEP_NONE;                // trap 路径; 跟 32-bit EBREAK 同
                return d;
            }
            // C.JALR (rs1!=0, rs2=0, bit12=1)
            d.kind    = OP_JALR;
            d.rd      = 1;                               // C.JALR: rd=x1 (ra)
            d.rs1     = rd_full;
            d.imm     = 0;
            d.pc_step = PC_STEP_RVC;
            return d;
        }

        // C.LWSP (funct3=010, CI-format with uimm): lw rd, uimm(x2)
        //   inst[12]    = uimm[5]
        //   inst[11:7]  = rd (5-bit, x1-x31; rd=0 reserved)
        //   inst[6:4]   = uimm[4:2]
        //   inst[3:2]   = uimm[7:6]
        //   uimm 8-bit unsigned, [1:0]=0 (4-byte align), range [0, 252]
        //   翻译: OP_LW rd, rs1=x2 (sp), imm=uimm
        if (funct3 == 0x2) {
            if (rd_full == 0) return d;              // reserved
            const uint32_t uimm =
                  ((inst >> 12) & 0x1u) << 5         // uimm[5]
                | ((inst >> 4)  & 0x7u) << 2         // uimm[4:2]
                | ((inst >> 2)  & 0x3u) << 6;        // uimm[7:6]
            d.kind = OP_LW;
            d.rd   = rd_full;
            d.rs1  = 2;                              // x2 (sp)
            d.imm  = (int32_t)uimm;
            return d;
        }

        // C.SWSP (funct3=110, CSS-format): sw rs2, uimm(x2)
        //   inst[12:9]  = uimm[5:2]
        //   inst[8:7]   = uimm[7:6]
        //   inst[6:2]   = rs2 (5-bit)
        //   uimm 8-bit unsigned, [1:0]=0 (4-byte align), range [0, 252]
        //   翻译: OP_SW rs1=x2 (sp), rs2, imm=uimm
        if (funct3 == 0x6) {
            const uint32_t uimm =
                  ((inst >> 9) & 0xFu) << 2         // uimm[5:2]
                | ((inst >> 7) & 0x3u) << 6;        // uimm[7:6]
            d.kind = OP_SW;
            d.rs1  = 2;                              // x2 (sp)
            d.rs2  = rs2_full;
            d.imm  = (int32_t)uimm;
            return d;
        }

        // 其他 C2 子段: F/D 扩展 (RVC 不支持) 或 reserved, 全 OP_UNSUPPORTED
        return d;
    }

    return d;
}

decoded_inst_t decode(u32_t inst) {
    // RVC (16-bit) 分流: inst[1:0] != 11 → 走 decode_rvc 路径
    if ((inst & 0x3u) != 0x3u) {
        return decode_rvc((uint16_t)inst);
    }

    // 32-bit 普通 RV 路径
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
        // funct7 严格 dispatch (RV ISA Manual Vol I Table 24.1 + M-ext Vol I §7.1):
        //   funct7 == 0x00: RV32I 8 条 (ADD/SLL/SLT/SLTU/XOR/SRL/OR/AND)
        //   funct7 == 0x20: RV32I 子集 SUB (funct3=0) / SRA (funct3=5); 其他 funct3 reserved
        //   funct7 == 0x01: RV32M 8 条 (MUL/MULH/MULHSU/MULHU/DIV/DIVU/REM/REMU)
        //   其他 funct7:    decode 归 OP_UNSUPPORTED
        //
        // funct7 严格 dispatch 起因: 老 switch 只查 funct3 + funct3=0/5 内嵌 funct7=0x20
        //   判 SUB/SRA, 其他 funct3 完全不查 funct7 → funct7=1 的 M ext 落到
        //   ADD/SLL/SLT/.../AND 路径跑乱数据 (silent miscompile)。现 funct7 严格 dispatch
        //   + M ext 8 条落地。
        case 0x33:
            switch (funct7) {
                case 0x00u:
                    switch (funct3) {
                        case 0: d.kind = OP_ADD;  break;
                        case 1: d.kind = OP_SLL;  break;
                        case 2: d.kind = OP_SLT;  break;
                        case 3: d.kind = OP_SLTU; break;
                        case 4: d.kind = OP_XOR;  break;
                        case 5: d.kind = OP_SRL;  break;
                        case 6: d.kind = OP_OR;   break;
                        case 7: d.kind = OP_AND;  break;
                        default:
                            // funct3 0..7 全覆盖, 不可达; 仅防御。
                            d.kind = OP_UNSUPPORTED;
                            break;
                    }
                    break;
                case 0x20u:
                    // 仅 funct3=0 (SUB) / funct3=5 (SRA) 合法; 其他 funct3 reserved。
                    switch (funct3) {
                        case 0: d.kind = OP_SUB; break;
                        case 5: d.kind = OP_SRA; break;
                        default:
                            d.kind = OP_UNSUPPORTED;
                            break;
                    }
                    break;
                case 0x01u:
                    // RV32M 整数乘除; 详 decode.h enum 段 M ext doc + spec edge cases。
                    switch (funct3) {
                        case 0: d.kind = OP_MUL;    break;
                        case 1: d.kind = OP_MULH;   break;
                        case 2: d.kind = OP_MULHSU; break;
                        case 3: d.kind = OP_MULHU;  break;
                        case 4: d.kind = OP_DIV;    break;
                        case 5: d.kind = OP_DIVU;   break;
                        case 6: d.kind = OP_REM;    break;
                        case 7: d.kind = OP_REMU;   break;
                        default:
                            // funct3 0..7 全覆盖, 不可达; 仅防御。
                            d.kind = OP_UNSUPPORTED;
                            break;
                    }
                    break;
                default:
                    // 其他 funct7 (RV ISA reserved 或未来 A/F/B-ext 等): OP_UNSUPPORTED。
                    d.kind = OP_UNSUPPORTED;
                    break;
            }
            break;

        // ---- B-type BRANCH ----
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
            /* branch 是 control flow 中的特例 — taken case 自写 pc, not-taken 顺序推进
             * (= pc + 指令长度)。设实际指令长度 (RV=4 / RVC=2 in decode_rvc), BRANCH_IF
             * 宏 taken case goto out 跳过 fetch loop 末段, not-taken 走末段 += d.pc_step
             * 自动推进. */
            d.pc_step = PC_STEP_RV;
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
                    // goto out 路径不依赖 pc_step (trap_raise(2) 长跳, pc_step 字段对
                    // unsupp 也无意义), 不需要 reset。
                    d.kind = OP_UNSUPPORTED;
                    break;
            }
            break;
        }

        // ---- J-type JAL ----
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
            /* JAL d.pc_step = 实际指令长度 (跟 branch 同形态), interpreter case 内 ra =
             * pc + d.pc_step (RV=4 / RVC=2 for C.JAL); case 末 goto out 跳过 fetch loop
             * 末段 += d.pc_step (避免破坏 case 写的 jump target)。 */
            d.pc_step = PC_STEP_RV;
            break;
        }

        // ---- I-type JALR ----
        case 0x67:
            // funct3 != 0 reserved by RV spec; 归 OP_UNSUPPORTED。
            if (funct3 != 0) break;       // d.kind 默认 OP_UNSUPPORTED
            // 立即数 12 位有符号 (与 OP-IMM 同型), 同样用 ((int32_t)inst) >> 20 算术右移做
            // sign-ext。目标地址的 & ~1u mask 不在 decode 做 (decode 是纯函数, 不知 rs1 值);
            // 由 interpreter / translator 在 case 内做 (WRITE_PC_OR_TRAP 的事)。
            d.kind    = OP_JALR;
            d.imm     = ((int32_t)inst) >> 20;
            /* 同 JAL: d.pc_step = 实际指令长度, 兼容 RVC C.JR / C.JALR (pc_step=PC_STEP_RVC) */
            d.pc_step = PC_STEP_RV;
            break;

        // ---- I-type SYSTEM (csr 6 变体 + ECALL/EBREAK/MRET/SRET + SFENCE.VMA) ----
        case 0x73: {
            // SYSTEM opcode 含两类指令:
            //   funct3 = 000: ECALL / EBREAK / MRET / SRET / WFI / SFENCE.VMA / ...
            //                 由 imm[11:0] (或 funct7) 进一步区分
            //   funct3 ∈ {001, 010, 011, 101, 110, 111}: csr 6 变体
            //
            // csr 字段约定 (decode.h enum 段已 doc):
            //   d.imm = csr 12-bit address (inst[31:20]); 无符号扩展到 int32_t (高 20 位 0)
            //   d.rs1 = inst[19:15] (decode 顶部已统一提取):
            //             RW/RS/RC:   rs1 寄存器号
            //             RWI/RSI/RCI: 5-bit zimm (interpreter 不查 regs, 直接用数值)
            //   d.rd / d.pc_step: 走默认 (rd 顶部已提取; pc_step = PC_STEP_RV)
            //
            // csr 是硬边界 (decode.h is_block_boundary_inst → 1, fetch loop 末退出
            // → dispatcher 重派发 pc + 4 进下一块)。
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
                    //   funct7 = 0x09 (= 0b0001001) → SFENCE.VMA  (rs1=vaddr, rs2=asid)
                    //   funct7 = 0x00 / 0x18       → 由 imm[11:0] 区分 ECALL/EBREAK/MRET
                    //
                    // 为什么 sfence.vma 不能用 imm[11:0] switch 识别: imm[4:0] = rs2 是变量
                    // (寄存器号 0..31), imm[11:0] 不固定值, 32 种变种。所以先看 funct7。
                    //
                    // 其他 funct7 + funct3=0 的指令 (WFI=imm 0x105 已实装; FENCE.I 等):
                    // 走 imm switch 走自己的 case, 不识别的归 default → OP_UNSUPPORTED。
                    if (funct7 == 0x09u) {
                        // SFENCE.VMA rs1, rs2
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
                            d.kind    = OP_SRET;        /* 跟 MRET 同形态, 走 _mstatus 的 S 段 */
                            d.pc_step = PC_STEP_NONE;
                            break;
                        case 0x105:
                            /* WFI: Wait For Interrupt. case 内自写 pc (醒来 PC+=4 或走 trap),
                             * NONE 跟 MRET/SRET 同体例; 块边界 (is_block_boundary_inst 返 1)。 */
                            d.kind    = OP_WFI;
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

        // ---- I-type LOAD ----
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

        // ---- S-type STORE ----
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

        // ---- MISC-MEM (FENCE / FENCE.I), opcode 0x0F ----
        // funct3 分流:
        //   000 = FENCE   (memory ordering hint; fm/pred/succ 字段不解 — 256 种组合都退化
        //                    为同一空 helper, 详见 isa/fence.h 双重 cover 论证)
        //   001 = FENCE.I (i-cache flush, Zifencei; 副作用 lrsc_clear_self)
        //   其他 funct3   = reserved → OP_UNSUPPORTED
        //
        // 字段约定: rd / rs1 / rs2 / imm 都不用 (interpreter case 不读); pc_step = PC_STEP_RV.
        // 块边界判定 (is_block_boundary_inst): FENCE → 0, FENCE.I → 1.
        case 0x0F: {
            switch (funct3) {
                case 0: d.kind = OP_FENCE;   break;
                case 1: d.kind = OP_FENCE_I; break;
                default:
                    d.kind = OP_UNSUPPORTED;
                    break;
            }
            break;
        }

        // ---- A 扩展 Zaamo (9 op) + Zalrsc (LR.W / SC.W), opcode 0x2F ----
        // RV Unprivileged Spec Vol I "A" extension (Zaamo + Zalrsc).
        // 同 opcode + funct3=010 (.W; RV32) 由 funct5 (bits[31:27]) 区分:
        //   funct5 = 0x00 → AMOADD.W       funct5 = 0x01 → AMOSWAP.W
        //   funct5 = 0x02 → LR.W           funct5 = 0x03 → SC.W
        //   funct5 = 0x04 → AMOXOR.W       funct5 = 0x08 → AMOOR.W
        //   funct5 = 0x0C → AMOAND.W       funct5 = 0x10 → AMOMIN.W
        //   funct5 = 0x14 → AMOMAX.W       funct5 = 0x18 → AMOMINU.W
        //   funct5 = 0x1C → AMOMAXU.W
        //   其他 funct5    reserved → OP_UNSUPPORTED.
        //
        // LR.W (funct5=0x02): rs2 字段 spec 强制为 0 (RV ISA Vol I §8.2);
        //   非 0 行为 implementation-defined. 项目当前不强制 reject (跟 QEMU 一致),
        //   decode 不读 rs2 也不报 illegal — interpreter LR_W case 也不消费 rs2.
        //
        // funct3 = 011 (.D, RV64) → OP_UNSUPPORTED (项目 RV32, 不实).
        // funct3 其他   reserved → OP_UNSUPPORTED.
        //
        // bits[26:25] aq/rl 字段 Q11 拍全 seq_cst, decode 不读 (op_kind 选择跟 aq/rl 无关).
        //
        // 字段约定 (跟 decode.h AMO 段一致):
        //   d.rd / d.rs1 / d.rs2 — 顶部统一提取, 含义见 decode.h
        //   d.imm = 0           AMO 无 imm; raw_inst 供 mtval 用
        //   d.pc_step = PC_STEP_RV (默认; AMO 不改控制流)
        case 0x2F: {
            const uint32_t funct5 = (inst >> 27) & 0x1Fu;     // bits[31:27]
            if (funct3 != 0x2u) {
                // .D 或 reserved funct3 — RV32 Zaamo 仅 .W
                d.kind = OP_UNSUPPORTED;
                break;
            }
            switch (funct5) {
                case 0x00: d.kind = OP_AMO_ADD_W;   break;
                case 0x01: d.kind = OP_AMO_SWAP_W;  break;
                case 0x02: d.kind = OP_LR_W;        break;
                case 0x03: d.kind = OP_SC_W;        break;
                case 0x04: d.kind = OP_AMO_XOR_W;   break;
                case 0x08: d.kind = OP_AMO_OR_W;    break;
                case 0x0C: d.kind = OP_AMO_AND_W;   break;
                case 0x10: d.kind = OP_AMO_MIN_W;   break;
                case 0x14: d.kind = OP_AMO_MAX_W;   break;
                case 0x18: d.kind = OP_AMO_MINU_W;  break;
                case 0x1C: d.kind = OP_AMO_MAXU_W;  break;
                // 其他 funct5 reserved 走兜底
                default:
                    d.kind = OP_UNSUPPORTED;
                    break;
            }
            break;
        }

        // ---- 其他 opcode ----
        // 真非法 opcode 全部走默认的 OP_UNSUPPORTED.
        default:
            // 已经初始化为 OP_UNSUPPORTED, 保持。
            break;
    }

    return d;
}
