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
