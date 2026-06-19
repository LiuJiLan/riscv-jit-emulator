//
// Created by liujilan on 2026/6/19.
// jit/translator.c —— RV → IR 翻译器实装 (T1+T2 范围: RV32I 算术全集 21 op
// 真翻译 + 其他截断 emit DISPATCH_EXIT 走 interpret 兜底).
//
// 实装顶段 doc 见 translator.h.
//
// 体例对齐:
//   - 取指 memcpy 4 字节 (跟 interpreter.c:226 同; 防 strict-aliasing)
//   - 跨页 check 推进后 cur_pc 进新 page 退 (跟 interpreter.c:896 同体例;
//     page_mask = ~0xFFFu)
//   - 软边界 BLOCK_INST_LIMIT - 1 (留 1 slot 给 DISPATCH_EXIT 收尾)
//   - PC_STEP 用 d.pc_step 而非 hardcoded 4 (T6 加 RVC 时自动适应; T1+T2 RV32I
//     算术全集都是 PC_STEP_RV = 4)
//
// T2 唯一"translator 端有运算"的 op 是 AUIPC: cur_pc + d.imm 合并写进 ir.imm
// (选 a 常量折叠, 跟 QEMU 默认 + rv8 一致; baked_pc 不进 ir_inst_t 字段).
// PA != VA trail: V1 BARE only, cur_pc = pa = VA; SV32 上线 (b_03+) 时
// AUIPC baked_pc 来源 (用 pa 还是 cpu->pc VA) 单独议.
//

#include "translator.h"

#include <stdint.h>
#include <string.h>          // memcpy: 4 字节取指, 防 strict-aliasing

#include "config.h"          // BLOCK_INST_LIMIT
#include "core/decode.h"     // decode + decoded_inst_t + op_kind_t
#include "platform/ram.h"    // gpa_to_hva_offset

/* ============================================================================
 * 跨页 mask (跟 interpreter.c page_mask = ~0xFFFu 同体例)
 * ============================================================================ */
#define TRANSLATOR_PAGE_MASK  (~(uxlen_t)0xFFFu)


void translator_translate(uxlen_t pa, regime_t regime,
                          ir_inst_t *ir_buf, size_t *n_insts) {
    (void)regime;   /* T1 c+ 不用 regime; T3 翻译 LOAD/STORE 时接 SUM/MXR check 形态 */

    /* 块入口 hva = gpa_to_hva_offset + pa (ram.h "省去每次访问的减法"). 块前缀
     * 内每条指令的 hva = hva_base + (cur_pc - pa). */
    uint8_t *hva_base = gpa_to_hva_offset + pa;
    uxlen_t cur_pc = pa;
    size_t i = 0;

    const uxlen_t entry_page = pa & TRANSLATOR_PAGE_MASK;

    /* 翻译循环 (块前缀):
     *   - 软边界 i < BLOCK_INST_LIMIT - 1u (留 1 slot 给 DISPATCH_EXIT 收尾)
     *   - 非 RV32I 算术全集截断 (含 OP_UNSUPPORTED / boundary 指令 / load/store/
     *     csr/amo/lrsc/fence/branch/jal/jalr 等; 推 T3-T6 真做)
     *   - 跨页 check (推进后 cur_pc 进新 page → 退)
     */
    while (i < BLOCK_INST_LIMIT - 1u) {
        u32_t inst;
        memcpy(&inst, hva_base + (cur_pc - pa), 4);

        decoded_inst_t d = decode(inst);

        /* ir_kind 默认 IR_OP_UNSUPPORTED 防 release -O2 maybe-uninitialized
         * 假警 (debug -O0 不撞; switch 全覆盖时实际不会读默认值, 但 GCC -O2
         * 静态分析推不出来). */
        ir_op_kind_t ir_kind = IR_OP_UNSUPPORTED;
        int32_t      ir_imm  = d.imm;           /* 默认沿用 d.imm */

        switch (d.kind) {
            /* ---- U-type ---- */
            case OP_LUI:    ir_kind = IR_OP_LUI;   break;
            case OP_AUIPC:  ir_kind = IR_OP_AUIPC;
                            /* 选 a 常量折叠: backend 跟 LUI 同 mov 形态.
                             * cur_pc 在 V1 BARE 下 = pa = VA. */
                            ir_imm  = (int32_t)(cur_pc + (uint32_t)d.imm);
                            break;

            /* ---- I-type OP-IMM ---- */
            case OP_ADDI:   ir_kind = IR_OP_ADDI;  break;
            case OP_SLTI:   ir_kind = IR_OP_SLTI;  break;
            case OP_SLTIU:  ir_kind = IR_OP_SLTIU; break;
            case OP_XORI:   ir_kind = IR_OP_XORI;  break;
            case OP_ORI:    ir_kind = IR_OP_ORI;   break;
            case OP_ANDI:   ir_kind = IR_OP_ANDI;  break;
            case OP_SLLI:   ir_kind = IR_OP_SLLI;  break;
            case OP_SRLI:   ir_kind = IR_OP_SRLI;  break;
            case OP_SRAI:   ir_kind = IR_OP_SRAI;  break;

            /* ---- R-type OP ---- */
            case OP_ADD:    ir_kind = IR_OP_ADD;   break;
            case OP_SUB:    ir_kind = IR_OP_SUB;   break;
            case OP_SLL:    ir_kind = IR_OP_SLL;   break;
            case OP_SLT:    ir_kind = IR_OP_SLT;   break;
            case OP_SLTU:   ir_kind = IR_OP_SLTU;  break;
            case OP_XOR:    ir_kind = IR_OP_XOR;   break;
            case OP_SRL:    ir_kind = IR_OP_SRL;   break;
            case OP_SRA:    ir_kind = IR_OP_SRA;   break;
            case OP_OR:     ir_kind = IR_OP_OR;    break;
            case OP_AND:    ir_kind = IR_OP_AND;   break;

            /* 非 RV32I 算术 — 块前缀截断, 走 DISPATCH_EXIT(cur_pc) 收尾让
             * dispatcher 下一轮重派发 (BLACK 时由 interpret 兜底). 显式列全
             * (而非走 default) 是因为 -Wswitch-enum -Werror 强制 op_kind_t
             * switch 同步; CLAUDE.md 顶段 "load-bearing mechanism" 段. */
            /* RV32M (推 T6 真做 emit) */
            case OP_MUL:    case OP_MULH:   case OP_MULHSU: case OP_MULHU:
            case OP_DIV:    case OP_DIVU:   case OP_REM:    case OP_REMU:
            /* Branch / JAL / JALR 控制流 (推 T4) */
            case OP_BEQ:    case OP_BNE:    case OP_BLT:    case OP_BGE:
            case OP_BLTU:   case OP_BGEU:
            case OP_JAL:    case OP_JALR:
            /* CSR (推 T3; 硬边界 — interpreter 同) */
            case OP_CSRRW:  case OP_CSRRS:  case OP_CSRRC:
            case OP_CSRRWI: case OP_CSRRSI: case OP_CSRRCI:
            /* SYSTEM (推 T3; 硬边界) */
            case OP_ECALL:  case OP_EBREAK: case OP_MRET:   case OP_SRET:
            case OP_SFENCE_VMA: case OP_WFI:
            /* LOAD / STORE (推 T3) */
            case OP_LB:     case OP_LH:     case OP_LW:
            case OP_LBU:    case OP_LHU:
            case OP_SB:     case OP_SH:     case OP_SW:
            /* MISC-MEM (推 T3; FENCE.I 硬边界) */
            case OP_FENCE:  case OP_FENCE_I:
            /* Zaamo 9 op (推 T3) */
            case OP_AMO_ADD_W:  case OP_AMO_SWAP_W: case OP_AMO_XOR_W:
            case OP_AMO_OR_W:   case OP_AMO_AND_W:
            case OP_AMO_MIN_W:  case OP_AMO_MAX_W:
            case OP_AMO_MINU_W: case OP_AMO_MAXU_W:
            /* Zalrsc (推 T3) */
            case OP_LR_W:   case OP_SC_W:
            /* 兜底 (后续 interpret 走 trap_raise cause=2) */
            case OP_UNSUPPORTED:
                goto out_translate;
        }

        ir_buf[i].kind = ir_kind;
        ir_buf[i].rd   = (uint8_t)d.rd;
        ir_buf[i].rs1  = (uint8_t)d.rs1;
        ir_buf[i].rs2  = (uint8_t)d.rs2;
        ir_buf[i].imm  = ir_imm;
        i++;

        cur_pc += d.pc_step;
        if ((cur_pc & TRANSLATOR_PAGE_MASK) != entry_page) {
            break;
        }
    }
out_translate:;

    /* 块出口 DISPATCH_EXIT 收尾 (target_pc = cur_pc = 截断指令 PC / 软边界出块
     * PC / 跨页边界 PC); backend emit 时编 "写 cpu->pc=target_pc + 写 count_out
     * + epilogue + ret".
     *
     * 块前缀空 (i == 0): target_pc = pa (跟块入口同); backend.compile_block 检
     * 测 n_insts == 1 时返 NOT_IMPLEMENTED, jit_entry.cc set_blacklist 让
     * interpret 兜底真执行那条 RV.
     *
     * rd/rs1/rs2/imm 清 0 是兜底 (backend default case 只读 target_pc, 这些字段
     * 不读; 清 0 防 stale 数据). */
    ir_buf[i].kind      = IR_OP_DISPATCH_EXIT;
    ir_buf[i].target_pc = cur_pc;
    ir_buf[i].rd        = 0;
    ir_buf[i].rs1       = 0;
    ir_buf[i].rs2       = 0;
    ir_buf[i].imm       = 0;
    i++;

    *n_insts = i;
}
