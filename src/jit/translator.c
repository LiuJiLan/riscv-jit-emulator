//
// Created by liujilan on 2026/6/19.
// jit/translator.c —— RV → IR 翻译器实装 (T1 c+ 范围: ADD/ADDI 真翻译 + 其他
// 截断 emit DISPATCH_EXIT 走 interpret 兜底).
//
// 实装顶段 doc 见 translator.h.
//
// 体例对齐:
//   - 取指 memcpy 4 字节 (跟 interpreter.c:226 同; 防 strict-aliasing)
//   - 跨页 check 推进后 cur_pc 进新 page 退 (跟 interpreter.c:896 同体例;
//     page_mask = ~0xFFFu)
//   - 软边界 BLOCK_INST_LIMIT - 1 (留 1 slot 给 DISPATCH_EXIT 收尾)
//   - PC_STEP 用 d.pc_step 而非 hardcoded 4 (T6 加 RVC 时自动适应; T1 ADD/ADDI
//     都是 PC_STEP_RV = 4)
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
     *   - 非 ADD/ADDI 截断 (含 OP_UNSUPPORTED / boundary 指令 / 其他算术 /
     *     load/store/csr 等)
     *   - 跨页 check (推进后 cur_pc 进新 page → 退)
     */
    while (i < BLOCK_INST_LIMIT - 1u) {
        u32_t inst;
        memcpy(&inst, hva_base + (cur_pc - pa), 4);

        decoded_inst_t d = decode(inst);

        if (d.kind != OP_ADD && d.kind != OP_ADDI) {
            break;
        }

        ir_buf[i].kind = (d.kind == OP_ADD) ? IR_OP_ADD : IR_OP_ADDI;
        ir_buf[i].rd   = (uint8_t)d.rd;
        ir_buf[i].rs1  = (uint8_t)d.rs1;
        ir_buf[i].rs2  = (uint8_t)d.rs2;
        ir_buf[i].imm  = d.imm;
        i++;

        cur_pc += d.pc_step;
        if ((cur_pc & TRANSLATOR_PAGE_MASK) != entry_page) {
            break;
        }
    }

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
