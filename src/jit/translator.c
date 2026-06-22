//
// jit/translator.c —— RV → IR 翻译器实装.
//
// 实装顶段 doc 见 translator.h. 当前真翻译 70 op (RV32 IMC + M; 详 ir.h 顶段
// 清单), 截断 op (OP_UNSUPPORTED) 走 interpret 兜底.
//
// 体例对齐:
//   - 取指 memcpy 4 字节 (跟 interpreter.c:226 同; 防 strict-aliasing)
//   - 跨页 check 推进后 cur_pc 进新 page 退 (跟 interpreter.c:896 同体例;
//     page_mask = ~0xFFFu)
//   - 软边界 i < BLOCK_INST_LIMIT (跟 interpreter.c:189 字面对偶, 真翻译
//     BLOCK_INST_LIMIT 条 RV inst; 末段 DISPATCH_EXIT 哨兵占 ir_buf 第
//     BLOCK_INST_LIMIT 槽, 故 caller stack 分配 ir_buf 大小 BLOCK_INST_LIMIT
//     + 1; b_04_session_004 对齐)
//   - PC_STEP 用 d.pc_step (RVC 时自动适应; PC_STEP_RV = 4, PC_STEP_RVC = 2)
//
// AUIPC: cur_pc + d.imm 合并写进 ir.imm (选 a 常量折叠, 跟 QEMU 默认 + rv8 一致;
//   baked_pc 不进 ir_inst_t 字段).
// PA != VA trail: 当前 cur_pc = pa, fixture 走 BARE 或者 SV32 直接映射时 pa = VA
// 等价; OS guest 真撞 pa != VA 时 (mapping 跨 page 不同 VA), AUIPC baked_pc 来源
// (用 pa 还是 cpu->pc VA) 单独议. 当前测试范围内未撞.
//
// 硬边界 op: CSR 6 + FENCE_I + BRANCH 6 + JAL + JALR + SYSTEM 6 = 21 op
// (plan §1.23.1 表 4 + 表 7; is_block_boundary_inst 返 1), translator 在
// case 装填 IR 后通过 decode.h is_block_boundary_inst(&d) 判定立刻 break
// 出翻译循环 — 跟 interpreter.c:792 fetch loop 末段字面对偶, 共享同一份
// 判定函数; 加新 RV op 时 decode.h is_block_boundary_inst 加 case
// (-Wswitch-enum 强制), translator 单点 pick up 不重复维护.
//
// TODO (JIT 独有优化路径, 明确推迟): 解释器视角"硬边界 = 立即切块"是必然的
// (解释器单条 inst execute 后必须重派发); JIT 视角则不一定 — IR 阶段已经能
// 看到一整条 RV inst 流, 有多种 IR-level 优化可在切块前融合:
//   1. BRANCH compile-time 静态目标 (target_pc baked) + JAL: 块内连续 BRANCH/JAL
//      链可被融合成单块, taken/fall-through 双出口都在同块 emit, 减少 dispatcher
//      round-trip. 当前 emit_ir_branch 一块两份 epilogue 已是双出口形态, 但仍
//      单块 (BRANCH 后即切); 真融合需要 translator 在 BRANCH/JAL target 也在
//      当前 page 时续译目标块.
//   2. CSR 一刀切硬边界 (plan §1.6 "过度刷新允许") 留了优化余地: 实际 CSRRS rd,
//      csr, x0 = 纯读不写, csrr mhartid 之类 RO identity CSR 完全无副作用 — 可
//      细分 csr_addr (mtvec/satp/sstatus/medeleg 等真有副作用的硬边界, 其他 read
//      不切块). 收益主要在 csr_heavy 类 fixture (当前 0.81x 退化, 见
//      REVIEW_REPORT §4.1).
//   3. IR 阶段常量传播 / 死代码消除: LUI + ADDI 序列折叠 baked 32-bit imm,
//      AUIPC + JALR pair 折叠 absolute target; 当前 AUIPC 已 baked cur_pc + imm
//      (translator.c 选 a 常量折叠), 但 LUI + ADDI link-time relocation pair
//      没融合.
// 跟 plan §2 deferred (block chaining + tiered JIT + Layer 3 跨块寄存器分配)
// 同性质: 都需要 JIT 视角的全局优化框架, 当前 v1 简化形态 ("硬边界 = 立即切块",
// 跟解释器对偶) 是基线, 未来 hot-trace profiling 数据到位后单独评估上不上.
// 不在 b_04 milestone 范围.
//
// CSR 字段约定 (ir.h 顶段 + ir_inst_t 段对偶 decode.h 字段约定):
//   ir_imm = d.imm = csr_addr 12-bit; RWI/RSI/RCI 变体 d.rs1 字段直接是 5-bit
//   zimm 数值, ir.rs1 装它即可 (跟 decoded_inst_t 同体例).
//
// 控制流 + SYSTEM 末段 DISPATCH_EXIT 装填的三个局部变量:
//   - ir_target_pc: BRANCH 装 taken_pc (= pre-incr cur_pc + d.imm); JAL/JALR 装
//                    rd 写入值 (= pre-incr cur_pc + d.pc_step); 其他 op 默认 0.
//   - hard_exit_pc: JAL 装 jump target (= pre-incr cur_pc + d.imm) 给末段
//                    DISPATCH_EXIT.target_pc; 其他 op 默认 0 走 cur_pc 路径.
//   - use_runtime_exit: JALR 设 1, 末段 IR 用 IR_OP_DISPATCH_EXIT_RUNTIME 哨兵
//                    (backend 从 r9d 读 runtime target); 其他 op 默认 0 走标准
//                    IR_OP_DISPATCH_EXIT 路径.
// BRANCH 末段 DISPATCH_EXIT.target_pc = cur_pc = fall_through_pc (taken_pc 已
// 装在 BRANCH IR 自身); MRET/SRET/WFI 末段 target_pc 装 cur_pc 但 backend 走
// COUNT_ONLY 出口不读此值 (helper 已自写 hart->regs[0]).
//
// 截断 op (仍 fallthrough goto out_translate, 走 interpret 兜底):
//   OP_UNSUPPORTED 1 op (兜底走 trap_raise cause=2 illegal).
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
    (void)regime;   /* translator 不消费 regime, 透传给 backend (LOAD/STORE fast path 用) */

    /* 块入口 hva = gpa_to_hva_offset + pa (ram.h "省去每次访问的减法"). 块前缀
     * 内每条指令的 hva = hva_base + (cur_pc - pa). */
    uint8_t *hva_base = gpa_to_hva_offset + pa;
    uxlen_t cur_pc = pa;
    size_t i = 0;

    const uxlen_t entry_page = pa & TRANSLATOR_PAGE_MASK;

    /* 末段 DISPATCH_EXIT 装填用 (loop 外 — 需要循环 break 后可见):
     *   hard_exit_pc       - JAL 装 jump target; 其他 op 不动
     *   have_hard_exit_pc  - 1 仅当 JAL 设了 hard_exit_pc (防 JAL imm=-cur_pc
     *                        撞 hard_exit_pc=0 sentinel 的 edge case)
     *   use_runtime_exit   - JALR 设 1; 末段走 IR_OP_DISPATCH_EXIT_RUNTIME */
    uxlen_t hard_exit_pc      = 0;
    int     have_hard_exit_pc = 0;
    int     use_runtime_exit  = 0;

    /* 翻译循环 (块前缀):
     *   - 软边界 i < BLOCK_INST_LIMIT (跟 interpreter.c:189 字面对偶, 真翻译
     *     BLOCK_INST_LIMIT 条 RV inst; caller stack 分配 ir_buf 大小
     *     BLOCK_INST_LIMIT + 1, 末槽留 DISPATCH_EXIT 哨兵)
     *   - 截断条件 (走 interpret 兜底): OP_UNSUPPORTED
     *   - 硬边界 op (21 op; 调 is_block_boundary_inst() 共享 decode.h) emit
     *     完 IR 后 break 切块
     *   - 跨页 check (推进后 cur_pc 进新 page → 退)
     */
    while (i < BLOCK_INST_LIMIT) {
        u32_t inst;
        memcpy(&inst, hva_base + (cur_pc - pa), 4);

        decoded_inst_t d = decode(inst);

        /* ir_kind 默认 IR_OP_UNSUPPORTED 防 release -O2 maybe-uninitialized
         * 假警 (debug -O0 不撞; switch 全覆盖时实际不会读默认值, 但 GCC -O2
         * 静态分析推不出来). */
        ir_op_kind_t ir_kind = IR_OP_UNSUPPORTED;
        int32_t      ir_imm  = d.imm;           /* 默认沿用 d.imm; CSR 时 d.imm
                                                 * 就是 csr_addr 12-bit
                                                 * (decode.h:158 line), 复用 imm
                                                 * 字段无需特殊处理 */
        /* ir_target_pc — BRANCH IR.target_pc 装 taken_pc; JAL/JALR IR.target_pc
         * 装 rd 写入值 (= pre-incr cur_pc + pc_step). 默认 0 (其他 op 不用此字段). */
        uxlen_t      ir_target_pc = 0;
        /* 硬边界 (CSR 6 + FENCE_I + BRANCH 6 + JAL + JALR + SYSTEM 6 = 21 op):
         * emit 完该 op IR 后, 末段 break 出循环 → DISPATCH_EXIT/_RUNTIME 收尾
         * 切块. 判定共享 decode.h is_block_boundary_inst() 跟 interpreter 对偶
         * (b_04_session_004 重构), 不在 case 内重复手填标志 — 加新 RV op 时
         * decode.h is_block_boundary_inst() 加 case (-Wswitch-enum 强制),
         * translator 自动 pick up. plan §1.23.1 表 4 + 表 7. */

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

            /* ---- I-type LOAD (5; rs1+imm = gva) ---- */
            case OP_LB:     ir_kind = IR_OP_LB;    break;
            case OP_LH:     ir_kind = IR_OP_LH;    break;
            case OP_LW:     ir_kind = IR_OP_LW;    break;
            case OP_LBU:    ir_kind = IR_OP_LBU;   break;
            case OP_LHU:    ir_kind = IR_OP_LHU;   break;

            /* ---- S-type STORE (3; rs1+imm = gva, rs2 = value) ---- */
            case OP_SB:     ir_kind = IR_OP_SB;    break;
            case OP_SH:     ir_kind = IR_OP_SH;    break;
            case OP_SW:     ir_kind = IR_OP_SW;    break;

            /* ---- I-type SYSTEM CSR (6; 硬边界 — emit 后切块) ----
             * ir_imm = d.imm (= csr_addr 12-bit, decode.h:158 line 字段约定);
             * ir.rs1 = d.rs1 直传 (RW/RS/RC 是寄存器号, RWI/RSI/RCI 是 5-bit
             * zimm 数值, 字段位置共用 — decode.h:159-162 line + ir.h 字段约定
             * I 变体段). */
            case OP_CSRRW:  ir_kind = IR_OP_CSRRW;  break;
            case OP_CSRRS:  ir_kind = IR_OP_CSRRS;  break;
            case OP_CSRRC:  ir_kind = IR_OP_CSRRC;  break;
            case OP_CSRRWI: ir_kind = IR_OP_CSRRWI; break;
            case OP_CSRRSI: ir_kind = IR_OP_CSRRSI; break;
            case OP_CSRRCI: ir_kind = IR_OP_CSRRCI; break;

            /* ---- R-type A 扩展 Zaamo (9; rs1=gva, rs2=value) ---- */
            case OP_AMO_ADD_W:  ir_kind = IR_OP_AMO_ADD_W;  break;
            case OP_AMO_SWAP_W: ir_kind = IR_OP_AMO_SWAP_W; break;
            case OP_AMO_XOR_W:  ir_kind = IR_OP_AMO_XOR_W;  break;
            case OP_AMO_OR_W:   ir_kind = IR_OP_AMO_OR_W;   break;
            case OP_AMO_AND_W:  ir_kind = IR_OP_AMO_AND_W;  break;
            case OP_AMO_MIN_W:  ir_kind = IR_OP_AMO_MIN_W;  break;
            case OP_AMO_MAX_W:  ir_kind = IR_OP_AMO_MAX_W;  break;
            case OP_AMO_MINU_W: ir_kind = IR_OP_AMO_MINU_W; break;
            case OP_AMO_MAXU_W: ir_kind = IR_OP_AMO_MAXU_W; break;

            /* ---- R-type A 扩展 Zalrsc (2) ---- */
            case OP_LR_W:   ir_kind = IR_OP_LR_W;  break;
            case OP_SC_W:   ir_kind = IR_OP_SC_W;  break;

            /* ---- MISC-MEM (2; FENCE_I 硬边界, FENCE 不是) ---- */
            case OP_FENCE:    ir_kind = IR_OP_FENCE;                          break;
            case OP_FENCE_I:  ir_kind = IR_OP_FENCE_I; break;

            /* ---- B-type BRANCH (6; 硬边界; 双出口) ----
             * BRANCH IR.target_pc 装 taken_pc (= 本指令 PC + d.imm); 末段
             * DISPATCH_EXIT.target_pc = cur_pc (= 本指令 PC + pc_step =
             * fall_through_pc). cur_pc 此时尚未推进, 用即可. */
            case OP_BEQ:
                ir_kind = IR_OP_BEQ;
                ir_target_pc = cur_pc + (uint32_t)d.imm;
                break;
            case OP_BNE:
                ir_kind = IR_OP_BNE;
                ir_target_pc = cur_pc + (uint32_t)d.imm;
                break;
            case OP_BLT:
                ir_kind = IR_OP_BLT;
                ir_target_pc = cur_pc + (uint32_t)d.imm;
                break;
            case OP_BGE:
                ir_kind = IR_OP_BGE;
                ir_target_pc = cur_pc + (uint32_t)d.imm;
                break;
            case OP_BLTU:
                ir_kind = IR_OP_BLTU;
                ir_target_pc = cur_pc + (uint32_t)d.imm;
                break;
            case OP_BGEU:
                ir_kind = IR_OP_BGEU;
                ir_target_pc = cur_pc + (uint32_t)d.imm;
                break;

            /* ---- J-type JAL (1; 硬边界; compile-time target) ----
             * JAL IR.target_pc 装 rd 写入值 (= 本指令 PC + pc_step); 末段
             * DISPATCH_EXIT.target_pc 由 hard_exit_pc 装 jump target
             * (= 本指令 PC + d.imm). */
            case OP_JAL:
                ir_kind = IR_OP_JAL;
                ir_target_pc      = cur_pc + d.pc_step;
                hard_exit_pc      = cur_pc + (uint32_t)d.imm;
                have_hard_exit_pc = 1;
                break;

            /* ---- I-type JALR (1; 硬边界; runtime target) ----
             * JALR IR.target_pc 装 rd 写入值 (= 本指令 PC + pc_step); ir.imm =
             * imm12 backend 端算 target = (rs1 + imm12) & ~1u 存 r9d; 末段 IR
             * 走 DISPATCH_EXIT_RUNTIME 从 r9d 读 target. */
            case OP_JALR:
                ir_kind = IR_OP_JALR;
                ir_target_pc = cur_pc + d.pc_step;
                use_runtime_exit = 1;
                break;

            /* ---- I-type SYSTEM (6; 全硬边界) ----
             * ECALL/EBREAK: backend emit `call trap_raise_exception` _Noreturn
             *   longjmp; 末段 DISPATCH_EXIT 是 dead code (backend 选择不 emit).
             * MRET/SRET: backend emit `call mret_helper/sret_helper`; helper 内
             *   含 PRIV_CHECK + 写 hart->regs[0] = xepc; 末段走 COUNT_ONLY
             *   不写 cpu->pc.
             * SFENCE_VMA: backend emit PRIV_CHECK inline + `call sfence_vma_helper`;
             *   末段走标准 DISPATCH_EXIT (helper 不写 cpu->pc).
             * WFI: backend emit TW 检查 inline + `call wfi_wait` (cond_wait 阻塞
             *   到 SRS / 中断 pending) + `call lrsc_clear_self` + pc += 4 自推
             *   进; 末段走 COUNT_ONLY 不写 cpu->pc.
             * raw_inst 由 ir_buf 末段统一装 (line 184), helper 内 PRIV_CHECK/TW
             * 失败 trap 时用作 mtval. */
            case OP_ECALL:       ir_kind = IR_OP_ECALL;       break;
            case OP_EBREAK:      ir_kind = IR_OP_EBREAK;      break;
            case OP_MRET:        ir_kind = IR_OP_MRET;        break;
            case OP_SRET:        ir_kind = IR_OP_SRET;        break;
            case OP_SFENCE_VMA:  ir_kind = IR_OP_SFENCE_VMA;  break;
            case OP_WFI:         ir_kind = IR_OP_WFI;         break;

            /* ---- RV32M MUL 家族 4 op ----
             * backend emit_ir_muldiv inline (3 路 imul/mul 路径 + MULHSU 64-bit
             * imul); 非边界 (is_block_boundary_inst 返 0), 翻译完续翻译同块. */
            case OP_MUL:    ir_kind = IR_OP_MUL;    break;
            case OP_MULH:   ir_kind = IR_OP_MULH;   break;
            case OP_MULHSU: ir_kind = IR_OP_MULHSU; break;
            case OP_MULHU:  ir_kind = IR_OP_MULHU;  break;

            /* ---- RV32M DIV/REM 家族 4 op ----
             * backend emit_ir_muldiv inline branch 兜 by-0 + INT_MIN/-1 overflow
             * 避 x86 #DE trap; 镜像 interpreter.c:357-397. 非边界. */
            case OP_DIV:    ir_kind = IR_OP_DIV;    break;
            case OP_DIVU:   ir_kind = IR_OP_DIVU;   break;
            case OP_REM:    ir_kind = IR_OP_REM;    break;
            case OP_REMU:   ir_kind = IR_OP_REMU;   break;

            /* 兜底 (interpret 走 trap_raise cause=2 illegal). 显式列 (而非走
             * default) 是因为 -Wswitch-enum -Werror 强制 op_kind_t switch 同
             * 步; CLAUDE.md 顶段 "load-bearing mechanism" 段. */
            case OP_UNSUPPORTED:
                goto out_translate;
        }

        ir_buf[i].kind      = ir_kind;
        ir_buf[i].target_pc = ir_target_pc;  /* BRANCH taken_pc / JAL/JALR rd 值;
                                              * 其他 op 默认 0 (backend 不读) */
        ir_buf[i].rd        = (uint8_t)d.rd;
        ir_buf[i].rs1       = (uint8_t)d.rs1;
        ir_buf[i].rs2       = (uint8_t)d.rs2;
        ir_buf[i].imm       = ir_imm;
        ir_buf[i].raw_inst  = inst;       /* CSR 6 + SYSTEM (MRET/SRET/SFENCE_VMA/
                                           * WFI) op illegal trap mtval; 其他 op
                                           * 不读 — 一律填便宜不分流 */
        i++;

        cur_pc += d.pc_step;

        /* 硬边界 21 op (CSR 6 + FENCE_I + BRANCH 6 + JAL + JALR + SYSTEM 6):
         * emit 完后强制 break 出循环, 走 DISPATCH_EXIT 收尾 — dispatcher 重派发
         * 以重新算 (regime, current_tlb) 跟 trap 状态. 跟解释器 fetch loop 末段
         * `if (is_block_boundary_inst(&d)) goto out;` 字面对偶 (interpreter.c:792),
         * 共享 decode.h is_block_boundary_inst() — 加新 RV op 时 decode.h
         * -Wswitch-enum 强制归类, translator 单点自动跟随. plan §1.23.1 +
         * ir.h 顶段 helper call 段. */
        if (is_block_boundary_inst(&d)) {
            break;
        }

        if ((cur_pc & TRANSLATOR_PAGE_MASK) != entry_page) {
            break;
        }
    }
out_translate:;

    /* 块出口 IR 收尾 (按 use_runtime_exit / have_hard_exit_pc 分流三形态):
     *
     *   1. JALR 块 (use_runtime_exit == 1):
     *        kind = IR_OP_DISPATCH_EXIT_RUNTIME; backend 从 r9d 读 runtime target
     *        写 cpu->pc + 写 count_out + epilogue. target_pc 字段不读.
     *
     *   2. JAL 块 (have_hard_exit_pc == 1):
     *        kind = IR_OP_DISPATCH_EXIT; target_pc = hard_exit_pc = JAL 的 jump
     *        target (= JAL pc + d.imm); backend 写 cpu->pc=target_pc.
     *
     *   3. 其他 (BRANCH / CSR / FENCE / SYSTEM / 截断 / 跨页 / 软边界):
     *        kind = IR_OP_DISPATCH_EXIT; target_pc = cur_pc (= 截断指令 PC /
     *        软边界出块 PC / 硬边界 op 后下一 PC). BRANCH 的 fall_through_pc /
     *        SYSTEM 的下一 PC 都走此路径. MRET/SRET/WFI 块 backend 走 COUNT_ONLY
     *        出口不读 target_pc.
     *
     * 块前缀空 (i == 0): target_pc = pa (跟块入口同); backend.compile_block 检
     * 测 n_insts == 1 时返 NOT_IMPLEMENTED, jit_entry.cc set_blacklist 让
     * interpret 兜底真执行那条 RV.
     *
     * rd/rs1/rs2/imm 清 0 是兜底 (backend default case 只读 target_pc, 这些字段
     * 不读; 清 0 防 stale 数据). */
    if (use_runtime_exit) {
        ir_buf[i].kind      = IR_OP_DISPATCH_EXIT_RUNTIME;
        ir_buf[i].target_pc = 0;        /* RUNTIME 不读; backend 从 r9d 读 */
    } else {
        ir_buf[i].kind      = IR_OP_DISPATCH_EXIT;
        ir_buf[i].target_pc = have_hard_exit_pc ? hard_exit_pc : cur_pc;
    }
    ir_buf[i].rd        = 0;
    ir_buf[i].rs1       = 0;
    ir_buf[i].rs2       = 0;
    ir_buf[i].imm       = 0;
    ir_buf[i].raw_inst  = 0;
    i++;

    *n_insts = i;
}
