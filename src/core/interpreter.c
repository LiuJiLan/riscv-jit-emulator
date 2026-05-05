//
// Created by liujilan on 2026/4/28.
// a01_3 interpreter 模块实现 (算术 / 逻辑 / 立即数子集)。
//
// 顶部模块文档见 interpreter.h。fast/slow path 协议见 dummy.txt §1; x0 编码见 §2;
// TLB 派发见 §4。
//

#include "interpreter.h"

#include "config.h"     // BLOCK_INST_LIMIT, IALIGN_MASK
#include "cpu.h"
#include "csr.h"        // csr_op + csr_op_t (a_01_5_a 6 csr case)
#include "decode.h"
#include "isa/lsu.h"    // load_helper (static inline) + store_helper (extern), a_01_6 5 load + 3 store
#include "riscv.h"      // PRIV_M (a_01_5_c MRET 读 hart->trap.xepc[PRIV_M])
#include "tlb.h"
#include "trap.h"       // trap_raise_exception 真 helper (a_01_5_b; 替换 a_01_4 的 file-static 占位)

#include <stdint.h>
#include <string.h>     // memcpy: 4 字节取指, 防 strict-aliasing

void interpret_one_block(cpu_t *hart, tlb_t *current_tlb,
                         uint8_t *hva_pc, uint32_t *count_out) {
    // a_01_6: current_tlb 透传给 load_helper / store_helper (BARE: NULL; SV32: 非 NULL)。
    // 不再 (void)current_tlb 抑制 unused — 真用了。

    // dummy.txt §2 局部垃圾桶变量: 写 x0 的 dead store 落点。
    // 编译器 DCE 会把这个 store 干掉, 等于 NO-OP; 保留是为统一 "所有写都通过同一个宏" 风格,
    // 让未来 IR / 后端不感知 x0 特殊性 (a01_3 体现不到, 但统一)。
    uint32_t x0_garbage = 0;
    (void)x0_garbage;            // 防 -Wunused-variable; 真有 WRITE_REG(0,...) 时编译器会用

    // 局部宏: hart / x0_garbage / pc 隐式捕获 (interpret_one_block 函数内, 上下文清楚)。
    // 函数末尾统一 #undef, 不污染翻译单元其它部分。
    // 注: 调用方必须传简单 lvalue (decode 后 d.rd / d.rs1 / d.rs2 都是 struct 字段, 无副作用)。
    #define READ_REG(r)       ((r) == 0u ? 0u : hart->regs[r])
    #define WRITE_REG(r, val) (*((r) == 0u ? &x0_garbage : &hart->regs[r]) = (uint32_t)(val))

    // WRITE_PC_OR_TRAP: control flow case 内统一写 pc 入口, 含对齐检查 + trap 占位。
    //   - target 对齐 → hart->regs[0] = target, case 正常 break, fetch loop 末尾 +=
    //     PC_STEP_NONE = 0 (case 已写 pc, +=0 NOP), is_block_boundary_inst 检查 → goto out
    //   - target 不对齐 → trap_raise_exception(hart, 0, target), 然后 goto out
    //                    (goto out 路径跳过末尾 pc_step / count++ / boundary 检查, 故
    //                    hart->regs[0] 保持为触发指令的 PC, count 不含本指令, 与 RV
    //                    precise trap 语义对齐)
    //   隐式捕获: hart (写 pc + 调 helper)。pc 不在宏内用 (helper 自己读 hart->regs[0])。
    #define WRITE_PC_OR_TRAP(target) do {                                  \
        uint32_t _t = (target);                                            \
        if ((_t & IALIGN_MASK) != 0u) {                                    \
            trap_raise_exception(hart, CAUSE_INST_ADDR_MISALIGNED, /*tval*/_t); \
            goto out;                                                      \
        }                                                                  \
        hart->regs[0] = _t;                                                \
    } while (0)

    // BRANCH_IF: 6 个 branch case 的统一封装 (taken → WRITE_PC_OR_TRAP; not-taken → +4)。
    //   not-taken 走 pc + 4 (32-bit 分支固定 4 字节; 未来 RVC 真支持 C.BEQZ/C.BNEZ 时, 那
    //   两个新 op_kind 单独处理 +2; 不混进本宏)。pc + 4 一定 IALIGN=16 对齐 (pc 自身已对齐),
    //   不需 check, 直接写。
    //   隐式捕获: hart, pc, d.imm。
    #define BRANCH_IF(cond) do {                                           \
        if (cond) WRITE_PC_OR_TRAP(pc + (uint32_t)d.imm);                  \
        else      hart->regs[0] = pc + 4u;                                 \
    } while (0)

    uint32_t count = 0;

    while (count < BLOCK_INST_LIMIT) {
        // 取指: hva_pc 指向当前指令字节起点。a01_3 假设块内不跨 4K page, 不需要每次重做
        // mmu_translate_pc。memcpy 防 strict-aliasing / unaligned 风险 (RV32 指令必 4 字节
        // 对齐, 实际 hva_pc 也对齐, 但 memcpy 表达更通用; 编译器会优化为单 mov)。
        uint32_t inst;
        memcpy(&inst, hva_pc, 4);

        const decoded_inst_t d = decode(inst);

        // 当前 PC = regs[0] (cpu.h: regs[0] 物理位置存 pc, x0 走 garbage 路径不碰这里)
        const uint32_t pc = hart->regs[0];

        // SLLI/SRLI/SRAI 的 shamt: decode 已把 5 位 shamt 放进 imm 低 5 位; R-type SLL/SRL/SRA
        // 用 rs2 寄存器值的低 5 位 (RV 规范要求; mini-rv32ima 同)。
        const uint32_t shamt_i = (uint32_t)d.imm & 0x1Fu;

        switch (d.kind) {
            // ---- U-type ----
            case OP_LUI:
                WRITE_REG(d.rd, (uint32_t)d.imm);
                break;
            case OP_AUIPC:
                WRITE_REG(d.rd, pc + (uint32_t)d.imm);
                break;

            // ---- I-type (OP-IMM) ----
            case OP_ADDI:
                WRITE_REG(d.rd, READ_REG(d.rs1) + (uint32_t)d.imm);
                break;
            case OP_SLTI:
                WRITE_REG(d.rd, ((int32_t)READ_REG(d.rs1) < d.imm) ? 1u : 0u);
                break;
            case OP_SLTIU:
                // SLTIU: imm 先符号扩展到 32 位, 再按 unsigned 比较 (RV 规范)
                WRITE_REG(d.rd, (READ_REG(d.rs1) < (uint32_t)d.imm) ? 1u : 0u);
                break;
            case OP_XORI:
                WRITE_REG(d.rd, READ_REG(d.rs1) ^ (uint32_t)d.imm);
                break;
            case OP_ORI:
                WRITE_REG(d.rd, READ_REG(d.rs1) | (uint32_t)d.imm);
                break;
            case OP_ANDI:
                WRITE_REG(d.rd, READ_REG(d.rs1) & (uint32_t)d.imm);
                break;
            case OP_SLLI:
                WRITE_REG(d.rd, READ_REG(d.rs1) << shamt_i);
                break;
            case OP_SRLI:
                WRITE_REG(d.rd, READ_REG(d.rs1) >> shamt_i);
                break;
            case OP_SRAI:
                // 算术右移: 必须 cast 到 int32_t, C 对 signed >> 才是算术右移
                WRITE_REG(d.rd, (uint32_t)((int32_t)READ_REG(d.rs1) >> shamt_i));
                break;

            // ---- R-type (OP) ----
            case OP_ADD:
                WRITE_REG(d.rd, READ_REG(d.rs1) + READ_REG(d.rs2));
                break;
            case OP_SUB:
                WRITE_REG(d.rd, READ_REG(d.rs1) - READ_REG(d.rs2));
                break;
            case OP_SLL:
                WRITE_REG(d.rd, READ_REG(d.rs1) << (READ_REG(d.rs2) & 0x1Fu));
                break;
            case OP_SLT:
                WRITE_REG(d.rd,
                          ((int32_t)READ_REG(d.rs1) < (int32_t)READ_REG(d.rs2)) ? 1u : 0u);
                break;
            case OP_SLTU:
                WRITE_REG(d.rd, (READ_REG(d.rs1) < READ_REG(d.rs2)) ? 1u : 0u);
                break;
            case OP_XOR:
                WRITE_REG(d.rd, READ_REG(d.rs1) ^ READ_REG(d.rs2));
                break;
            case OP_SRL:
                WRITE_REG(d.rd, READ_REG(d.rs1) >> (READ_REG(d.rs2) & 0x1Fu));
                break;
            case OP_SRA:
                WRITE_REG(d.rd,
                          (uint32_t)((int32_t)READ_REG(d.rs1) >> (READ_REG(d.rs2) & 0x1Fu)));
                break;
            case OP_OR:
                WRITE_REG(d.rd, READ_REG(d.rs1) | READ_REG(d.rs2));
                break;
            case OP_AND:
                WRITE_REG(d.rd, READ_REG(d.rs1) & READ_REG(d.rs2));
                break;

            // ---- B-type BRANCH (a_01_4) ----
            // 比较 rs1 / rs2 (按 funct3 决定有/无符号 + 比较方向); taken 走 pc + imm
            // (经 WRITE_PC_OR_TRAP 含对齐检查), not-taken 走 pc + 4 (固定 32-bit 分支)。
            case OP_BEQ:  BRANCH_IF(READ_REG(d.rs1)            == READ_REG(d.rs2));            break;
            case OP_BNE:  BRANCH_IF(READ_REG(d.rs1)            != READ_REG(d.rs2));            break;
            case OP_BLT:  BRANCH_IF((int32_t)READ_REG(d.rs1)   <  (int32_t)READ_REG(d.rs2));   break;
            case OP_BGE:  BRANCH_IF((int32_t)READ_REG(d.rs1)   >= (int32_t)READ_REG(d.rs2));   break;
            case OP_BLTU: BRANCH_IF(READ_REG(d.rs1)            <  READ_REG(d.rs2));            break;
            case OP_BGEU: BRANCH_IF(READ_REG(d.rs1)            >= READ_REG(d.rs2));            break;

            // ---- J-type JAL (a_01_4) ----
            // rd = pc + 4 (返回地址, 调用约定: rd=x1 是 ra, rd=x0 是无返回纯跳转); pc =
            // pc + imm (经 WRITE_PC_OR_TRAP 含对齐检查)。
            // 写 rd 在写 pc 之前: 即使后者 trap 也要先把 rd 写好? 不, 反过来想 — 如果 trap
            // 触发, 整条指令视为未执行 (precise trap), rd 也不该被写。所以 WRITE_REG 应该
            // 在对齐检查通过之后。但 WRITE_PC_OR_TRAP 的对齐检查在写 hart->regs[0] 之前,
            // 而 WRITE_REG 写的是 rd != 0; pc-misalign 在 IALIGN=16 + jal/branch imm[0]=0
            // 编码下永远不命中, 顺序问题在本项目不构成实际正确性 bug; 但为了"语义严格",
            // 之后 (a_01_5 真接 trap) 可以重构成 "先算 target → 检查对齐 → 写 rd → 写 pc"。
            // 当前先按"写 rd 再 WRITE_PC_OR_TRAP"形态走, 简洁 + 与现状语义一致。
            case OP_JAL:
                WRITE_REG(d.rd, pc + 4u);
                WRITE_PC_OR_TRAP(pc + (uint32_t)d.imm);
                break;

            // ---- I-type JALR (a_01_4) ----
            // 同 JAL, 区别: 目标 = (rs1 + imm) & ~1u (RV spec 强制 mask LSB; mask 后 bit[0]
            // 永远 0, 即 IALIGN=16 永远过)。
            case OP_JALR:
                WRITE_REG(d.rd, pc + 4u);
                WRITE_PC_OR_TRAP((READ_REG(d.rs1) + (uint32_t)d.imm) & ~1u);
                break;

            // ---- I-type SYSTEM CSR 6 变体 (a_01_5_a) ----
            // 6 个 op_kind 在 csr 侧映射到 3 个内核操作 (csr_op_t) + new_val 来源:
            //   RW/RS/RC:    new_val = READ_REG(d.rs1)             /* rs1 是寄存器号 */
            //   RWI/RSI/RCI: new_val = (uint32_t)d.rs1             /* rs1 字段当 5-bit zimm */
            // csr_op 返回 read_old, WRITE_REG(d.rd, old) 写 rd (rd=x0 走 dummy.txt §2 dead store
            // 路径, 自然丢弃)。case 末 break (不 goto out), fetch loop 末 count++ 后
            // is_block_boundary_inst (Step 2 已 return 1) 让 fetch loop 退出, dispatcher
            // 重派发 pc + 4 进下一块。
            //
            // d.imm 是 12-bit csr addr (decode 已无符号扩展放低 12 位, 高 20 位 0); 这里
            // (uint32_t)d.imm 强转去除 int32_t 符号扩展风险 (d.imm 实际值范围 [0, 0xFFF],
            // cast 等价于 d.imm & 0xFFF, 但语义清晰)。
            //
            // d.raw_inst 是 32-bit 原始指令编码 (decode 顶部已填), 给 csr_op 内 trap 路径
            // 填 mtval 用 (a_01_5_b 真接 trap.c 后激活)。
            case OP_CSRRW:
                WRITE_REG(d.rd, csr_op(hart, (uint32_t)d.imm, READ_REG(d.rs1),
                                       CSR_OP_RW, d.raw_inst));
                break;
            case OP_CSRRS:
                WRITE_REG(d.rd, csr_op(hart, (uint32_t)d.imm, READ_REG(d.rs1),
                                       CSR_OP_RS, d.raw_inst));
                break;
            case OP_CSRRC:
                WRITE_REG(d.rd, csr_op(hart, (uint32_t)d.imm, READ_REG(d.rs1),
                                       CSR_OP_RC, d.raw_inst));
                break;
            case OP_CSRRWI:
                WRITE_REG(d.rd, csr_op(hart, (uint32_t)d.imm, (uint32_t)d.rs1,
                                       CSR_OP_RW, d.raw_inst));
                break;
            case OP_CSRRSI:
                WRITE_REG(d.rd, csr_op(hart, (uint32_t)d.imm, (uint32_t)d.rs1,
                                       CSR_OP_RS, d.raw_inst));
                break;
            case OP_CSRRCI:
                WRITE_REG(d.rd, csr_op(hart, (uint32_t)d.imm, (uint32_t)d.rs1,
                                       CSR_OP_RC, d.raw_inst));
                break;

            // ---- I-type SYSTEM (ECALL / EBREAK / MRET) ---- a_01_5_c
            //
            // ECALL: 触发 environment-call trap, cause 按 priv 分流 (RV spec table 3.6):
            //          priv U → cause 8, priv S → cause 9, priv M → cause 11
            //          有意写成 (8 + hart->priv) 公式 — RV 编码巧合: PRIV_U=0, PRIV_S=1,
            //          PRIV_M=3 与 cause 8/9/11 差 8。a_01 hart->priv 永远 PRIV_M (= 3),
            //          所以实际值 = 11。Spike/QEMU 同做法。
            //          tval = 0 (RV spec §3.1.17: ECALL/EBREAK 的 mtval write 0)。
            //
            // EBREAK: cause 3 (breakpoint, RV spec); 不分 priv。tval 一般 = 0
            //          (实现 debug 子集时可填触发 PC, 项目当前 = 0)。
            //
            // MRET: 从 trap handler 回归; 不调 trap_raise_exception, 是 csr 路径 + trap_set_state
            //          的反操作 (a_01_7 真激活, 替换 a_01_5_c 简化版):
            //          - hart->priv = mstatus.MPP                 (从 MPP 恢复 caller priv)
            //          - mstatus.MIE  = mstatus.MPIE              (恢复 interrupt-enable)
            //          - mstatus.MPIE = 1                          (RV spec 要求)
            //          - mstatus.MPP  = PRIV_U                     (RV spec least-priv reset)
            //          - hart->regs[0] = hart->trap.xepc[PRIV_M]   (从 mepc 恢复 PC)
            //          - hart->trap.in_trap = 0                    (复位嵌套链)
            //
            //          case 末 break (不 goto out): fetch loop 末 += PC_STEP_NONE (=0, NOP),
            //          count++ 计入本指令 (precise: MRET 已成功执行), boundary 检查 (MRET 是
            //          boundary) → goto out 退出 fetch loop, dispatcher 重派发 from xepc。
            //
            //          权限要求: MRET 仅在 priv >= M 时合法 (M-mode CSR 入口); a_01_7 csr_op 入口判
            //          激活后, 由 csrw mepc / csrr mepc 这条 csr 入口判把关; 但 MRET 本身不是 csr
            //          指令, 是 system 指令, 没走 csr_op。当前 fixture 只在 M-mode 跑 MRET; 真做
            //          U/S-mode 时, MRET 在 U/S 触发 cause=2 (illegal instruction) 是 spec 要求,
            //          a_01_7 暂不接 (本次 fixture 不构造 U-mode mret, 留 a_01_8+)。
            case OP_ECALL:
                /* RV 编码巧合: PRIV_U=0/S=1/M=3 ↔ cause 8/9/11 (= CAUSE_ECALL_FROM_U + priv).
                 * Spike / QEMU 同写法; PRIV_H=2 在没 H 扩展下不会触发 (hart->priv ∉ {2}). */
                trap_raise_exception(hart, CAUSE_ECALL_FROM_U + hart->priv, /*tval*/0u);
                goto out;
            case OP_EBREAK:
                trap_raise_exception(hart, CAUSE_BREAKPOINT, /*tval*/0u);
                goto out;
            case OP_MRET: {
                uint32_t mstatus_lo = (uint32_t)(hart->trap._mstatus & 0xFFFFFFFFu);

                /* hart->priv = MPP (从 mstatus 恢复 caller priv) */
                hart->priv = (uint8_t)((mstatus_lo >> MSTATUS_MPP_SHIFT) & MSTATUS_MPP_MASK);

                /* mstatus.MIE = mstatus.MPIE */
                if (mstatus_lo & MSTATUS_MPIE) mstatus_lo |=  MSTATUS_MIE;
                else                           mstatus_lo &= ~MSTATUS_MIE;

                /* mstatus.MPIE = 1 (RV spec) */
                mstatus_lo |= MSTATUS_MPIE;

                /* mstatus.MPP = PRIV_U = 0 (RV spec least-priv reset) */
                mstatus_lo &= ~MSTATUS_MPP;
                /* PRIV_U=0 已是清零默认; 显式 `| (PRIV_U << MSTATUS_MPP_SHIFT)` 是 0, 省略 */

                hart->trap._mstatus = (hart->trap._mstatus & 0xFFFFFFFF00000000ULL)
                                    | (uint64_t)mstatus_lo;

                hart->trap.in_trap = 0;
                hart->regs[0]      = hart->trap.xepc[PRIV_M];
                break;
            }

            // ---- I-type LOAD (5 op, a_01_6) ----
            //
            // 不对称设计 (file_plan §8.interpreter D 区, dummy.txt §1 末段): load 走
            // static inline load_helper (isa/lsu.h, fast path); BARE 路径直接 host load,
            // SV32 路径 a_01_7+ 加 TLB lookup + walker_helper_load miss。
            //
            // 方案 A (helper 不知 signed): load_helper 返回低 size 字节有效 + 高位 0 的 uint32_t,
            // case 自做 sext (LB int8_t / LH int16_t cast → int32_t 再 cast 回 uint32_t) 或
            // zext (LBU/LHU 直接传, 高位已是 0)。LW size=4 直传整 32 位。
            //
            // ea 算成 uint32_t (RV32 wraparound 算术; gva = rs1 + imm 自然 wrap)。imm 是
            // int32_t (decode 已 sign-ext), 加到 uint32_t 上要先 cast (避免 -Wsign-conversion;
            // 跟 a_01_4 BRANCH_IF / WRITE_PC_OR_TRAP 路径同模式)。
            //
            // 错误路径 (load_helper 内部 trap_raise_exception _Noreturn longjmp):
            //   misalign (gva & (size-1)) != 0    → cause 4
            //   PA 不在 RAM (a_01_6 BARE)         → cause 5 + fprintf "MMIO not implemented"
            //   SV32 路径 (a_01_6 不可达)         → cause 13 + fprintf "SV32 not implemented"
            // 长跳走 dispatcher 落点; 这里 break 后的 fetch loop 末尾 += pc_step / count++ /
            // boundary 检查不会执行。
            case OP_LB: {
                uint32_t ea = READ_REG(d.rs1) + (uint32_t)d.imm;
                WRITE_REG(d.rd, (uint32_t)(int32_t)(int8_t) load_helper(hart, current_tlb, ea, 1u));
                break;
            }
            case OP_LH: {
                uint32_t ea = READ_REG(d.rs1) + (uint32_t)d.imm;
                WRITE_REG(d.rd, (uint32_t)(int32_t)(int16_t)load_helper(hart, current_tlb, ea, 2u));
                break;
            }
            case OP_LW: {
                uint32_t ea = READ_REG(d.rs1) + (uint32_t)d.imm;
                WRITE_REG(d.rd,                              load_helper(hart, current_tlb, ea, 4u));
                break;
            }
            case OP_LBU: {
                uint32_t ea = READ_REG(d.rs1) + (uint32_t)d.imm;
                WRITE_REG(d.rd,                              load_helper(hart, current_tlb, ea, 1u));
                break;
            }
            case OP_LHU: {
                uint32_t ea = READ_REG(d.rs1) + (uint32_t)d.imm;
                WRITE_REG(d.rd,                              load_helper(hart, current_tlb, ea, 2u));
                break;
            }

            // ---- S-type STORE (3 op, a_01_6) ----
            //
            // 不对称设计: store 走 extern store_helper (isa/lsu.c, slow path)。helper 内部
            // 处理 misalign / RAM 检查 / host store / reservation 清除占位 / 未来 SMC 检测;
            // 解释器 case 不感知。a_01_5_c 跟 a_01_5_b 比 case 数量增加, 但 store path 形态
            // 跟 csr_op 入口风格一致 (caller 简洁, helper 内做事)。
            //
            // SB/SH/SW 写多少字节由 size 决定; value = READ_REG(d.rs2) 整 32 位传给 helper,
            // helper 内部 memcpy size 字节 (SB 写低 8 位, SH 写低 16 位, SW 写全 32 位)。
            //
            // 错误路径 (store_helper 内部 trap_raise_exception _Noreturn longjmp):
            //   misalign → cause 6; 不在 RAM → cause 7 + fprintf; SV32 占位 → cause 15。
            case OP_SB: {
                uint32_t ea = READ_REG(d.rs1) + (uint32_t)d.imm;
                store_helper(hart, current_tlb, ea, READ_REG(d.rs2), 1u);
                break;
            }
            case OP_SH: {
                uint32_t ea = READ_REG(d.rs1) + (uint32_t)d.imm;
                store_helper(hart, current_tlb, ea, READ_REG(d.rs2), 2u);
                break;
            }
            case OP_SW: {
                uint32_t ea = READ_REG(d.rs1) + (uint32_t)d.imm;
                store_helper(hart, current_tlb, ea, READ_REG(d.rs2), 4u);
                break;
            }

            // ---- 兜底 ---- (a_01_5_b 接 trap_raise_exception 真路径)
            case OP_UNSUPPORTED:
                // RV cause 2 = illegal instruction; tval = raw_inst (RV spec §3.1.16)。
                // a_01_5_b: helper 内部调 trap_set_state (写 xcause/xtval/xepc, regs[0]=xtvec),
                //           普通 return; 这里 goto out 退出 fetch loop, dispatcher 通过
                //           while(in_trap < 3) 接管 (in_trap=3 时退出 dispatcher)。
                // a_01_5_c: helper 标 _Noreturn longjmp, goto out 变 unreachable 但保留无害。
                trap_raise_exception(hart, CAUSE_ILLEGAL_INSTRUCTION, /*tval*/d.raw_inst);
                goto out;
        }

        // PC 推进 (数据驱动, decode 一次决定):
        //   PC_STEP_RV   (4): 普通 32-bit 算术 / 逻辑 / 立即数 op, fetch loop +4
        //   PC_STEP_RVC  (2): 16-bit C 扩展, fetch loop +2
        //   PC_STEP_NONE (0): control flow op (branch/jal/jalr/...) case 自描述 pc,
        //                     fetch loop += 0 是 NOP (case 内必须 hart->regs[0] = ...,
        //                     漏 = 死循环 → hard limit 立刻 break, 易发现)
        // 设计意图见 decode.h PC_STEP_* 注释。
        hart->regs[0] += d.pc_step;
        hva_pc        += d.pc_step;
        count++;

        // 硬边界判定 (a_01_4 接入): branch/jal/jalr 类 op 已在上方 case 内写好新 pc, 此处
        // 退出 fetch loop, 让 dispatcher 重新 mmu_translate_pc 拿新入口的 hva 进下一块。
        // boundary 那条指令计入 count (precise: 已成功执行); trap 路径走 WRITE_PC_OR_TRAP
        // 内 goto out, 不到此处, count 不含 trap 那条。
        if (is_block_boundary_inst(&d)) goto out;
    }

out:
    *count_out = count;

    #undef READ_REG
    #undef WRITE_REG
    #undef WRITE_PC_OR_TRAP
    #undef BRANCH_IF
}
