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
#include "decode.h"
#include "tlb.h"

#include <inttypes.h>   // PRIx32 for trap_raise_exception fprintf
#include <stdint.h>
#include <stdio.h>      // fprintf for trap_raise_exception 占位
#include <string.h>     // memcpy: 4 字节取指, 防 strict-aliasing

// ----------------------------------------------------------------------------
// trap_raise_exception —— 同步异常入口 (a_01_4 临时形态, 解释器 + 未来 JIT 共用)
//
// 接口契约:
//   - mepc 隐式 = hart->regs[0] (= 触发指令的 PC)。caller 必须在调本 helper 前
//     不动 hart->regs[0] (control flow case 走 WRITE_PC_OR_TRAP 宏满足: 对齐
//     检查在写 pc 之前; 其它 case 也满足: case 头部触发 trap, 没动过 pc)。
//   - tval 是 cause-specific:
//       cause 0  (instruction-address-misaligned): tval = target_pc
//       cause 2  (illegal instruction):            tval = raw_inst
//       cause 4/6 (load/store-address-misaligned): tval = effective_address
//       cause 12/13/15 (page fault):                tval = bad_va
//   - xtval / xcause / xepc 中 x 是哪个 priv (M / S) 由 helper 内部根据 mideleg /
//     medeleg 决定, caller 不需要知道 (这个分流在 a_01_5 真接 trap.c 时落地)。
//
// 演进路径:
//   a_01_4: fprintf 占位 (本 helper); 函数返回, caller 必须 goto out 退出 fetch loop。
//   a_01_5: trap.c 接入。helper 内部:
//     (1) 走 mideleg/medeleg 决定 trap deliver 到哪个 priv;
//     (2) 设 xcause / xtval / xepc, 切 priv mode, 跳 xtvec;
//     (3) siglongjmp(*hart->jmp_buf_ptr) 跳回 dispatcher 主循环。
//     helper 不返回, 可以加 _Noreturn 让编译器知道 (caller 内 goto out 变 dead 但
//     无害; 不强制删除)。
//   解释器 / JIT 共用 helper 接口形态 (cpu_t*, cause, tval); JIT translator emit
//   的 host code 在每个 may-trap 点也插入对齐检查 + 调本 helper, 与解释器同模式。
// ----------------------------------------------------------------------------
static void trap_raise_exception(cpu_t *hart, uint32_t cause, uint32_t tval) {
    fprintf(stderr,
            "[trap_raise_exception] cause=%u tval=0x%08" PRIx32
            " mepc(=hart pc)=0x%08" PRIx32 "  (a_01_4 placeholder)\n",
            cause, tval, hart->regs[0]);
}

void interpret_one_block(cpu_t *hart, tlb_t *current_tlb,
                         uint8_t *hva_pc, uint32_t *count_out) {
    (void)current_tlb;          // a01_3 不用 (load/store + mmu_walker_* 没接)

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
            trap_raise_exception(hart, /*cause*/0u, /*tval*/_t);            \
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

            // ---- 兜底 ----
            case OP_UNSUPPORTED:
                // 不写 rd, 不 advance pc — pc 停在触发指令上 (RV trap 语义对齐: 真接 trap.c
                // 后这里改为 trap_raise(2, mtval=d.raw_inst) + longjmp, dispatcher 主循环
                // 不返回这里; 当前 a01_3 用 break 让 dispatcher 看 count_out 知道 block
                // 跑了多少条, fixture 末尾 ecall 触发即停)。
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
