//
// Created by liujilan on 2026/4/28.
// a01_3 interpreter 模块实现 (算术 / 逻辑 / 立即数子集)。
//
// 顶部模块文档见 interpreter.h。fast/slow path 协议见 dummy.txt §1; x0 编码见 §2;
// TLB 派发见 §4。
//

#include "interpreter.h"

#include "cpu.h"
#include "decode.h"
#include "tlb.h"

#include <stdint.h>
#include <string.h>     // memcpy: 4 字节取指, 防 strict-aliasing

// ----------------------------------------------------------------------------
// a01_3 失控保护 hard limit:
// fetch loop 强制最多跑 64 条, 防止 fixture 写错时无限循环 (例如忘写 ecall 末尾)。
// 数字 64 与 plan.md §1.23.2 软边界初版默认对齐 ("64 不是绝对值, 在 [32, 128] 之间都
// 合理。初版定 64 起步, 性能数据出来后再调"); 真做软边界 (a_01_4) 时移到 config.h, 并
// 加跨 4K page 检查 + 真边界 op (branch/jal/jalr/sfence/wfi/...) 截断 + 与 translator
// 共享同一个常量 (file_plan §8.interpreter G4 / R3)。
// a01_3 fixture 4 条远低于 64, 不会触发限制。
// ----------------------------------------------------------------------------
#define INTERP_A01_3_HARD_LIMIT  64u

void interpret_one_block(cpu_t *hart, tlb_t *current_tlb,
                         uint8_t *hva_pc, uint32_t *count_out) {
    (void)current_tlb;          // a01_3 不用 (load/store + mmu_walker_* 没接)

    // dummy.txt §2 局部垃圾桶变量: 写 x0 的 dead store 落点。
    // 编译器 DCE 会把这个 store 干掉, 等于 NO-OP; 保留是为统一 "所有写都通过同一个宏" 风格,
    // 让未来 IR / 后端不感知 x0 特殊性 (a01_3 体现不到, 但统一)。
    uint32_t x0_garbage = 0;
    (void)x0_garbage;            // 防 -Wunused-variable; 真有 WRITE_REG(0,...) 时编译器会用

    // 局部宏: hart / x0_garbage 隐式捕获 (interpret_one_block 函数内, 上下文清楚)。
    // 函数末尾 #undef, 不污染翻译单元其它部分。
    // 注: 调用方必须传简单 lvalue (decode 后 d.rd / d.rs1 / d.rs2 都是 struct 字段, 无副作用)。
    #define READ_REG(r)       ((r) == 0u ? 0u : hart->regs[r])
    #define WRITE_REG(r, val) (*((r) == 0u ? &x0_garbage : &hart->regs[r]) = (uint32_t)(val))

    uint32_t count = 0;

    while (count < INTERP_A01_3_HARD_LIMIT) {
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
    }

out:
    *count_out = count;

    #undef READ_REG
    #undef WRITE_REG
}
