//
// Created by liujilan on 2026/4/28.
// a01_3 decode 模块对外接口。
//
// 职责: 把 RV32 32-bit 指令解析成 (op_kind, rd, rs1, rs2, imm, raw_inst) 的纯数据结构,
//       给 interpreter / translator 共用。decode 是纯函数, 不读 / 不写 cpu_t。
//
// op_kind_t 当前范围 (a01_3 + a01_4):
//   - a01_3: 算术 / 逻辑 / 立即数子集 (21 个真 op)
//   - a01_4: 控制流 (branch 6 + jal + jalr = 8 个真 op)
//   共 29 个真 op + 1 个 OP_UNSUPPORTED 兜底。
//   不在此范围的 opcode (load / store / fence / csr / ecall / ebreak / mret / sret / wfi /
//   amo / 等) decode 全部归 OP_UNSUPPORTED。
//
// op_kind_t 增长策略: 真要支持新 op 时再加 enum case + interpreter switch case + (未来)
//   translator emit case; -Wswitch-enum + -Werror 强制 switch 一致性, 增量加新 op_kind
//   时编译器逼着补 case, 不会漏。
//
// decoded_inst_t 字段:
//   kind     - op_kind_t, 由 decode 分类
//   rd       - 0..31, 写目标寄存器号 (R/I/U-type 都有; OP_UNSUPPORTED 时按通用编码位置填,
//              但调用方不应使用)
//   rs1, rs2 - 0..31, 源寄存器号 (R/I-type 用 rs1; R-type 还用 rs2; U-type 都没用)
//   imm      - int32_t, 已符号扩展; 含义按 kind 不同:
//                I-type (ADDI/SLTI/...): 12 位 imm 符号扩到 32 位
//                I-type shift (SLLI/SRLI/SRAI): shamt 5 位放低位 (无符号扩展)
//                U-type (LUI/AUIPC):     20 位 imm 左移 12 位 (低 12 位 0)
//                R-type (ADD/SUB/...):   不用, decode 设 0
//   raw_inst - 原始 32 位指令; 留给 debug / illegal-instruction trap 的 mtval 填充
//                (RV 规范规定 illegal trap 的 mtval = 触发指令本身; 未来 a_03 trap.c
//                接 OP_UNSUPPORTED → trap_raise(2) 时, raw_inst 现成可用, 不用反查 hva)
//
// 注: x0 寄存器号约定见 dummy.txt §2 (decode 不特判, rd/rs1/rs2 字段都可能是 0,
//     interpreter 路径自己用 READ_REG / WRITE_REG 宏分流)。
//

#ifndef CORE_DECODE_H
#define CORE_DECODE_H

#include <stdint.h>

typedef enum {
    // ---- U-type (2) ----
    OP_LUI = 0,         // U-type, opcode 0x37
    OP_AUIPC,           // U-type, opcode 0x17

    // ---- I-type OP-IMM (9), opcode 0x13 ----
    OP_ADDI,
    OP_SLTI,
    OP_SLTIU,
    OP_XORI,
    OP_ORI,
    OP_ANDI,
    OP_SLLI,            // shamt 在 imm 低 5 位
    OP_SRLI,
    OP_SRAI,

    // ---- R-type OP (10), opcode 0x33 ----
    OP_ADD,
    OP_SUB,
    OP_SLL,
    OP_SLT,
    OP_SLTU,
    OP_XOR,
    OP_SRL,
    OP_SRA,
    OP_OR,
    OP_AND,

    // ---- B-type BRANCH (6), opcode 0x63 ---- a_01_4
    // 6 个变体 funct3 编码 (RV spec): BEQ=000, BNE=001, BLT=100, BGE=101, BLTU=110, BGEU=111
    // (funct3=010 / 011 reserved, decode 归 OP_UNSUPPORTED)。
    // 立即数 13 位有符号 (multiple of 2, imm[0]=0 编码强制); 4 段位拼接见 decode.c。
    OP_BEQ,
    OP_BNE,
    OP_BLT,
    OP_BGE,
    OP_BLTU,
    OP_BGEU,

    // ---- J-type JAL (1), opcode 0x6F ---- a_01_4
    // 立即数 21 位有符号 (multiple of 2, imm[0]=0 编码强制); 4 段位拼接见 decode.c。
    // ±1 MiB 跳转范围。
    OP_JAL,

    // ---- I-type JALR (1), opcode 0x67, funct3=000 ---- a_01_4
    // 立即数复用 I-type 12 位有符号 (与 ADDI / SLTI 等同型)。
    // 目标 = (rs1 + imm) & ~1u (RV spec 强制 mask LSB)。
    // funct3 != 0 reserved by spec, decode 归 OP_UNSUPPORTED。
    OP_JALR,

    // ---- 兜底 ----
    // 不在当前范围的 opcode (load 0x03 / store 0x23 / fence 0x0F / system 0x73 / amo 0x2F /
    // 真非法 opcode / 真非法 funct3 子段) 全部归这里。
    // interpreter 收到这个 case break 出 fetch loop, dispatcher 看 count_out 知道
    // 本次 block 跑了多少条; 未来 a_01_5 trap.c 接入后改为 trap_raise(2) Illegal Instruction
    // (RV cause code 2), mtval = decoded.raw_inst (RV 规范要求)。
    OP_UNSUPPORTED,
} op_kind_t;

// PC 步进量, 由 decode 一次决定, fetch loop 末尾统一 hart->regs[0] += d.pc_step。
//   PC_STEP_RV    : 普通 RV 指令 (RV32 / RV64 都是 4 字节固定长度), fetch loop +4
//   PC_STEP_RVC   : 16-bit C 扩展 (compressed), fetch loop +2
//   PC_STEP_NONE  : control flow op (branch / jal / jalr / mret / sret / ...), case
//                   自描述 pc, fetch loop += 0 是 NOP (case 内必须 hart->regs[0] = ...,
//                   漏 = 死循环 → 64 hard limit 立刻 break, 容易发现)
//
// 设计动机: pc 推进数据驱动 (decode 决定), fetch loop 不 if (is_boundary), 与 ISA 长度
// 处理统一 (RVC 与 control flow 在 fetch loop 看法一致, 都是"d.pc_step 多少就推多少")。
//
// 注: 块边界 (is_block_boundary_inst) 与 pc_step 是两个独立维度:
//   - branch 跳/不跳: pc_step=PC_STEP_NONE (case 自设 pc), is_boundary=true (块尾)
//   - sfence.vma:    pc_step=PC_STEP_RV (普通 +4), is_boundary=true (改 TLB)
//   - addi:          pc_step=PC_STEP_RV, is_boundary=false
#define PC_STEP_RV    4u
#define PC_STEP_RVC   2u
#define PC_STEP_NONE  0u

typedef struct {
    op_kind_t kind;
    uint32_t  rd;
    uint32_t  rs1;
    uint32_t  rs2;
    int32_t   imm;
    uint32_t  raw_inst;     // 原始指令; debug / illegal trap mtval 用
                            // (32-bit 指令 = 完整 32 位; 16-bit RVC 时 = 低 16 位 + 高
                            // 16 位 0, 因为 decode_rvc 用 (uint16_t)inst cast 截断;
                            // RV illegal trap mtval 规范也是按指令长度填, 取低 16 位即可)
    uint32_t  pc_step;      // PC_STEP_RV / PC_STEP_RVC / PC_STEP_NONE
} decoded_inst_t;

// 纯函数: 不读 / 不写 cpu_t, 不依赖 mmu / tlb / ram。
// 对于不识别的 opcode, kind = OP_UNSUPPORTED; 其它字段 (rd/rs1/rs2/imm) 仍按通用编码位置
// 填入, 但调用方不应使用 (除 raw_inst 用作 trap mtval)。
decoded_inst_t decode(uint32_t inst);

// ----------------------------------------------------------------------------
// is_block_boundary_inst —— 块边界判定 (硬边界), 共享给 interpreter + 未来 translator
//
// 硬边界 = 必须结束当前 block 的指令 (改变控制流 / 改变全局状态导致后续译码假设失效)。
// 共享 inline 函数保证 interpreter 与 translator 对"什么算硬边界"判断 100% 一致 —
// -Wswitch-enum + -Werror 强制下方 switch 覆盖所有 op_kind_t, 加新 op_kind 时编译器
// 逼着补 case (boundary or 不是), 不会漏。
//
// a_01_3 起步: 没有 boundary op (算术 / 逻辑 / 立即数都不是); OP_UNSUPPORTED 不归 boundary
// (它是 trap 触发, 走 trap_raise + longjmp, 不走"块边界"路径; a_01_3 临时形态用 break +
// 不动 pc 与 RV trap 语义对齐, 不是 boundary)。
//
// a_01_4 加 boundary op: branch (BEQ/BNE/BLT/BGE/BLTU/BGEU) + jal + jalr (8 个)
// a_01_5 加 boundary op: CSR 写 (CSRRW/RS/RC + I 变体共 6 个) + ECALL/EBREAK/MRET (3) +
//                        WFI / SFENCE.VMA / FENCE.I (Zifencei + Zicsr)
//   注: file_plan §7.decode G3 说 "所有 CSR 写都视为软边界 (过度刷新允许)" 简化策略, 即
//       CSR 读 (CSRRS rd, csr, x0 = 读) 不一定要 boundary; 但保险起见统一视为 boundary
//       (a_01_5 真做时再细化)
//
// 软边界 (块长度上限) 不在这里, 由 interpreter / translator 各自循环维护计数器,
// 见 file_plan §7.decode G4 / §8.interpreter R3 + plan.md §1.23.2 (默认 64)。
//
// 返回 int 0/1 (与 codebase 一致, 不引入 bool / stdbool.h; mmu_translate_pc / dispatcher
// 都是 int 风格)。
//
// 待评估 (用户拍 xii): "在指令实现的差不多后, 要考虑是否放入 decoded_inst_t 结构体" —
// 即未来某个 milestone 重新评估是否把 boundary 信息从 helper 移到 decoded_inst_t 字段
// (与 pc_step 同风格, 数据驱动)。当前阶段保持 helper, 因为 boundary 是项目级判断 (策略)
// 而 pc_step 是 ISA 属性 (数据), 两者粒度不同。
// ----------------------------------------------------------------------------
static inline int is_block_boundary_inst(const decoded_inst_t *d) {
    switch (d->kind) {
        // ---- a_01_3 op (全部不是 boundary) ----
        case OP_LUI:   case OP_AUIPC:
        case OP_ADDI:  case OP_SLTI:  case OP_SLTIU: case OP_XORI:
        case OP_ORI:   case OP_ANDI:
        case OP_SLLI:  case OP_SRLI:  case OP_SRAI:
        case OP_ADD:   case OP_SUB:   case OP_SLL:   case OP_SLT:  case OP_SLTU:
        case OP_XOR:   case OP_SRL:   case OP_SRA:   case OP_OR:   case OP_AND:
            return 0;

        // ---- OP_UNSUPPORTED (a_01_3 临时不归 boundary, 走 fetch loop goto out;
        //      a_01_5 真接 trap 后改 trap_raise(2) + longjmp, 也不归 boundary) ----
        case OP_UNSUPPORTED:
            return 0;

        // ---- a_01_4 控制流 (硬边界) ----
        // 8 个 op 都改 pc, 块必须在此结束。interpreter / translator (未来) 在 fetch loop 末
        // 检查本 helper 返回值, 是 1 则 break 出 loop, dispatcher 重新走 block 1+2+3 (重新
        // mmu_translate_pc 拿新 pc 的 hva, 进新一块 block)。
        // 注: 当前 helper 还没真被 caller 用 (Step 3 在 interpreter 接), 改成 return 1 仅
        // 是 "声明事实", 运行时行为要等 Step 3 + Step 4 才生效。
        case OP_BEQ:  case OP_BNE:
        case OP_BLT:  case OP_BGE:  case OP_BLTU: case OP_BGEU:
        case OP_JAL:  case OP_JALR:
            return 1;

        // ---- a_01_5 待加: ----
        // case OP_CSRRW: case OP_CSRRS: case OP_CSRRC:
        // case OP_CSRRWI: case OP_CSRRSI: case OP_CSRRCI:
        // case OP_ECALL: case OP_EBREAK: case OP_MRET:
        //     return 1;
    }
    return 0;  // -Wswitch-enum 下不可达 (所有 enum 必须在 switch 列出); 防御写法
}

#endif //CORE_DECODE_H
