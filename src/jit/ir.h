//
// jit/ir.h —— JIT 自制 IR 定义 (POD; backend 视角 host-agnostic + ISA-agnostic).
//
// ============================================================================
// 设计依据 (plan §1.21 三层架构 + IR 选择 = 选项 B)
// ============================================================================
//
// 三层职责 (Dispatcher / Translator / JitBackend):
//   - Translator 知道 RISC-V, 不知道 host  → 产出 IR (后端无关)
//   - JitBackend 知道 host, 不知道 RISC-V  → 消费 IR
//   - 三层之间的接口 = 自制 IR, 这是分层成立的关键
//
// IR vs decoded_inst_t (重要区分):
//   - decoded_inst_t (core/decode.h) 是 **RV 视角** 三地址码 — op_kind_t 名字
//     OP_LUI / OP_AUIPC / OP_BEQ / ... 都是 RV opcode; interpreter / translator
//     共享 decode
//   - ir_inst_t (本头) 是 **backend 视角** host-agnostic + ISA-agnostic 中间形式
//     — backend 不感知 RV opcode, 只看 IR op
//   - 两者形态相似但定位不同; backend 直接消费 decoded_inst_t = plan §1.21
//     否决的选项 A "无 IR / backend 被迫 RV 化"
//
// ============================================================================
// 真翻译范围 (70 op = 62 RV32I+扩展 + 8 RV32M)
// ============================================================================
//   RV32I 算术 21 (U-type 2 + I-imm 9 + R-type 10)
//   LOAD/STORE 8 (LOAD 5 + STORE 3; helper call + BARE/SV32 fast path)
//   CSR 6 + AMO 9 + LR/SC 2 + FENCE/FENCE_I 2 (helper call; 硬边界)
//   BRANCH 6 + JAL + JALR + SYSTEM 6 (硬边界; 控制流; 末段出口模板分流)
//   RV32M 8 (MUL 4 + DIV/REM 4; 非边界; emit_ir_muldiv inline; 详 emit doc)
//
// 出口模板 3 哨兵 (IR_OP_UNSUPPORTED + IR_OP_DISPATCH_EXIT + DISPATCH_EXIT_RUNTIME):
//   IR_OP_UNSUPPORTED          - enum 完整性哨兵 (永不真出现在 IR buffer;
//                                 -Wswitch-enum + -Werror 强制 backend switch
//                                 完整, default case 写 __builtin_unreachable /
//                                 abort 兜底). 跟 decode.h OP_UNSUPPORTED 同名
//                                 陷阱: decode 端真撞, IR 端哨兵不撞.
//   IR_OP_DISPATCH_EXIT        - 块出口模板 (compile-time imm target); target_pc
//                                 字段填下一块入口 pc. backend emit 写 cpu->pc
//                                 + 写 count_out + epilogue + ret. 大多数 IR 流
//                                 末尾都是这条.
//   IR_OP_DISPATCH_EXIT_RUNTIME - 块出口模板 (runtime reg target; 给 JALR 用).
//                                 backend emit 从约定 scratch reg r9d 读 target
//                                 PC 写 cpu->pc, 其他跟 DISPATCH_EXIT 一致.
//
// 算术 21 op 语义 (op_kind 名字直观, 不列等式):
//   U-type 2: LUI / AUIPC (translator 端 AUIPC 常量折叠 baked_pc + d.imm; 跟
//     QEMU 默认 + rv8 一致)
//   I-imm 9: ADDI/SLTI/SLTIU/XORI/ORI/ANDI/SLLI/SRLI/SRAI (SLLI/SRLI/SRAI shamt
//     在 imm 低 5 位)
//   R-type 10: ADD/SUB/SLL/SLT/SLTU/XOR/SRL/SRA/OR/AND
//
//   rd ∈ x1-x5 走固定 host reg 路径; 否则 load tmp/op/store tmp; rd = x0 直接
//   跳过写 (dummy.txt §2 dead store 体例).
//
// 内存访问 + 系统 op 概要 (字段约定见下方 ir_inst_t doc):
//   LOAD 5: LB/LH/LW/LBU/LHU — rs1+imm = gva, rd 接 sext/zext 后值
//   STORE 3: SB/SH/SW — rs1+imm = gva, rs2 = value
//   CSR 6: CSRRW/RS/RC + I 变体 — imm 字段存 csr_addr (12-bit), RW/RS/RC 用 rs1
//     寄存器号, RWI/RSI/RCI 用 rs1 字段当 5-bit zimm; 跟 decoded_inst_t 同做法
//   AMO 9: AMO_ADD/SWAP/XOR/OR/AND/MIN/MAX/MINU/MAXU_W — atomic RMW (32-bit);
//     aq/rl 当前忽略 (见下方 "aq/rl 忽略" 段)
//   LR/SC 2: LR_W (rd = M[rs1], 建立 reservation) / SC_W (cond store, rd 接
//     0 成功 / 1 失败)
//   MISC-MEM 2: FENCE (BARE host strong order 退化 nop) / FENCE_I (i-cache flush
//     + lrsc_clear_self; Zifencei)
//
// 控制流 + SYSTEM 14 op:
//   BRANCH 6: BEQ/BNE/BLT/BGE/BLTU/BGEU — 双出口块体, BRANCH IR 自 emit cmp +
//     jcc + Label; taken 边内嵌 emit_dispatch_exit(target_pc=ir.target_pc=
//     taken_pc) + emit_epilogue; fall-through 边走块末 DISPATCH_EXIT (target_pc
//     = cur_pc = fall_through_pc). 一块两份 epilogue, asmjit JitRuntime page-
//     granularity 不撞限.
//   JAL: IR 自身 emit 写 rd = pc + pc_step (compile-time imm, 装在 ir.target_pc);
//     块末 DISPATCH_EXIT 装 target_pc = pc + imm = jump target (translator 算好).
//   JALR: IR 自身 emit 算 runtime target = (rs1 + imm12) & ~1u 存约定 scratch
//     reg r9d + 写 rd = pc + pc_step (compile-time imm 装在 ir.target_pc);
//     块末 DISPATCH_EXIT_RUNTIME 从 r9d 读 target 写 cpu->pc. JALR 跟末 RUNTIME
//     之间无 helper call, r9d caller-saved 安全 (后续若加 instrumentation 需改).
//   SYSTEM 6:
//     ECALL/EBREAK    backend emit `call trap_raise_exception(...)` (_Noreturn
//                     longjmp); cause 编码 inline 算
//     MRET/SRET       backend emit `call {m,s}ret_helper(hart, raw_inst)`;
//                     helper 翻 mstatus MIE/MPIE/MPP + MDT/SDT Smdbltrp 联动;
//                     写 hart->regs[0] = {m,s}epc. 末段 DISPATCH_EXIT_COUNT_ONLY
//                     不写 cpu->pc (helper 已写)
//     SFENCE_VMA      backend emit PRIV_CHECK_OR_TRAP(PRIV_S) inline + `call
//                     sfence_vma_helper(hart, vaddr_val, asid_val, rs1, rs2)`;
//                     末段走标准 DISPATCH_EXIT (helper 不写 cpu->pc)
//     WFI             backend emit TW 检查 inline + `call wfi_wait(...)` (block
//                     阻塞 cond_wait 跟 interpreter 同行为) + `call lrsc_clear_self`
//                     + `add cpu->regs[0], 4` 自推进. 末段 DISPATCH_EXIT_COUNT_ONLY
//
// CSR/FENCE/FENCE_I/BRANCH/JAL/JALR/SYSTEM 全是硬边界 op (plan §1.23.1 表 4 +
// 表 7), translator emit 完该 op 后立刻 emit 末段出口模板切块.
//
// RV32M 8 op 形态 (非边界):
//   MUL 4 (MUL/MULH/MULHU/MULHSU): 32x32 multiply, MUL 低 32, MUL*H 高 32
//     (signed/unsigned 视角差异); host x86 imul/mul (单 op 形式 → edx:eax =
//     64-bit result) 直接 emit. backend 走 emit_ir_muldiv (sibling of
//     emit_ir_arith).
//   DIV 4 (DIV/DIVU/REM/REMU): 32-bit divide; by-0 跟 INT_MIN/-1 overflow edge
//     case inline branch 兜在 idiv 前 (避 x86 #DE trap; 镜像 interpreter.c:357-397
//     语义). 不走 helper (M 是纯算术, 无内存副作用, 无 trap).
//
// ============================================================================
// 设计约定
// ============================================================================
//
// aq/rl 忽略 (AMO + LR/SC memory order 修饰):
//   当前完全忽略, 跟 Spike (riscv-isa-sim insns/amoadd_w.h case 不读 aq/rl) +
//   rvemu (d0iasm cpu.rs `let _aq = ...; // TODO`) 同体例. 项目 host x86_64
//   strong order + 起步阶段不跑 RVWMO weak memory consistency 严格 stress,
//   忽略行为符合用户态单机 emulator 工业惯例.
//   真做时按 QEMU 体例 (target/riscv/insn_trans/trans_rva.c.inc) translator
//   端 emit fence (tcg_gen_mb 风格), IR 不存 — 是局部增量, 不破坏现 IR 设计;
//   weak host (ARM/AArch64) JIT 才有真收益, x86_64 host 永远是 no-op.
//   (plan §2 改进项 #8)
//
// RVC: 不在 ir_op_kind_t 维度扩 enum — decode_rvc (decode.c) 把 RVC 折成 32-bit
//   op_kind_t (C.ADD→OP_ADD / C.BEQZ→OP_BEQ / 等), translator 走 d.pc_step 字段
//   (PC_STEP_RVC=2 或 PC_STEP_RV=4) 推 cur_pc, IR 跟 backend 都 RVC-agnostic.
//   ir.target_pc 已含 RVC 跨步预计算 (translator BRANCH/JAL/JALR taken_pc 用
//   cur_pc + d.imm; fall_through_pc 用 cur_pc + d.pc_step). backend 端无
//   hardcoded "+4" 残留 (WFI 自推进 +4 不参 RVC, RV spec 强制 32-bit encoding),
//   全读 ir.target_pc / exit_inst.target_pc.
//
// IR 流形态:
//   [真翻译 op 前缀 N 条 (0 ≤ N ≤ BLOCK_INST_LIMIT - 1)]
//   [出口模板 1 条 (target_pc = 下一块入口 pc = 截断指令 PC / 软边界出块 PC /
//    硬边界 op 后的 PC; JALR 块用 DISPATCH_EXIT_RUNTIME, 其他用 DISPATCH_EXIT)]
//
// 块前缀空 (N=0, 第一条就是非翻译 op): translator 仍 emit 出口模板一条,
// 但 backend.compile_block 检测到 IR 流只有出口模板时返 JIT_ERR_NOT_IMPLEMENTED
// (块前缀无真翻译指令, 不值得 install), jit_entry.cc 走 set_blacklist 路径;
// dispatcher 下次 lookup BLACK miss → interpret 兜底真执行那条 RV 指令.
//
// regime 分流: backend.compile_block 接 regime_t 参数; REGIME_BARE / REGIME_SV32_S
// / REGIME_SV32_U 三 regime 全真编 (LOAD/STORE fast path 按 regime 不同 baked).
// 详 mmu.h regime_t 接口分裂明文 + backend_asmjit.cc emit_ir_load/store doc.
//
// 命名:
//   - 类型 ir_*_t (ir_op_kind_t / ir_inst_t)
//   - enum 值 IR_OP_* (IR_OP_UNSUPPORTED / IR_OP_DISPATCH_EXIT / 70 真翻译 op)
//   - 跟 decode.h op_kind_t / OP_* 平行, 区分 RV 视角 vs backend 视角
//

#ifndef JIT_IR_H
#define JIT_IR_H

#include <stdint.h>

#include "riscv.h"   // uxlen_t (target_pc 字段; dummy.txt §13 typedef family)


// ----------------------------------------------------------------------------
// ir_op_kind_t —— IR op 分类 (3 哨兵/出口 + 70 真翻译 op)
//
// IR_OP_UNSUPPORTED           - enum 完整性哨兵 (永不真出现在 IR buffer);
//                              backend default case 写 __builtin_unreachable
//                              / abort 兜底
// IR_OP_DISPATCH_EXIT         - 块出口模板 (compile-time imm target); target_pc
//                              填下一 pc
// IR_OP_DISPATCH_EXIT_RUNTIME - 块出口模板 (runtime reg target; JALR 用);
//                              backend emit 从约定 scratch reg r9d 读 target
//                              写 cpu->pc, 其他跟 DISPATCH_EXIT 一致
//
// 70 真翻译 op 按 decode.h op_kind_t 分段顺序排 (RV32I 算术 21 → LOAD 5 →
// STORE 3 → CSR 6 → AMO 9 → LR/SC 2 → MISC-MEM 2 → BRANCH 6 → JAL → JALR →
// SYSTEM 6 → RV32M 8); 每条 op 语义见顶段 doc.
//
// IR_OP_UNSUPPORTED=0 / IR_OP_DISPATCH_EXIT=1 / IR_OP_DISPATCH_EXIT_RUNTIME=2
// sentinel 显式值保留 (其他位置可自由调整 enum 顺序而不影响 sentinel 语义);
// 真翻译 op 不指定显式值, switch 按名字访问. 后续扩 op 按 decode.h 段追加末段,
// sentinel 不动.
// ----------------------------------------------------------------------------
typedef enum {
    IR_OP_UNSUPPORTED           = 0,
    IR_OP_DISPATCH_EXIT         = 1,
    IR_OP_DISPATCH_EXIT_RUNTIME = 2,

    /* ---- U-type (2) ---- */
    IR_OP_LUI,
    IR_OP_AUIPC,

    /* ---- I-type OP-IMM (9; SLLI/SRLI/SRAI shamt 在 imm 低 5 位) ---- */
    IR_OP_ADDI,
    IR_OP_SLTI,
    IR_OP_SLTIU,
    IR_OP_XORI,
    IR_OP_ORI,
    IR_OP_ANDI,
    IR_OP_SLLI,
    IR_OP_SRLI,
    IR_OP_SRAI,

    /* ---- R-type OP (10) ---- */
    IR_OP_ADD,
    IR_OP_SUB,
    IR_OP_SLL,
    IR_OP_SLT,
    IR_OP_SLTU,
    IR_OP_XOR,
    IR_OP_SRL,
    IR_OP_SRA,
    IR_OP_OR,
    IR_OP_AND,

    /* ---- I-type LOAD (5; rs1+imm = gva) ---- */
    IR_OP_LB,
    IR_OP_LH,
    IR_OP_LW,
    IR_OP_LBU,
    IR_OP_LHU,

    /* ---- S-type STORE (3; rs1+imm = gva, rs2 = value) ---- */
    IR_OP_SB,
    IR_OP_SH,
    IR_OP_SW,

    /* ---- I-type SYSTEM CSR (6; imm = csr_addr 12-bit, RW/RS/RC 用 rs1 寄
            存器号, RWI/RSI/RCI 用 rs1 字段当 5-bit zimm) ---- */
    IR_OP_CSRRW,
    IR_OP_CSRRS,
    IR_OP_CSRRC,
    IR_OP_CSRRWI,
    IR_OP_CSRRSI,
    IR_OP_CSRRCI,

    /* ---- R-type A 扩展 Zaamo (9; rs1=gva, rs2=value) ---- */
    IR_OP_AMO_ADD_W,
    IR_OP_AMO_SWAP_W,
    IR_OP_AMO_XOR_W,
    IR_OP_AMO_OR_W,
    IR_OP_AMO_AND_W,
    IR_OP_AMO_MIN_W,
    IR_OP_AMO_MAX_W,
    IR_OP_AMO_MINU_W,
    IR_OP_AMO_MAXU_W,

    /* ---- R-type A 扩展 Zalrsc (2) ---- */
    IR_OP_LR_W,    /* 只用 rs1 (= gva), rd 接 raw 32-bit */
    IR_OP_SC_W,    /* rs1 (= gva) + rs2 (= value), rd = 0 成功 / 1 失败 */

    /* ---- MISC-MEM (2; 无 reg 字段, 硬边界后续紧跟 DISPATCH_EXIT) ---- */
    IR_OP_FENCE,
    IR_OP_FENCE_I,

    /* ---- B-type BRANCH (6; 硬边界; 双出口; target_pc 装 taken_pc) ---- */
    IR_OP_BEQ,
    IR_OP_BNE,
    IR_OP_BLT,
    IR_OP_BGE,
    IR_OP_BLTU,
    IR_OP_BGEU,

    /* ---- J-type JAL (1; 硬边界; compile-time target;
            target_pc 装 rd 写入值 = pc + pc_step) ---- */
    IR_OP_JAL,

    /* ---- I-type JALR (1; 硬边界; runtime target;
            target_pc 装 rd 写入值; 末 IR 用 DISPATCH_EXIT_RUNTIME) ---- */
    IR_OP_JALR,

    /* ---- I-type SYSTEM (6; 全硬边界) ---- */
    IR_OP_ECALL,
    IR_OP_EBREAK,
    IR_OP_MRET,
    IR_OP_SRET,
    IR_OP_SFENCE_VMA,
    IR_OP_WFI,

    /* ---- R-type RV32M (8; 非边界; MUL 4 + DIV/REM 4) ----
     * MUL/MULH/MULHU/MULHSU: 32x32 multiply, MUL 低 32, MUL*H 高 32 (signed/
     *   unsigned 视角差异); host x86 imul/mul (单 op 形式 → edx:eax = 64-bit
     *   result) 直接 emit. backend 走 emit_ir_muldiv (sibling of emit_ir_arith).
     * DIV/DIVU/REM/REMU: 32-bit divide; by-0 跟 INT_MIN/-1 overflow edge case
     *   inline branch 兜在 idiv 前 (避 x86 #DE trap; 镜像 interpreter.c:357-397
     *   语义). 不走 helper (M 是纯算术, 无内存副作用, 无 trap). */
    IR_OP_MUL,
    IR_OP_MULH,
    IR_OP_MULHSU,
    IR_OP_MULHU,
    IR_OP_DIV,
    IR_OP_DIVU,
    IR_OP_REM,
    IR_OP_REMU,
} ir_op_kind_t;


// ----------------------------------------------------------------------------
// ir_inst_t —— 单条 IR 指令 (POD; 三地址码 + 块出口 target_pc 联合)
//
// 字段表保持 6 个 + 1 个 raw_inst (一 RV op 一 IR op_kind 拆细):
//   - funct3 / funct5 不加 (跟 op_kind 重复编码; 例 LB/LH/LW 已拆 op_kind,
//     funct3 信息已在 op_kind 内; AMO 9 op 各占独立 op_kind, funct5 同冗余)
//   - csr_addr 复用 imm 字段 (跟 decoded_inst_t 同做法; CSR 入口判 priv / RO
//     写 / illegal csr 都由 csr_op helper 处理, backend 只 emit
//     `call csr_op(hart, csr_addr=imm, ...)` 静态绑定)
//   - aq/rl 不加 (ir.h 顶段 doc 'aq/rl 忽略' 段)
//   - raw_inst 加: CSR 6 op 调 csr_op + SYSTEM MRET/SRET/SFENCE/WFI 调 helper
//     时需要 raw_inst 作 illegal inst trap mtval 来源 (RV spec §3.1.16 强制);
//     其他 op (算术 / LOAD/STORE / AMO/LR/SC/FENCE / BRANCH/JAL/JALR / RV32M)
//     不读 raw_inst 字段
//
// 字段使用约定 (按 kind 不同):
//   kind = IR_OP_DISPATCH_EXIT:
//     target_pc - 下一块入口 PC (硬边界算 / 软边界出块 PC / 截断指令 PC; JAL 时
//                  装 jump target = pc + imm; BRANCH 时装 fall_through_pc;
//                  MRET/SRET/WFI 装 pc + pc_step 但 backend 走 COUNT_ONLY 不读)
//     rd/rs1/rs2/imm - 不用
//   kind = IR_OP_DISPATCH_EXIT_RUNTIME (JALR 用):
//     target_pc / rd / rs1 / rs2 / imm - 不用 (target 从 scratch reg r9d 读,
//                                              translator 由前条 JALR IR 装好)
//   kind ∈ U-type {IR_OP_LUI, IR_OP_AUIPC}:
//     rd  - 寄存器号 0..31
//     imm - 32 位常数 (translator 端: LUI = decode.imm 直传; AUIPC = baked_pc +
//           decode.imm 合并 — 选 a 常量折叠, 跟 QEMU 默认 + rv8 一致)
//     rs1 / rs2 / target_pc - 不用
//   kind ∈ I-imm 算术 {IR_OP_ADDI, IR_OP_SLTI, IR_OP_SLTIU, IR_OP_XORI,
//                       IR_OP_ORI, IR_OP_ANDI, IR_OP_SLLI, IR_OP_SRLI, IR_OP_SRAI}:
//     rd / rs1 - 寄存器号 0..31
//     imm      - 12 位符号扩展立即数 (跟 decoded_inst_t.imm 体例对偶; SLLI/SRLI/
//                SRAI 时低 5 位是 shamt, backend 端 & 0x1F)
//     rs2 / target_pc - 不用
//   kind ∈ R-type 算术 {IR_OP_ADD, IR_OP_SUB, IR_OP_SLL, IR_OP_SLT, IR_OP_SLTU,
//                       IR_OP_XOR, IR_OP_SRL, IR_OP_SRA, IR_OP_OR, IR_OP_AND}:
//     rd / rs1 / rs2 - 寄存器号 0..31
//     imm / target_pc - 不用
//   kind ∈ LOAD {IR_OP_LB, IR_OP_LH, IR_OP_LW, IR_OP_LBU, IR_OP_LHU}:
//     rd / rs1 - 寄存器号 0..31
//     imm      - 12 位符号扩展立即数 (offset; gva = rs1 + imm)
//     rs2 / target_pc - 不用
//   kind ∈ STORE {IR_OP_SB, IR_OP_SH, IR_OP_SW}:
//     rs1 / rs2 - 寄存器号 (rs1 base, rs2 value)
//     imm       - 12 位符号扩展立即数 (offset; gva = rs1 + imm)
//     rd / target_pc - 不用 (STORE 无 rd)
//   kind ∈ CSR R/S/C {IR_OP_CSRRW, IR_OP_CSRRS, IR_OP_CSRRC}:
//     rd / rs1 - 寄存器号 (rd 写回旧值; rs1 提供新值)
//     imm      - csr_addr (12-bit, 高 20 位 0; 跟 decoded_inst_t.imm CSR 段对偶)
//     raw_inst - RV 原始 32-bit 指令编码 (illegal trap mtval 用; RV spec §3.1.16)
//     rs2 / target_pc - 不用
//   kind ∈ CSR I 变体 {IR_OP_CSRRWI, IR_OP_CSRRSI, IR_OP_CSRRCI}:
//     rd  - 寄存器号 (rd 写回旧值)
//     rs1 - **5-bit zimm** (rs1 字段位置共用; backend emit 时直接用此数值, 不查 regs)
//     imm - csr_addr (同 R/S/C)
//     raw_inst - 同 R/S/C
//     rs2 / target_pc - 不用
//   kind ∈ AMO 9 {IR_OP_AMO_*}:
//     rd / rs1 / rs2 - 寄存器号 (rd 接旧值; rs1=gva; rs2=value)
//     imm / target_pc - 不用 (AMO 立即数为 0)
//   kind = IR_OP_LR_W:
//     rd / rs1 - 寄存器号 (rd 接 raw 32-bit; rs1=gva)
//     rs2 / imm / target_pc - 不用 (LR 立即数为 0)
//   kind = IR_OP_SC_W:
//     rd / rs1 / rs2 - 寄存器号 (rd = 0 成功 / 1 失败; rs1=gva; rs2=value)
//     imm / target_pc - 不用
//   kind ∈ FENCE {IR_OP_FENCE, IR_OP_FENCE_I}:
//     rd / rs1 / rs2 / imm / target_pc - 不用 (helper 无 reg 参数)
//   kind ∈ BRANCH 6 {IR_OP_BEQ, IR_OP_BNE, IR_OP_BLT, IR_OP_BGE, IR_OP_BLTU,
//                     IR_OP_BGEU}:
//     rs1 / rs2 - 寄存器号 (条件比较两操作数)
//     target_pc - taken_pc (translator 算 cur_pc + (uint32_t)d.imm; backend
//                  emit cmp + jcc + taken_label: emit_dispatch_exit(taken_pc)
//                  + emit_epilogue; fall-through 走块末 DISPATCH_EXIT)
//     rd / imm / raw_inst - 不用 (imm 装也可调试 trace; backend 不读)
//   kind = IR_OP_JAL:
//     rd        - 链接寄存器号 (rd = pc + pc_step)
//     target_pc - rd 写入值 = pc + pc_step (compile-time imm, translator 算)
//     rs1 / rs2 / imm / raw_inst - 不用 (jump target = pc + imm 装在末
//                                          DISPATCH_EXIT.target_pc, 不在 IR 自身)
//   kind = IR_OP_JALR:
//     rd        - 链接寄存器号
//     rs1       - 基址寄存器号
//     imm       - 12-bit signed imm12 (offset; backend 算 target = (rs1 + imm) & ~1u)
//     target_pc - rd 写入值 = pc + pc_step (compile-time imm)
//     rs2 / raw_inst - 不用 (末 IR 是 DISPATCH_EXIT_RUNTIME 从 r9d 读 target)
//   kind ∈ SYSTEM 6 {IR_OP_ECALL, IR_OP_EBREAK, IR_OP_MRET, IR_OP_SRET,
//                     IR_OP_SFENCE_VMA, IR_OP_WFI}:
//     ECALL/EBREAK:
//       rd / rs1 / rs2 / imm / target_pc - 不用 (cause inline 算, tval=0)
//       raw_inst - 不用 (ECALL/EBREAK 不需要 mtval)
//     MRET/SRET:
//       rd / rs1 / rs2 / imm / target_pc - 不用
//       raw_inst - 触发指令原码 (helper 内 PRIV_CHECK 失败时填 mtval)
//     SFENCE_VMA:
//       rs1 / rs2 - 寄存器号 (rs1=vaddr base 跟 rs2=ASID; backend 读 regs 装入参;
//                    rs1/rs2=0 触发 RV magic encoding 走全清, 详 isa/sfence.c)
//       raw_inst  - 触发指令原码 (PRIV_CHECK 失败 mtval)
//       rd / imm / target_pc - 不用
//     WFI:
//       raw_inst - 触发指令原码 (TW 检查失败 mtval)
//       rd / rs1 / rs2 / imm / target_pc - 不用
//   kind ∈ RV32M 8 {IR_OP_MUL, IR_OP_MULH, IR_OP_MULHSU, IR_OP_MULHU,
//                     IR_OP_DIV, IR_OP_DIVU, IR_OP_REM, IR_OP_REMU}:
//     rd / rs1 / rs2 - 寄存器号 0..31
//     imm / target_pc / raw_inst - 不用 (M 是纯算术, 无 trap, 无 mtval 需求)
//   kind = IR_OP_UNSUPPORTED:
//     永不真出现 — 哨兵, 字段不读
//
// 字段类型选 uint8_t 寄存器号 (节省 IR 流内存; RV reg 0..31 占 5 bit, uint8_t
// 够) + int32_t imm (跟 decoded_inst_t.imm 同类型).
// ----------------------------------------------------------------------------
typedef struct {
    ir_op_kind_t kind;
    uxlen_t      target_pc;
    uint8_t      rd;
    uint8_t      rs1;
    uint8_t      rs2;
    int32_t      imm;
    u32_t        raw_inst;    /* CSR 6 + SYSTEM MRET/SRET/SFENCE/WFI illegal trap mtval; 其他 op 不读 */
} ir_inst_t;


#endif //JIT_IR_H
