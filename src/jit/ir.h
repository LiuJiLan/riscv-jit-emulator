//
// Created by liujilan on 2026/6/5.
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
//     — backend 不感知 RV opcode, 只看 IR op (例 IR_OP_BRANCH_LT 合并 BLT/BGE
//     signed 版, 参数标 signed)
//   - 两者形态相似但定位不同; backend 直接消费 decoded_inst_t = plan §1.21
//     否决的选项 A "无 IR / backend 被迫 RV 化"
//
// ============================================================================
// T1-T4 真做范围 (62 op = RV32I 算术 21 + LOAD/STORE 8 + CSR 6 + AMO 9 +
//                  LR/SC 2 + FENCE 2 + BRANCH 6 + JAL + JALR + SYSTEM 6)
// ============================================================================
//
// IR_OP_UNSUPPORTED         - enum 完整性哨兵 (永不真出现在 IR buffer; -Wswitch-
//                              enum + -Werror 强制 backend switch 完整, default
//                              case 写 __builtin_unreachable / abort 兜底). 跟
//                              decode.h OP_UNSUPPORTED 同名陷阱: decode 端真撞,
//                              IR 端哨兵不撞.
// IR_OP_DISPATCH_EXIT       - 块出口模板 (compile-time imm target); target_pc
//                              字段填下一块入口 pc. backend emit 写 cpu->pc +
//                              写 count_out + epilogue + ret. 大多数 IR 流末尾
//                              都是这条 (BRANCH 6 / JAL / CSR / FENCE / SYSTEM
//                              ECALL/EBREAK/SFENCE/MRET/SRET/WFI).
// IR_OP_DISPATCH_EXIT_RUNTIME - 块出口模板 (runtime reg target; T4 加给 JALR
//                              用). backend emit 从约定 scratch reg r9d 读
//                              target PC 写 cpu->pc, 其他跟 DISPATCH_EXIT 一致.
//                              IR 流末尾仅 JALR 块用此 sentinel; 其他 13 op 末
//                              仍用 DISPATCH_EXIT.
//
// RV32I 算术全集 21 op (T1+T2; 按 decode.h op_kind_t 同分段顺序):
//   U-type 2:
//     IR_OP_LUI    rd = (uint32_t)imm                  (decode 已 sext20 + <<12)
//     IR_OP_AUIPC  rd = (uint32_t)imm                  (translator 端常量折叠;
//                                                       imm 字段存 baked_pc +
//                                                       d.imm 合并后常数; 跟
//                                                       QEMU 默认 + rv8 一致)
//   I-imm 9 (rs1 + imm; SLLI/SRLI/SRAI 的 shamt 在 imm 低 5 位):
//     IR_OP_ADDI   rd = rs1 + imm
//     IR_OP_SLTI   rd = ((int32_t)rs1 < imm) ? 1 : 0   (signed 比较)
//     IR_OP_SLTIU  rd = (rs1 < (uint32_t)imm) ? 1 : 0  (imm 先 sext 到 32, 再
//                                                       unsigned 比较)
//     IR_OP_XORI   rd = rs1 ^ imm
//     IR_OP_ORI    rd = rs1 | imm
//     IR_OP_ANDI   rd = rs1 & imm
//     IR_OP_SLLI   rd = rs1 << (imm & 0x1F)
//     IR_OP_SRLI   rd = rs1 >> (imm & 0x1F)
//     IR_OP_SRAI   rd = (int32_t)rs1 >> (imm & 0x1F)   (算术右移)
//   R-type 10 (rs1 + rs2):
//     IR_OP_ADD    rd = rs1 + rs2
//     IR_OP_SUB    rd = rs1 - rs2
//     IR_OP_SLL    rd = rs1 << (rs2 & 0x1F)
//     IR_OP_SLT    rd = ((int32_t)rs1 < (int32_t)rs2) ? 1 : 0
//     IR_OP_SLTU   rd = (rs1 < rs2) ? 1 : 0
//     IR_OP_XOR    rd = rs1 ^ rs2
//     IR_OP_SRL    rd = rs1 >> (rs2 & 0x1F)
//     IR_OP_SRA    rd = (int32_t)rs1 >> (rs2 & 0x1F)
//     IR_OP_OR     rd = rs1 | rs2
//     IR_OP_AND    rd = rs1 & rs2
//
// rd ∈ x1-x5 走固定 host reg 路径; 否则 load tmp/op/store tmp; rd = x0 直接
// 跳过写 (dummy.txt §2 dead store 体例).
//
// T3 真做范围 (helper call + BARE regime fast path):
//
//   LOAD 5 (I-type, opcode 0x03; rs1+imm = gva):
//     IR_OP_LB   rd = sext8 (M[gva][7:0])
//     IR_OP_LH   rd = sext16(M[gva+1:gva])
//     IR_OP_LW   rd = M[gva+3:gva]
//     IR_OP_LBU  rd = zext8 (M[gva][7:0])
//     IR_OP_LHU  rd = zext16(M[gva+1:gva])
//   STORE 3 (S-type, opcode 0x23; rs1+imm = gva, rs2 = value):
//     IR_OP_SB   M[gva][7:0]    = rs2[7:0]
//     IR_OP_SH   M[gva+1:gva]   = rs2[15:0]
//     IR_OP_SW   M[gva+3:gva]   = rs2
//   CSR 6 (I-type, opcode 0x73 funct3 != 0; imm 字段存 csr_addr 12-bit;
//          RW/RS/RC 用 rs1 寄存器号, RWI/RSI/RCI 用 rs1 字段当 5-bit zimm —
//          跟 decoded_inst_t 同做法, decode.h:158-162 line 字段约定对偶):
//     IR_OP_CSRRW   rd = csr; csr = rs1
//     IR_OP_CSRRS   rd = csr; csr |= rs1
//     IR_OP_CSRRC   rd = csr; csr &= ~rs1
//     IR_OP_CSRRWI  rd = csr; csr = zimm5
//     IR_OP_CSRRSI  rd = csr; csr |= zimm5
//     IR_OP_CSRRCI  rd = csr; csr &= ~zimm5
//   AMO 9 (R-type, opcode 0x2F funct3=0x2 funct5 区分; rs1=gva, rs2=value):
//     IR_OP_AMO_ADD_W / SWAP_W / XOR_W / OR_W / AND_W / MIN_W / MAX_W /
//     MINU_W / MAXU_W   atomic RMW W (32-bit) 一族; aq/rl 当前忽略 (见下方
//     "aq/rl TODO")
//   LR/SC 2 (R-type, opcode 0x2F funct3=0x2 funct5=0x02/0x03):
//     IR_OP_LR_W   rd = M[rs1]; 建立 self.reservation = rs1
//     IR_OP_SC_W   rd = (self.reservation == rs1) ? 0 : 1; success 则 M[rs1] = rs2
//   MISC-MEM 2 (opcode 0x0F; 无 reg 操作数):
//     IR_OP_FENCE     memory barrier (BARE host strong order 退化 nop)
//     IR_OP_FENCE_I   instruction-cache flush + lrsc_clear_self (Zifencei)
//
// CSR/FENCE/FENCE_I 是硬边界 op (plan §1.23.1 表 4 + 表 7), translator emit
// 完该 op 后立刻 emit IR_OP_DISPATCH_EXIT 切块.
//
// T4 真做范围 (控制流 + 块出口模板全形态):
//
//   BRANCH 6 (B-type, opcode 0x63; rs1/rs2 比较, imm = 13-bit signed offset):
//     IR_OP_BEQ   taken if rs1 == rs2  (signed/unsigned 不区分)
//     IR_OP_BNE   taken if rs1 != rs2
//     IR_OP_BLT   taken if (int32_t)rs1 <  (int32_t)rs2
//     IR_OP_BGE   taken if (int32_t)rs1 >= (int32_t)rs2
//     IR_OP_BLTU  taken if rs1 < rs2   (unsigned)
//     IR_OP_BGEU  taken if rs1 >= rs2  (unsigned)
//     双出口块体: BRANCH IR 自 emit cmp + jcc + Label; taken 边内嵌 emit_dispatch_exit
//     (target_pc=BRANCH IR.target_pc=taken_pc) + emit_epilogue; fall-through 边
//     走块末 DISPATCH_EXIT (target_pc=cur_pc=fall_through_pc=pc+4). 一块两份
//     epilogue, host code 块大小 2x 但 asmjit JitRuntime page-granularity 不撞限.
//   JAL 1 (J-type, opcode 0x6F; rd = pc + pc_step, pc = pc + imm; compile-time target):
//     IR_OP_JAL   IR 自身只 emit 写 rd = pc + pc_step (compile-time imm,
//                  装在 ir.target_pc); 块末 DISPATCH_EXIT 装 target_pc = pc + imm
//                  = jump target (translator 算好).
//   JALR 1 (I-type, opcode 0x67; rd = pc + pc_step, pc = (rs1 + imm12) & ~1u; runtime target):
//     IR_OP_JALR  IR 自身 emit 算 runtime target = (rs1 + imm12) & ~1u 存约定
//                  scratch reg r9d + 写 rd = pc + pc_step (compile-time imm,
//                  装在 ir.target_pc); 块末 DISPATCH_EXIT_RUNTIME 从 r9d 读
//                  target 写 cpu->pc. JALR 跟末 RUNTIME 之间无 helper call,
//                  r9d caller-saved 安全 (后续若加 instrumentation 需改 scratch).
//   SYSTEM 6 (I-type, opcode 0x73 funct3=0 imm-encoded; 全硬边界):
//     IR_OP_ECALL     backend emit `call trap_raise_exception(hart, 8+priv, 0)`
//                      (_Noreturn longjmp); cause = CAUSE_ECALL_FROM_U + hart->priv
//                      (runtime 读 priv).
//     IR_OP_EBREAK    backend emit `call trap_raise_exception(hart, 3, 0)`
//                      (_Noreturn longjmp); cause = CAUSE_BREAKPOINT compile-time.
//     IR_OP_MRET      backend emit `call mret_helper(hart, raw_inst)` (helper
//                      内含 PRIV_CHECK_OR_TRAP(PRIV_M); 状态机翻 mstatus
//                      MIE/MPIE/MPP + MDT/SDT Smdbltrp 联动; 写 hart->regs[0]
//                      = xepc[PRIV_M]). 末段 DISPATCH_EXIT_COUNT_ONLY 不写
//                      cpu->pc (helper 已写).
//     IR_OP_SRET      backend emit `call sret_helper(hart, raw_inst)` (同 MRET
//                      但 S-mode 字段段 + sepc). 末段同 MRET.
//     IR_OP_SFENCE_VMA backend emit PRIV_CHECK_OR_TRAP(PRIV_S) inline + `call
//                      sfence_vma_helper(hart, vaddr_val, asid_val, rs1, rs2)`;
//                      末段走标准 DISPATCH_EXIT (helper 不写 cpu->pc).
//     IR_OP_WFI       backend emit TW 检查 inline + `call wfi_wait(hartid,
//                      wfi_should_wake_default, hart)` (block 阻塞 cond_wait
//                      跟 interpreter 同行为) + `call lrsc_clear_self(hart)` +
//                      `add cpu->regs[0], 4` 自推进. 末段 DISPATCH_EXIT_COUNT_ONLY.
//
// BRANCH/JAL/JALR/SYSTEM 6 全是硬边界 op; translator emit 完该 op 后立刻 emit
// 末段 DISPATCH_EXIT / DISPATCH_EXIT_RUNTIME 切块.
//
// aq/rl TODO (AMO + LR/SC memory order 修饰; B2 决议 chat session_004):
//   当前完全忽略, 跟 Spike (riscv-isa-sim insns/amoadd_w.h case 不读 aq/rl) +
//   rvemu (d0iasm cpu.rs `let _aq = ...; // TODO`) 同体例. 项目 host x86_64
//   strong order + 起步阶段不跑 RVWMO weak memory consistency 严格 stress,
//   忽略行为符合用户态单机 emulator 工业惯例.
//   真做时按 QEMU 体例 (target/riscv/insn_trans/trans_rva.c.inc) translator
//   端 emit fence (tcg_gen_mb 风格), IR 不存 — 是局部增量, 不破坏现 IR 设计;
//   weak host (ARM/AArch64) JIT 才有真收益, x86_64 host 永远是 no-op.
//
// 后续 T7 扩 enum:
//   T7 M+RVC:          MUL/DIV/REM 8 + RVC 压缩指令
//
// IR 流形态 (T1+T2+T3):
//   [48 op 之一 前缀 N 条 (0 ≤ N ≤ BLOCK_INST_LIMIT - 1)]
//   [IR_OP_DISPATCH_EXIT 收尾 1 条 (target_pc = 下一块入口 pc = 截断指令 PC 或
//    软边界出块 PC 或 硬边界 op 后的 PC)]
//
// 块前缀空 (N=0, 第一条就是非翻译 op): translator 仍 emit DISPATCH_EXIT 收尾一
// 条, 但 backend.compile_block 检测到 IR 流只有 DISPATCH_EXIT 时返
// JIT_ERR_NOT_IMPLEMENTED (块前缀无真翻译指令, 不值得 install), jit_entry.cc
// 走 set_blacklist 路径; dispatcher 下次 lookup BLACK miss → interpret 兜底真
// 执行那条 RV 指令.
//
// regime 分流 (T3 实状): backend.compile_block 接 regime_t 参数; T3 阶段只
// REGIME_BARE 真实装 emit, REGIME_SV32_S / _U 直接返 JIT_ERR_NOT_IMPLEMENTED
// (jit_entry 走 set_blacklist → BLACK → interpret 兜底). 不是 bug 是预期降级
// (b_01 T2/T4 已就位 jit_cache 状态机). SV32 fast path baked PTE_U / SUM/MXR
// runtime 读 mstatus 形态复杂 + 真撞 fixture 少, 推到 b_03 上 SMC + 真 SV32
// OS guest 时再做更经济. 详 mmu.h:71-83 line 接口分裂明文 + plan T3 plan file.
//
// AUIPC PA != VA trail: V1 BARE only, translator cur_pc = pa = VA; SV32 上线
// (b_03+) 时 AUIPC baked_pc 来源 (用 pa 还是 cpu->pc VA) 单独议.
//
// 命名:
//   - 类型 ir_*_t (ir_op_kind_t / ir_inst_t)
//   - enum 值 IR_OP_* (IR_OP_UNSUPPORTED / IR_OP_DISPATCH_EXIT / 21 RV32I 算术 op)
//   - 跟 decode.h op_kind_t / OP_* 平行, 区分 RV 视角 vs backend 视角
//

#ifndef JIT_IR_H
#define JIT_IR_H

#include <stdint.h>

#include "riscv.h"   // uxlen_t (target_pc 字段; dummy.txt §13 typedef family)


// ----------------------------------------------------------------------------
// ir_op_kind_t —— IR op 分类 (T1-T4 范围: 3 哨兵/出口 + 62 真翻译 op)
//
// IR_OP_UNSUPPORTED         - enum 完整性哨兵 (永不真出现在 IR buffer); backend
//                              default case 写 __builtin_unreachable / abort 兜底
// IR_OP_DISPATCH_EXIT       - 块出口模板 (compile-time imm target); target_pc 填
//                              下一 pc
// IR_OP_DISPATCH_EXIT_RUNTIME - 块出口模板 (runtime reg target; T4 加给 JALR);
//                              backend emit 从约定 scratch reg r9d 读 target
//                              写 cpu->pc, 其他跟 DISPATCH_EXIT 一致
//
// 62 真翻译 op 按 decode.h op_kind_t 分段顺序排 (RV32I 算术 21 → LOAD 5 →
// STORE 3 → CSR 6 → AMO 9 → LR/SC 2 → MISC-MEM 2 → BRANCH 6 → JAL → JALR →
// SYSTEM 6); 每条 op 语义见顶段 doc.
//
// IR_OP_UNSUPPORTED=0 / IR_OP_DISPATCH_EXIT=1 / IR_OP_DISPATCH_EXIT_RUNTIME=2
// sentinel 显式值保留 (其他位置可自由调整 enum 顺序而不影响 sentinel 语义); 真翻译
// op 不指定显式值, switch 按名字访问.
//
// 后续 T7 扩 enum 按 RV op 分组追加 (M 扩展; RVC 压缩); 接口承诺 sentinel 值
// 不动, 真翻译 op 按 decode.h 段追加末段.
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

    /* ---- B-type BRANCH (6; T4 加; 硬边界; 双出口; target_pc 装 taken_pc) ---- */
    IR_OP_BEQ,
    IR_OP_BNE,
    IR_OP_BLT,
    IR_OP_BGE,
    IR_OP_BLTU,
    IR_OP_BGEU,

    /* ---- J-type JAL (1; T4 加; 硬边界; compile-time target;
            target_pc 装 rd 写入值 = pc + pc_step) ---- */
    IR_OP_JAL,

    /* ---- I-type JALR (1; T4 加; 硬边界; runtime target;
            target_pc 装 rd 写入值; 末 IR 用 DISPATCH_EXIT_RUNTIME) ---- */
    IR_OP_JALR,

    /* ---- I-type SYSTEM (6; T4 加; 全硬边界) ---- */
    IR_OP_ECALL,
    IR_OP_EBREAK,
    IR_OP_MRET,
    IR_OP_SRET,
    IR_OP_SFENCE_VMA,
    IR_OP_WFI,
} ir_op_kind_t;


// ----------------------------------------------------------------------------
// ir_inst_t —— 单条 IR 指令 (POD; 三地址码 + 块出口 target_pc 联合)
//
// T3 决议 (B1=b1 chat session_004): 27 个新 op_kind 继承 T1+T2 拆细体例
// (一 RV op 一 IR op_kind), ir_inst_t 字段表保持 6 个不动 + 1 个 raw_inst:
//   - funct3 / funct5 不加 (跟 op_kind 重复编码; 例 LB/LH/LW 已拆 op_kind,
//     funct3 信息已在 op_kind 内; AMO 9 op 各占独立 op_kind, funct5 同冗余)
//   - csr_addr 复用 imm 字段 (跟 decoded_inst_t 同做法, decode.h:158 line 字段
//     约定; CSR 入口判 priv / RO 写 / illegal csr 都由 csr_op helper 处理,
//     backend 只 emit `call csr_op(hart, csr_addr=imm, ...)` 静态绑定)
//   - aq/rl 不加 (B2=a 决议; ir.h 顶段 doc aq/rl TODO 段)
//   - raw_inst 加 (T3 chat 内拍): CSR 6 op 调 csr_op 时需要 raw_inst 作 illegal
//     inst trap mtval 来源 (RV spec §3.1.16 强制); 不是 op_kind 冗余, 是真信息
//     缺失. 其他 op (LOAD/STORE/AMO/LR/SC/FENCE 等) 不读 raw_inst 字段
//
// 字段使用约定 (按 kind 不同):
//   kind = IR_OP_DISPATCH_EXIT:
//     target_pc - 下一块入口 PC (硬边界算 / 软边界出块 PC / 截断指令 PC; JAL 时
//                  装 jump target = pc + imm; BRANCH 时装 fall_through_pc = pc + 4;
//                  MRET/SRET/WFI 装 pc + 4 但 backend 走 COUNT_ONLY 不读)
//     rd/rs1/rs2/imm - 不用
//   kind = IR_OP_DISPATCH_EXIT_RUNTIME (T4; JALR 用):
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
//                     IR_OP_BGEU} (T4):
//     rs1 / rs2 - 寄存器号 (条件比较两操作数)
//     target_pc - taken_pc (translator 算 cur_pc + (uint32_t)d.imm; backend
//                  emit cmp + jcc + taken_label: emit_dispatch_exit(taken_pc)
//                  + emit_epilogue; fall-through 走块末 DISPATCH_EXIT)
//     rd / imm / raw_inst - 不用 (imm 装也可调试 trace; backend 不读)
//   kind = IR_OP_JAL (T4):
//     rd        - 链接寄存器号 (rd = pc + pc_step)
//     target_pc - rd 写入值 = pc + pc_step (compile-time imm, translator 算)
//     rs1 / rs2 / imm / raw_inst - 不用 (jump target = pc + imm 装在末
//                                          DISPATCH_EXIT.target_pc, 不在 IR 自身)
//   kind = IR_OP_JALR (T4):
//     rd        - 链接寄存器号
//     rs1       - 基址寄存器号
//     imm       - 12-bit signed imm12 (offset; backend 算 target = (rs1 + imm) & ~1u)
//     target_pc - rd 写入值 = pc + pc_step (compile-time imm)
//     rs2 / raw_inst - 不用 (末 IR 是 DISPATCH_EXIT_RUNTIME 从 r9d 读 target)
//   kind ∈ SYSTEM 6 {IR_OP_ECALL, IR_OP_EBREAK, IR_OP_MRET, IR_OP_SRET,
//                     IR_OP_SFENCE_VMA, IR_OP_WFI} (T4):
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
    u32_t        raw_inst;    /* CSR 6 op illegal trap mtval; 其他 op 不读 */
} ir_inst_t;


#endif //JIT_IR_H
