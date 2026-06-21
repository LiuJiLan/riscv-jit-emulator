//
// decode 模块对外接口。
//
// 职责: 把 RV32 (含 RVC) 指令解析成 (op_kind, rd, rs1, rs2, imm, raw_inst, pc_step) 的
//       纯数据结构, 给 interpreter / translator 共用。decode 是纯函数, 不读 / 不写 cpu_t。
//
// op_kind_t 范围 (按 RV 指令类型分组; RVC 不单立 op_kind, 复用 RV32I 同源 op):
//   - 算术 / 逻辑 / 立即数 (RV32I): 21 (LUI/AUIPC + 9 OP-IMM + 10 OP)
//   - 整数乘除 (RV32M): 8 (MUL/MULH/MULHSU/MULHU + DIV/DIVU/REM/REMU;
//                                    物理位置 enum 内嵌入 R-type OP 段 OP_AND 之后,
//                                    同 opcode 0x33 聚合, 见下方 M ext 段 doc)
//   - 控制流: 8 (6 branch + JAL + JALR)
//   - CSR: 6 (CSRRW/RS/RC + I 变体 RWI/RSI/RCI)
//   - SYSTEM: 6 (ECALL / EBREAK / MRET / SRET / SFENCE_VMA / WFI)
//   - LOAD / STORE: 8 (5 LB/LH/LW/LBU/LHU + 3 SB/SH/SW)
//   - MISC-MEM (RVI base + Zifencei): 2 (FENCE / FENCE.I; opcode 0x0F)
//   - A 扩展 Zaamo: 9 (AMOADD/SWAP/XOR/OR/AND/MIN/MAX/MINU/MAXU.W; opcode 0x2F)
//   - A 扩展 Zalrsc: 2 (LR.W / SC.W; opcode 0x2F + funct5=0x02/0x03)
//   共 68 个真 op + 1 个 OP_UNSUPPORTED 兜底 = 69 个 op_kind。
//   不在此范围的 opcode (F/D / 等) decode 全部归 OP_UNSUPPORTED。
//
// op_kind_t 增长策略: 真要支持新 op 时再加 enum case + interpreter switch case + (未来)
//   translator emit case; -Wswitch-enum + -Werror 强制 switch 一致性, 增量加新 op_kind
//   时编译器逼着补 case, 不会漏。enum 顺序按 RV 指令 type/opcode 分段; 段内可追加 (例如
//   同段 R-type opcode 0x33 内 M ext 8 条加在 OR/AND 之后); 段间一般不重排, 真要归位
//   历史散落项时单独 commit (git blame 噪声大但单 commit 自隔离, enum 数字重排对
//   interpreter/decode 用名字访问无功能影响)。
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
//   raw_inst - 原始指令; 留给 debug / illegal-instruction trap 的 mtval 填充 (RV 规范规定
//                illegal trap 的 mtval = 触发指令本身; 32-bit 指令 = 完整 32 位; 16-bit
//                RVC = 低 16 位 + 高 16 位 0)
//   pc_step  - PC 步进量 (PC_STEP_RV / PC_STEP_RVC / PC_STEP_NONE), 详见下方 pc_step doc
//
// 注: x0 寄存器号约定见 dummy.txt §2 (decode 不特判, rd/rs1/rs2 字段都可能是 0,
//     interpreter 路径自己用 READ_REG / WRITE_REG 宏分流)。
//

#ifndef CORE_DECODE_H
#define CORE_DECODE_H

#include <stdint.h>

#include "riscv.h"  // u32_t (typedef family; dummy.txt §13)

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

    // ---- RV32M Integer Multiply/Divide (8), opcode 0x33 funct7=0x01 ----
    // RV ISA Manual Vol I "M" extension. 物理位置嵌入 R-type OP 段 (同 opcode 0x33);
    // funct3 8 子分类 (跟 RV32I OP funct7=0x00/0x20 路径互斥):
    //   funct3=0 MUL    rd = (rs1 × rs2)[31:0]            (低 32 bit; signed/unsigned 等价)
    //   funct3=1 MULH   rd = (sx(rs1) × sx(rs2))[63:32]   (signed × signed 高 32 bit)
    //   funct3=2 MULHSU rd = (sx(rs1) × zx(rs2))[63:32]   (signed × unsigned 高 32 bit; 不对称)
    //   funct3=3 MULHU  rd = (zx(rs1) × zx(rs2))[63:32]   (unsigned × unsigned 高 32 bit)
    //   funct3=4 DIV    rd = rs1 / rs2 (signed)
    //   funct3=5 DIVU   rd = rs1 / rs2 (unsigned)
    //   funct3=6 REM    rd = rs1 % rs2 (signed)
    //   funct3=7 REMU   rd = rs1 % rs2 (unsigned)
    //
    // spec edge cases (RV Unprivileged §7.2, decode 不感知, interpreter 自处理; 全部不 trap):
    //   - DIV  by-0           : rd = -1 (all-ones)
    //   - DIVU by-0           : rd = 2^32-1 (UINT32_MAX, all-ones)
    //   - REM  by-0           : rd = rs1 (dividend 原样)
    //   - REMU by-0           : rd = rs1
    //   - DIV  INT_MIN ÷ -1   : rd = INT_MIN (overflow 不 trap, 返 dividend; C signed
    //                                          overflow 是 UB, 必须 if 短路)
    //   - REM  INT_MIN ÷ -1   : rd = 0       (overflow 不 trap, 返 0; 同上短路)
    //
    // 字段约定 (跟 R-type OP 一致):
    //   d.rd / d.rs1 / d.rs2: 寄存器号 (decode 顶部统一提取)
    //   d.imm:                 不用 (R-type 无 imm)
    //   d.pc_step:             PC_STEP_RV (普通 +4; 默认值)
    //
    // 非块边界 (is_block_boundary_inst → 0; 跟算术 / 逻辑 / R-type OP 同性质)。
    //
    // funct7 严格分支起因: 早期 decode case 0x33 funct3 switch 不查 funct7, funct7=1
    //   的 M ext 被当 ADD/SLL/SLT/.../AND 跑出乱数据 (silent miscompile)。现 funct7
    //   严格分支 (0x00/0x20/0x01 显式枚举, 其他 OP_UNSUPPORTED)。
    OP_MUL,
    OP_MULH,
    OP_MULHSU,
    OP_MULHU,
    OP_DIV,
    OP_DIVU,
    OP_REM,
    OP_REMU,

    // ---- B-type BRANCH (6), opcode 0x63 ----
    // 6 个变体 funct3 编码 (RV spec): BEQ=000, BNE=001, BLT=100, BGE=101, BLTU=110, BGEU=111
    // (funct3=010 / 011 reserved, decode 归 OP_UNSUPPORTED)。
    // 立即数 13 位有符号 (multiple of 2, imm[0]=0 编码强制); 4 段位拼接见 decode.c。
    OP_BEQ,
    OP_BNE,
    OP_BLT,
    OP_BGE,
    OP_BLTU,
    OP_BGEU,

    // ---- J-type JAL (1), opcode 0x6F ----
    // 立即数 21 位有符号 (multiple of 2, imm[0]=0 编码强制); 4 段位拼接见 decode.c。
    // ±1 MiB 跳转范围。
    OP_JAL,

    // ---- I-type JALR (1), opcode 0x67, funct3=000 ----
    // 立即数复用 I-type 12 位有符号 (与 ADDI / SLTI 等同型)。
    // 目标 = (rs1 + imm) & ~1u (RV spec 强制 mask LSB)。
    // funct3 != 0 reserved by spec, decode 归 OP_UNSUPPORTED。
    OP_JALR,

    // ---- I-type SYSTEM CSR (6), opcode 0x73 ----
    // 6 个 csr 指令变体, funct3 编码 (RV spec):
    //   001 = CSRRW   atomic Read-Write
    //   010 = CSRRS   atomic Read-Set    (按位 |)
    //   011 = CSRRC   atomic Read-Clear  (按位 & ~)
    //   101 = CSRRWI  atomic Read-Write  Immediate (rs1 字段当 5-bit zimm)
    //   110 = CSRRSI  atomic Read-Set    Immediate
    //   111 = CSRRCI  atomic Read-Clear  Immediate
    // funct3 = 000 在 SYSTEM opcode 内是 ECALL / EBREAK / MRET / SRET / WFI 等
    // (各自由 imm[11:0] 进一步区分; 见下方 SYSTEM 段)。
    //
    // 字段约定 (与 dummy.txt §2 x0 编码对齐):
    //   d.imm = csr 12-bit address (inst[31:20]); 无符号, 高 20 位 0
    //   d.rs1 = inst[19:15]:
    //             RW/RS/RC 变体:  rs1 寄存器号 (interpreter READ_REG(d.rs1) 取源值)
    //             RWI/RSI/RCI 变体: 5-bit zimm (interpreter 直接用 d.rs1 数值, 不查 regs)
    //             两种语义在 op_kind 区分, 字段位置共用 (Spike / QEMU 同做法)
    //   d.rd  = inst[11:7]: 写目标寄存器号; rd=x0 是合法 (csr 副作用 / 写仍发生, 只是
    //                       读出来的旧值丢; 各小 r/w helper 不感知 rd)
    //   d.pc_step = PC_STEP_RV (csr 不是 control flow, fetch loop 正常 +4; 但 csr 是
    //                            硬边界, fetch loop 末尾 is_block_boundary_inst 检查
    //                            后退出, dispatcher 重派发)
    //
    // is_block_boundary_inst 这 6 个 case 都 return 1 (硬边界, plan §1.6 简化策略:
    // "所有 csr 写视为硬边界, 过度刷新允许")。
    OP_CSRRW,
    OP_CSRRS,
    OP_CSRRC,
    OP_CSRRWI,
    OP_CSRRSI,
    OP_CSRRCI,

    // ---- I-type SYSTEM (ECALL / EBREAK / MRET, 3 op) ----
    // opcode 0x73, funct3=000, 由 imm[11:0] (= inst[31:20]) 进一步区分:
    //   imm[11:0] = 0x000 → ECALL    (Environment Call from current priv)
    //   imm[11:0] = 0x001 → EBREAK   (Breakpoint)
    //   imm[11:0] = 0x302 → MRET     (Machine-mode Return; 从 trap handler 回归)
    //   imm[11:0] = 0x102 → SRET     (Supervisor-mode Return; 见下方 SRET 段)
    //   funct7=0x09       → SFENCE.VMA (见下方 SFENCE.VMA 段)
    //   imm[11:0] = 0x105 → WFI         (Wait For Interrupt; 见下方 WFI 段)
    //   其他保留 imm/funct7 组合仍归 OP_UNSUPPORTED, 真做时再加
    //   (注: FENCE / FENCE.I 不在 SYSTEM opcode 下, 它们在 MISC-MEM opcode 0x0F, 见下方
    //   MISC-MEM 段)
    //
    // 字段约定:
    //   d.imm = 0 (ECALL/EBREAK 的 imm[11:0] 已用于区分 op_kind, interpreter case 不读此字段;
    //               MRET 也不读)
    //   d.rd / d.rs1 = 0 (RV spec 要求, decode 顶部统一提取的值理论上应是 0; 实际 fixture 编
    //                      ecall=0x73 时这些位都 0; 不强求, interpreter case 不读)
    //   d.pc_step = PC_STEP_RV (4 字节固定, fetch loop 推进 +4 — 但 ECALL/EBREAK 走 trap 路径
    //                            不到 fetch loop 末尾; MRET 写 pc=xepc 后 case 不动 +pc_step,
    //                            走 boundary 退出 fetch loop)
    //   注: MRET 像 control flow op (改 pc), 但因为它通过 csr 路径 (xepc) 而不是立即数算 pc,
    //       这里 pc_step 设 PC_STEP_RV 然后 case 内手动覆盖 hart->regs[0] = xepc 也行;
    //       或设 PC_STEP_NONE 跟 jal/branch 同模式。当前选 PC_STEP_NONE (case 写 pc, 不
    //       让 fetch loop 末尾 += 4 误推 4 字节)。下面 decode.c 也跟此约定。
    //
    // 是块边界 (硬边界): ECALL/EBREAK 触发 trap (cause 11/3) 控制流跳 xtvec; MRET 跳 xepc.
    // 都改 pc, 必须结束当前 block (is_block_boundary_inst 3 case → return 1)。
    OP_ECALL,
    OP_EBREAK,
    OP_MRET,

    // ---- I-type SYSTEM SRET ----
    // opcode 0x73, funct3=0, imm[11:0]=0x102 (= inst[31:20]); rd=0, rs1=0 (RV spec)。
    // 跟 OP_MRET 同形态, 不同点:
    //   - MRET: priv=MPP; MIE=MPIE; MPIE=1; MPP=U; pc=mepc; mstatus.MDT=0
    //           (+ 若新 priv ∈ {U/VS/VU}: sstatus.SDT=0; Smdbltrp/Ssdbltrp 联动)
    //   - SRET: priv=SPP; SIE=SPIE; SPIE=1; SPP=U; pc=sepc; sstatus.SDT=0
    //   操作的是 _mstatus 的不同位段 (M-mode 段 vs S-mode 段); pc 来源不同
    //   (mepc=trap.xepc[PRIV_M] vs sepc=trap.xepc[PRIV_S])
    //
    // 权限要求: SRET 仅在 hart->priv >= S 时合法 (M/S 模式可用; U 模式触发 cause 2 illegal)。
    // 加 mstatus.TSR=1 时 S-mode SRET 也 trap to M (cause 2; 当前不实现 TSR 检查,
    // 真做 OS 隔离时在 interpreter case 入口加)。
    //
    // is_block_boundary_inst → 1 (改 pc, 必须块尾)。
    OP_SRET,

    // ---- I-type SYSTEM SFENCE.VMA ----
    //
    // 编码 (RV Privileged Spec Vol II §10.6.1):
    //   opcode  = 0x73 (SYSTEM)                       inst[6:0]
    //   rd      = 0    (RV spec 要求, 否则 reserved)  inst[11:7]
    //   funct3  = 0                                    inst[14:12]
    //   rs1     = vaddr 寄存器号                       inst[19:15]
    //              rs1 == x0 → 操作覆盖所有 vaddr (简化方案 4.a 全清/单 ASID 全清)
    //   rs2     = asid 寄存器号                        inst[24:20]
    //              rs2 == x0 → 操作覆盖所有 asid (简化方案 4.a)
    //   funct7  = 0x09 (= 0b0001001)                   inst[31:25]
    //
    // 跟 ECALL/EBREAK/MRET 用 imm[11:0] 区分不同, sfence.vma 由 funct7=0x09 区分; 因为
    // imm[4:0] = rs2 字段 (5-bit 寄存器号) 是变量, imm[11:0] 不固定值 (rs2 ∈ 0..31, 32 个变种)。
    //
    // 字段约定:
    //   d.rs1 = vaddr 寄存器号 (interpreter case 内 READ_REG(d.rs1) 拿 vaddr 值)
    //   d.rs2 = asid 寄存器号  (interpreter case 内 READ_REG(d.rs2) 拿 asid 值, 截断到 ASID_MASK
    //                            位由 sfence helper 内部做)
    //   d.rd  = 0 (RV spec; decode 顶部统一提取 = 0)
    //   d.imm = funct7|rs2 = 0x120..0x13F (decode 顶部 imm = inst[31:20] 提取所得; sfence case
    //            不读此字段, 仅供 raw_inst trap 路径用)
    //   d.pc_step = PC_STEP_RV (sfence 不是 control flow, +4 推进; 但是硬边界, fetch loop 末
    //                            is_block_boundary_inst → 1 → goto out, dispatcher 重派发)
    //
    // 是块边界 (硬边界): sfence.vma 改 TLB 状态, 块内后续指令译码假设 (TLB / current_tlb 路径)
    // 失效; dispatcher 重派发时按新 satp/TLB 状态走 block 1 (current_tlb 重新选叶 TLB)。
    //
    // 未来 mstatus.TVM 检查: TVM=1 + S-mode sfence.vma → trap cause 2; 真做 OS 隔离时在
    // interpreter case 入口 (或 sfence helper 入口) 加。
    OP_SFENCE_VMA,

    // ---- I-type SYSTEM WFI ----
    // opcode 0x73, funct3=0, imm[11:0]=0x105; rd=0, rs1=0 (RV spec)。
    //
    // RV Privileged Spec §3.3.2: WFI 是一条 hint — 实现可挂起 hart 等中断, 也可直接当 NOP。
    // 唤醒条件 (跟 mstatus.MIE 无关): (mie & mip) != 0 任何 enabled-by-mie 的中断 pending。
    // 唤醒后:
    //   - mstatus.MIE=1 → 走 trap 入向量 (xtvec)
    //   - mstatus.MIE=0 → 继续 PC+4 (典型用法: 关中断 WFI 等事件, 软件醒后 poll mip)
    //
    // mstatus.TW (Timeout Wait): TW=1 + priv<M 时 WFI 必须 trap illegal (RV spec §3.1.6.5);
    // interpreter case 入口检查; 当前实现 TW 触发立即 illegal (timeout=0 合法)。
    //
    // 字段约定:
    //   d.rd = d.rs1 = 0 (RV spec; decode 顶部统一提取 = 0)
    //   d.imm = 0x105 (decode 顶部提取所得; interpreter case 不读)
    //   d.pc_step = PC_STEP_NONE (case 末尾自写 pc += 4 也行, 但 NONE 是 "case 必自写 pc"
    //                              的明确表态; 跟 MRET/SRET 同体例 — 它们也走 case 自写 pc 路径)
    //   注: NONE 跟实际行为 (醒来后 PC+4 或 trap 跳 xtvec) 兼容: trap 路径不到 fetch loop
    //   末段; PC+4 路径 case 内 hart->regs[0] += 4 显式做
    //
    // 是块边界 (硬边界): WFI 必须块尾 — 唤醒可能触发 trap (pc 跳 xtvec) 或继续 (pc+=4),
    // 块内后续指令无法静态译码 (路径不定); dispatcher 重派发时按新 pc 走 block 1。
    OP_WFI,

    // ---- I-type LOAD (5), opcode 0x03 ----
    // funct3 编码 (RV spec):
    //   000 = LB    (Load Byte,           sign-ext 8-bit)
    //   001 = LH    (Load Halfword,       sign-ext 16-bit)
    //   010 = LW    (Load Word,           32-bit)
    //   100 = LBU   (Load Byte Unsigned,  zero-ext 8-bit)
    //   101 = LHU   (Load Halfword Unsigned, zero-ext 16-bit)
    //   011 / 110 / 111 = reserved (RV64 LD/LWU 等), decode 归 OP_UNSUPPORTED
    //
    // 字段约定 (与 ADDI 等 I-type 一致):
    //   d.rd       = inst[11:7]: 写目标寄存器 (rd=x0 → dummy.txt §2 dead store)
    //   d.rs1      = inst[19:15]: 基地址寄存器 (ea = READ_REG(d.rs1) + d.imm)
    //   d.imm      = sign-ext inst[31:20] (12 位有符号, 复用 ADDI 路径)
    //   d.rs2      = inst[24:20] = imm 低 5 位 garbage (decode 顶部统一提取, 不特判)
    //   d.pc_step  = PC_STEP_RV (普通 +4)
    //
    // load 不是块边界 (is_block_boundary_inst → 0); 内存访问不改控制流, 块内可连续多条。
    // 实际访问 case 内调 lsu_load_helper (isa/lsu.h, inline 顶层): BARE 内联 RAM/MMIO
    // 分流 / SV32 TLB hit 直接 *hva / miss 调 mmu_walker_helper_load。sext/zext 由
    // case 各自做 (LB: int8_t cast → int32_t; LH: int16_t cast; LBU/LHU: 直接 zero-ext;
    // LW: 直传)。不对称真机理见 src/isa/lsu.h 顶段 + dummy.txt §1 末段。
    OP_LB,
    OP_LH,
    OP_LW,
    OP_LBU,
    OP_LHU,

    // ---- S-type STORE (3), opcode 0x23 ----
    // funct3 编码 (RV spec):
    //   000 = SB    (Store Byte,     8-bit)
    //   001 = SH    (Store Halfword, 16-bit)
    //   010 = SW    (Store Word,     32-bit)
    //   011 / 100..111 = reserved (RV64 SD 等), decode 归 OP_UNSUPPORTED
    //
    // S-type 立即数 12 位有符号, 由 inst 中两段拼接 (RV spec Vol I, Memory Access):
    //   imm[11:5] = inst[31:25]    (高 7 位)
    //   imm[4:0]  = inst[11:7]     (低 5 位)
    // 不是连续段位 — 这是 RV 的设计取舍 (让 rs1/rs2 字段位置在所有指令类型保持一致, decoder
    // 可以先无脑提取 rs1/rs2/funct3 再按 opcode 算 imm)。
    //
    // 字段约定:
    //   d.rs1 = inst[19:15]: 基地址寄存器 (ea = READ_REG(d.rs1) + d.imm)
    //   d.rs2 = inst[24:20]: 源寄存器 (写入内存的值 = READ_REG(d.rs2))
    //   d.imm = sign-ext { inst[31:25], inst[11:7] } (12 位)
    //   d.rd  = inst[11:7] = imm 低 5 位 garbage (decode 顶部统一提取, 不特判)
    //   d.pc_step = PC_STEP_RV
    //
    // store 不是块边界 (is_block_boundary_inst → 0)。
    // 实际访问 case 内调 lsu_store_helper (isa/lsu.h, inline 顶层): BARE 内联 RAM/MMIO
    // 分流 / SV32 TLB hit 调 store_helper(hva,..) / miss 调 mmu_walker_helper_store。
    // reservation 清除 + SMC 检测 集中在 lsu.c store_helper (HVA-based, RAM 写 + 副作用
    // 入口); MMIO 不经 store_helper, 直接 mmio_write_helper。
    // 不对称真机理见 src/isa/lsu.h 顶段 + dummy.txt §1 末段。
    OP_SB,
    OP_SH,
    OP_SW,

    // ---- MISC-MEM (2), opcode 0x0F ----
    // RV Unprivileged Spec: FENCE (RVI base "Memory Ordering"), FENCE.I (Zifencei).
    // 同 opcode 0x0F 由 funct3 区分:
    //   funct3 = 000 → FENCE   (memory ordering hint; fm/pred/succ 字段不解, 全 NOP)
    //   funct3 = 001 → FENCE.I (i-cache flush; 副作用 lrsc_clear_self)
    //   其他 funct3 → OP_UNSUPPORTED (reserved)
    //
    // 跟 SYSTEM 段 SFENCE.VMA 拆开的原因 (Q19): opcode 不同 + 手册不同卷 + 语义不同族.
    // 详见 isa/fence.h 顶段.
    //
    // 字段约定:
    //   d.rd / d.rs1 / d.rs2 / d.imm  不用 (interpreter case 不读; fence 字段全弃)
    //   d.pc_step                       = PC_STEP_RV (普通 +4)
    //
    // 是块边界:
    //   FENCE   - 否 (memory ordering 不动指令流, 块内可继续译; is_block_boundary_inst → 0)
    //   FENCE.I - 是 (i-cache flush + reservation 清, 块内后续指令可能跟新视图不一致;
    //              → 1, 同 SFENCE.VMA 体例 — 都是"动状态后块边界")
    OP_FENCE,
    OP_FENCE_I,

    // ---- A 扩展 Zaamo (9), opcode 0x2F ----
    // RV Unprivileged Spec Vol I "A" extension Zaamo 子扩展.
    // 同 opcode 0x2F + funct3=010 (.W; RV32 only) 由 funct5 区分:
    //   funct5 = 0x00 → AMOADD.W  (atomic_fetch_add)
    //   funct5 = 0x01 → AMOSWAP.W (atomic_exchange)
    //   funct5 = 0x04 → AMOXOR.W  (atomic_fetch_xor)
    //   funct5 = 0x08 → AMOOR.W   (atomic_fetch_or)
    //   funct5 = 0x0C → AMOAND.W  (atomic_fetch_and)
    //   funct5 = 0x10 → AMOMIN.W  (signed min, CAS loop 手写)
    //   funct5 = 0x14 → AMOMAX.W  (signed max, CAS loop 手写)
    //   funct5 = 0x18 → AMOMINU.W (unsigned min, CAS loop 手写)
    //   funct5 = 0x1C → AMOMAXU.W (unsigned max, CAS loop 手写)
    //   funct5 = 0x02 LR.W / 0x03 SC.W 是 Zalrsc 子扩展, OP_LR_W / OP_SC_W (见下段).
    //   其他 funct5 → OP_UNSUPPORTED (reserved by spec)
    //
    // 字段约定:
    //   d.rd       = inst[11:7]  写目标寄存器 (sext32(old) 写入; rd=x0 dead store)
    //   d.rs1      = inst[19:15] gva 基址寄存器 (AMO 无 imm offset; ea = READ_REG(d.rs1))
    //   d.rs2      = inst[24:20] AMO 操作 "源" 值寄存器 (ADD/AND/OR/XOR 是位级源, SWAP
    //                  覆盖, MIN/MAX 比较值)
    //   d.imm      = 0           AMO 无 imm (decode 不提取; 字段位置被 funct5/funct3/rs2
    //                  占用; raw_inst 字段供 mtval 用)
    //   d.pc_step  = PC_STEP_RV (普通 +4; AMO 不改控制流)
    //
    // bits[26:25] aq/rl 字段 Q11 拍全 seq_cst 不精确化, decode 不读 (op_kind 选择不依赖
    //   aq/rl, 9 op 直接由 funct5 决定).
    //
    // 是块边界: 否 (AMO 不改 pc, 不改全局 TLB 状态; 跟普通 store 同; is_block_boundary_inst → 0).
    //   misalign / MMIO 拒 / walker fault 经 trap_raise_exception _Noreturn longjmp 跳回
    //   dispatcher, 不走 boundary 路径.
    //
    // 实际访问 case 内调 amo_xxx_helper (isa/amo.h, inline 顶层): BARE 内联 RAM/MMIO 分流
    //   (MMIO 拒 cause 7) / SV32 TLB hit 调 amo_xxx_apply / miss 调 mmu_walker_helper_amo_xxx.
    //   3-layer 模型见 amo.h 顶段; 跟 store 严格对偶但**不复用 store_helper** (RMW vs memcpy).
    OP_AMO_ADD_W,
    OP_AMO_SWAP_W,
    OP_AMO_XOR_W,
    OP_AMO_OR_W,
    OP_AMO_AND_W,
    OP_AMO_MIN_W,
    OP_AMO_MAX_W,
    OP_AMO_MINU_W,
    OP_AMO_MAXU_W,

    // ---- A 扩展 Zalrsc (LR.W / SC.W), opcode 0x2F + funct3=010 + funct5=0x02/0x03 ----
    // RV Unprivileged Spec Vol I "A" extension Zalrsc 子扩展.
    //
    // 字段约定:
    //   d.rd       = inst[11:7]
    //                  LR.W: 写 zero-ext(*pa) 32-bit 值
    //                  SC.W: 写 0=success / 1=fail
    //   d.rs1      = inst[19:15] gva 基址寄存器 (AMO 同; LR/SC 也无 imm offset)
    //   d.rs2      = inst[24:20]
    //                  LR.W: spec 强制 0, 非 0 implementation-defined (decode 不读)
    //                  SC.W: 写源值寄存器 (atomic_store *pa = READ_REG(d.rs2))
    //   d.imm      = 0
    //   d.pc_step  = PC_STEP_RV
    //
    // bits[26:25] aq/rl: 跟 AMO 一致, decode 不读 (全 seq_cst, Q11).
    //
    // 是块边界: 否 (LR/SC 不改 pc, 跟 AMO/store 同). misalign (LR cause 4 / SC cause 6) /
    //   MMIO 拒 (cause 5/7) / walker fault 经 _Noreturn longjmp 不走 boundary 路径.
    //
    // 实际访问 case 内调 lrsc_lr_helper / lrsc_sc_helper (interpreter case 走类似 lsu/amo
    //   的 BARE/SV32 分流 + IS_GPA_RAM check, 最终调 lrsc_lr_w / lrsc_sc_w).
    OP_LR_W,
    OP_SC_W,

    // ---- 兜底 ----
    // 不在当前范围的 opcode (真非法 opcode / 真非法 funct3 / funct5 子段) 全部归这里。
    // interpreter 走 trap_raise_exception(cause=2 Illegal Instruction,
    // mtval=decoded.raw_inst) _Noreturn longjmp 跳回 dispatcher (RV 规范规定 illegal trap
    // 的 mtval = 触发指令本身)。
    OP_UNSUPPORTED,
} op_kind_t;

// pc_step: PC 步进量 (由 decode 一次决定, fetch loop 末尾统一 hart->regs[0] += d.pc_step)。
//
// 当前归类:
//   PC_STEP_RV   : 普通 RV 指令 + 32-bit branch (B-type) + 32-bit jal/jalr
//   PC_STEP_RVC  : 16-bit C 扩展 (含 C.BEQZ/C.BNEZ branch + C.J/C.JAL/C.JR/C.JALR jump)
//   PC_STEP_NONE : "总跳" 且不顺序推进的 op (mret / sret / ecall / ebreak / c.ebreak),
//                  case 自描述 pc 或走 trap 路径, fetch loop += 0 NOP
//
// branch / jal / jalr 设实际指令长度 (RV=4 / RVC=2): taken case 自写 pc + goto out 跳过
// fetch loop 末段 (避免 += d.pc_step 破坏 case 写好的 target); branch not-taken 让末段
// += d.pc_step 自动顺序推进; jal/jalr 也用 d.pc_step 算 ra (= pc + d.pc_step), 兼容
// 32-bit 与 RVC 的 ra 长度差异。
//
// 设计哲学 (运行时可能不确定时, 优先保证可解码的确定):
//   - PC_STEP_NONE 是 "case 必自写 pc" 的兜底约定 (case 100% 控制 pc, fetch loop 不动)
//   - 凡是 "运行时跳/不跳" 在 decode 阶段不能预知的 op (branch 是典型: 取决于运行时
//     rs1/rs2 比较结果), pc_step 优先保证 "可解码可确定的事实" = 实际指令长度;
//     把 "运行时不确定的部分" (跳的话 target 在哪) 留给 case 处理 (taken case 自写 pc +
//     goto out, not-taken 让 fetch loop += d.pc_step 兜底)
//   - 哪怕"总跳"的 op (jal / jalr), 只要需要算指令长度 (ra = pc + 长度), 也用实际长度
//     而不是 hardcoded 4 — 兼容 RVC C.JAL / C.JALR 的 ra = pc + 2
//
// 注: 块边界 (is_block_boundary_inst) 与 pc_step 独立 — branch / sfence.vma 都
// pc_step != PC_STEP_NONE 但 is_boundary = true。
#define PC_STEP_RV    4u
#define PC_STEP_RVC   2u
#define PC_STEP_NONE  0u

typedef struct {
    op_kind_t kind;
    uint32_t  rd;
    uint32_t  rs1;
    uint32_t  rs2;
    int32_t   imm;
    u32_t     raw_inst;     // 原始指令; debug / illegal trap mtval 用
                            // (32-bit 指令 = 完整 32 位; 16-bit RVC 时 = 低 16 位 + 高
                            // 16 位 0, 因为 decode_rvc 用 (uint16_t)inst cast 截断;
                            // RV illegal trap mtval 规范也是按指令长度填, 取低 16 位即可)
    uint32_t  pc_step;      // PC_STEP_RV / PC_STEP_RVC / PC_STEP_NONE
} decoded_inst_t;

// 纯函数: 不读 / 不写 cpu_t, 不依赖 mmu / tlb / ram。
// 对于不识别的 opcode, kind = OP_UNSUPPORTED; 其它字段 (rd/rs1/rs2/imm) 仍按通用编码位置
// 填入, 但调用方不应使用 (除 raw_inst 用作 trap mtval)。
decoded_inst_t decode(u32_t inst);

// ----------------------------------------------------------------------------
// is_block_boundary_inst —— 块边界判定 (硬边界), 共享给 interpreter + 未来 translator
//
// 硬边界 = 必须结束当前 block 的指令 (改变控制流 / 改变全局状态导致后续译码假设失效)。
// 共享 inline 函数保证 interpreter 与 translator 对"什么算硬边界"判断 100% 一致 —
// -Wswitch-enum + -Werror 强制下方 switch 覆盖所有 op_kind_t, 加新 op_kind 时编译器
// 逼着补 case (boundary or 不是), 不会漏。
//
// 当前归类:
//   非 boundary: 算术 / 逻辑 / 立即数 (LUI/AUIPC + 9 OP-IMM + 10 OP) + LOAD/STORE 8 个;
//                AMO 9 个 (Zaamo, 不改 pc 不动 TLB, 跟 store 同性质);
//                FENCE 1 个 (memory ordering 不动指令流);
//                OP_UNSUPPORTED (走 trap_raise + longjmp, 不经"块边界"路径)
//   boundary:    控制流 8 个 (6 branch + JAL + JALR) + CSR 6 变体 + SYSTEM 4 (ECALL/
//                EBREAK/MRET/SRET) + SFENCE.VMA 1 + WFI 1 + FENCE.I 1 (i-cache flush)
//
// CSR 一刀切策略: 6 变体全视为硬边界 (plan §1.6 简化策略 "过度刷新允许")。实际上 CSRRS
// rd, csr, x0 = 纯读不写, 不一定要 boundary; 但保险起见统一视为 boundary, 未来真细化时
// 按 csr_addr (例如 mtvec / mideleg / satp 写) 分流时再优化。
//
// load/store 不改控制流, 只动内存, 块内可连续多条; trap 路径 (misalign / access fault)
// 经 trap_raise_exception _Noreturn longjmp 跳回 dispatcher, 不走 boundary 路径。
//
// 软边界 (块长度上限 BLOCK_INST_LIMIT) 不在这里, 由 interpreter / translator 各自循环
// 维护计数器, 详见 config.h。
//
// 返回 int 0/1 (与 codebase 一致, 不引入 bool / stdbool.h; mmu_translate_pc / dispatcher
// 都是 int 风格)。
//
// 待评估: 在指令实现的差不多后, 重新评估是否把 boundary 信息从 helper 移到 decoded_inst_t
// 字段 (与 pc_step 同风格, 数据驱动)。当前保持 helper — boundary 是项目级判断 (策略),
// pc_step 是 ISA 属性 (数据), 两者粒度不同。
// ----------------------------------------------------------------------------
static inline int is_block_boundary_inst(const decoded_inst_t *d) {
    switch (d->kind) {
        // ---- 算术 / 逻辑 / 立即数 (非 boundary) ----
        case OP_LUI:   case OP_AUIPC:
        case OP_ADDI:  case OP_SLTI:  case OP_SLTIU: case OP_XORI:
        case OP_ORI:   case OP_ANDI:
        case OP_SLLI:  case OP_SRLI:  case OP_SRAI:
        case OP_ADD:   case OP_SUB:   case OP_SLL:   case OP_SLT:  case OP_SLTU:
        case OP_XOR:   case OP_SRL:   case OP_SRA:   case OP_OR:   case OP_AND:
        // ---- RV32M 8 条 (非 boundary; 跟 R-type OP 算术同性质, 同 opcode 0x33) ----
        case OP_MUL:   case OP_MULH:  case OP_MULHSU: case OP_MULHU:
        case OP_DIV:   case OP_DIVU:  case OP_REM:    case OP_REMU:
            return 0;

        // ---- OP_UNSUPPORTED (非 boundary; 走 trap_raise(2) + longjmp 跳回 dispatcher,
        //      不经"块边界"路径) ----
        case OP_UNSUPPORTED:
            return 0;

        // ---- 控制流 8 个 (硬边界) ----
        // 都改 pc, 块必须在此结束。interpreter / translator (未来) 在 fetch loop 末检查本
        // helper 返回值, 是 1 则 break 出 loop, dispatcher 重新走 block 1+2+3 (重新
        // mmu_translate_pc 拿新 pc 的 hva, 进新一块 block)。
        case OP_BEQ:  case OP_BNE:
        case OP_BLT:  case OP_BGE:  case OP_BLTU: case OP_BGEU:
        case OP_JAL:  case OP_JALR:
            return 1;

        // ---- CSR 6 变体 (硬边界) ----
        // 一刀切策略: 所有 csr 视为硬边界 (plan §1.6 "过度刷新允许"); 详见上方 doc。
        case OP_CSRRW:  case OP_CSRRS:  case OP_CSRRC:
        case OP_CSRRWI: case OP_CSRRSI: case OP_CSRRCI:
            return 1;

        // ---- SYSTEM 6 个 (硬边界) ----
        // 子分组各自原因不同, 但都改 pc 或全局 TLB 状态, 必须块尾:
        //   ECALL/EBREAK 触发 trap → pc 跳 xtvec
        //   MRET / SRET → pc 跳 mepc / sepc
        //   SFENCE.VMA  改 TLB 状态; 块内后续指令译码假设 (current_tlb 路径) 失效;
        //                dispatcher 重派发时按新 satp/TLB 状态走 block 1 (current_tlb 重选叶 TLB)
        //   WFI         唤醒可能 trap (MIE=1 + 中断 pending → 跳 xtvec) 或继续 (MIE=0 → PC+4);
        //                块内后续指令静态译码假设不成立 (路径不定)
        case OP_ECALL: case OP_EBREAK: case OP_MRET: case OP_SRET:
        case OP_SFENCE_VMA:
        case OP_WFI:
            return 1;

        // ---- load/store 8 个 (非 boundary) ----
        // 只动内存不改控制流 (pc += 4 普通推进); trap 路径 (misalign cause 4/6, access fault
        // cause 5/7) 经 trap_raise_exception _Noreturn longjmp 跳回 dispatcher, 不走 boundary。
        case OP_LB:  case OP_LH:  case OP_LW:  case OP_LBU: case OP_LHU:
        case OP_SB:  case OP_SH:  case OP_SW:
            return 0;

        // ---- MISC-MEM 2 个 ----
        // FENCE   非 boundary: memory ordering 不动指令流, 同 block 继续译 (x86 TSO 双重
        //                       cover 论证见 isa/fence.h)
        // FENCE.I 是 boundary:  i-cache flush + lrsc_clear_self 副作用; 块内后续指令译码
        //                       视图变 (同 SFENCE.VMA 体例 — "动状态后块边界")
        case OP_FENCE:
            return 0;
        case OP_FENCE_I:
            return 1;

        // ---- A 扩展 Zaamo 9 个 (非 boundary) ----
        // AMO 不改 pc, 不改全局 TLB / 控制状态 (只改单 word memory + LR/SC reservation
        // 表); 跟普通 store 同性质. trap 路径 (misalign cause 6 / MMIO 拒 cause 7 /
        // walker fault cause 15) 经 trap_raise_exception _Noreturn longjmp 跳回 dispatcher,
        // 不走 boundary 路径.
        case OP_AMO_ADD_W:  case OP_AMO_SWAP_W: case OP_AMO_XOR_W:
        case OP_AMO_OR_W:   case OP_AMO_AND_W:
        case OP_AMO_MIN_W:  case OP_AMO_MAX_W:
        case OP_AMO_MINU_W: case OP_AMO_MAXU_W:
            return 0;

        // ---- A 扩展 Zalrsc 2 个 (非 boundary; 跟 AMO/store 同性质) ----
        // LR.W / SC.W 不改 pc, 不改 TLB; 操作 *pa + reservation 字段, 跟 AMO/store 同;
        // trap 路径 (misalign cause 4/6 / MMIO 拒 cause 5/7 / walker fault) 经
        // trap_raise_exception _Noreturn longjmp 跳回 dispatcher, 不走 boundary 路径.
        case OP_LR_W: case OP_SC_W:
            return 0;
    }
    return 0;  // -Wswitch-enum 下不可达 (所有 enum 必须在 switch 列出); 防御写法
}

#endif //CORE_DECODE_H
