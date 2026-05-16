//
// Created by liujilan on 2026/4/28.
// interpreter 模块实现 (decode → switch → 执行; pure case + may-trap helper + boundary)。
//
// 顶部模块文档见 interpreter.h。fast/slow path 协议见 dummy.txt §1; x0 编码见 §2;
// TLB 派发见 §4。
//

#include "interpreter.h"

#include "config.h"     // BLOCK_INST_LIMIT, IALIGN_MASK
#include "cpu.h"
#include "csr.h"        // csr_op + csr_op_t
#include "decode.h"
#include "isa/lsu.h"    // lsu_load_helper / lsu_store_helper (inline 顶层) + store_helper (extern, HVA-based)
#include "isa/sfence.h" // sfence_vma_helper (extern)
#include "riscv.h"      // PRIV_M (MRET 读 hart->trap.xepc[PRIV_M])
#include "tlb.h"
#include "trap.h"       // trap_raise_exception (_Noreturn longjmp)

#include <stdint.h>
#include <string.h>     // memcpy: 4 字节取指, 防 strict-aliasing

void interpret_one_block(cpu_t *hart, tlb_t *current_tlb,
                         uint8_t *hva_pc, uint64_t *count_out) {
    // current_tlb 透传给 lsu_load_helper / lsu_store_helper (BARE: NULL; SV32: 非 NULL);
    // interpreter 自感知 priv (NULL/非NULL 编码 BARE/SV32, lsu 内部分流; a_02 session_004 P3)。

    // dummy.txt §2 局部垃圾桶变量: 写 x0 的 dead store 落点。
    // 编译器 DCE 会把这个 store 干掉, 等于 NO-OP; 保留是为统一 "所有写都通过同一个宏" 风格,
    // 让未来 IR / 后端不感知 x0 特殊性。
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
            SYNC_COUNT();                                                  \
            trap_raise_exception(hart, CAUSE_INST_ADDR_MISALIGNED, /*tval*/_t); \
            goto out;                                                      \
        }                                                                  \
        hart->regs[0] = _t;                                                \
    } while (0)

    // BRANCH_IF: 6 个 branch case 的统一封装 (taken → WRITE_PC_OR_TRAP + goto out;
    //                                       not-taken → 不写 pc, 让 fetch loop 末段 +=
    //                                       d.pc_step 自动推进)。
    //
    // branch 是 control flow 特例:
    //   - taken: case 自写 pc (跳到 imm 目标), 必须 goto out 跳过 fetch loop 末段, 否则
    //            末段 += d.pc_step 会破坏 pc (= taken_target + 2/4); count++ + SYNC_COUNT
    //            在 goto out 之前显式做 (跟 fetch loop 末段一致语义)
    //   - not-taken: 不写 pc (进 case 时 hart->regs[0] = branch PC, 顺序推进的下一条 = pc +
    //                 实际指令长度); fetch loop 末段 hart->regs[0] += d.pc_step 自动推进 →
    //                 pc + 2 (RVC) 或 pc + 4 (32-bit); 然后 boundary check goto out
    //                 (branch is hard boundary)
    //
    // d.pc_step 透传 (decode_rvc CB / 32-bit B-type):
    //   - 32-bit branch: PC_STEP_RV = 4
    //   - RVC C.BEQZ/C.BNEZ: PC_STEP_RVC = 2
    //   两条路径都通过 d.pc_step 在 not-taken 端自动适配, BRANCH_IF 不需要再判断 raw_inst
    //   位段 (decode 阶段已知信息透传到 d.pc_step; 详见 decode.h pc_step doc 设计哲学)。
    //
    // pc + 2 / pc + 4 都一定 IALIGN-aligned (pc 自身已对齐, +2/+4 不破坏对齐), 不需 check,
    // fetch loop 末段直接 += 即可。taken 路径仍走 WRITE_PC_OR_TRAP 做 misalign check (branch
    // target = pc + imm 可能 misalign)。
    //
    // 隐式捕获: hart, pc, d.imm, count, count_out。
    #define BRANCH_IF(cond) do {                                           \
        if (cond) {                                                        \
            WRITE_PC_OR_TRAP(pc + (uint32_t)d.imm);                        \
            count++;                                                       \
            SYNC_COUNT();                                                  \
            goto out;                                                      \
        }                                                                  \
        /* not-taken: 不写 pc, fetch loop 末段 += d.pc_step + boundary out */ \
    } while (0)

    // SYNC_COUNT —— 把当前 count 同步到 dispatcher 的 count_out 指针。
    //
    // 使用场景 (跟 JIT prologue/epilogue 哲学一致, dummy.txt §1):
    //   在"可能 trap / 回 dispatcher" 的边界同步 count, 让 longjmp 路径下 dispatcher 收到
    //   "trap 触发指令前已成功执行的指令数" — 跟 RV precise trap 一致 (trap 那条不算入)。
    //
    // 不在每条指令都同步 (避免 pure case 多付一次 store; 跟 JIT 内部 hot path 不付 store
    // 同哲学); 只在已知 may-trap 位置插入。具体位置:
    //   - WRITE_PC_OR_TRAP 内 trap_raise 之前 (misalign target; branch/jal/jalr 共用此宏)
    //   - case OP_ECALL / OP_EBREAK / OP_UNSUPPORTED — trap_raise_exception 之前
    //   - case OP_LB/LH/LW/LBU/LHU — lsu_load_helper 调用之前 (helper 内可能 trap_raise)
    //   - case OP_SB/SH/SW         — lsu_store_helper 调用之前
    //   - CSR 6 case (CSRRW/RS/RC/WI/SI/CI) — csr_op 调用之前 (csr_op 内可能 trap_raise
    //     illegal csr / privilege violation)
    //
    // 不需要插入的位置:
    //   - 算术 / 逻辑 / 移位 / NOP / LUI / AUIPC: pure case, 不可能 trap
    //   - OP_MRET / OP_SRET: csr 路径 + trap_set_state (dummy.txt §1 path 2b 不长跳)
    //   - OP_SFENCE_VMA: sfence_vma_helper 4.a 简化方案不 trap
    //   - branch / jal / jalr 自身: 通过 WRITE_PC_OR_TRAP 间接覆盖 (target misalign 才 trap)
    //
    // 漏标检测: 漏写 SYNC_COUNT 的 may-trap case → fixture 跑后 total_count 偏小 (缺该
    // block 的 count); cosmetic 不影响功能, 但 dump count 不对会立即提示, bug 浅显 (跟
    // dummy.txt §1 末段 helper may_trap/pure 标注不同, 那个漏标会让寄存器脏污难追)。
    //
    // 实施: 单赋值用 do-while(0) 包装让 SYNC_COUNT(); 像 statement (跟 BRANCH_IF /
    // WRITE_PC_OR_TRAP 同形态)。
    //
    // 隐式捕获: count (interpret_one_block 内局部变量), count_out (函数参数指针)。
    #define SYNC_COUNT() do { *count_out = count; } while (0)

    // LOAD_MISALIGN_CHECK / STORE_MISALIGN_CHECK —— gva 级 misalign 检查 (a_02
    // session_004 P3 后契约): caller (interpreter case 入口) 一处做, helper
    // (lsu_*_helper / mmu_walker_helper_* / store_helper) 都信任 caller 已查。
    //
    // size=1 时 mask=0, LB/LBU/SB 永远过 (但形式保留, 跟 LH/LW/SH/SW 一致, 同形宏)。
    //
    // SYNC_COUNT 在 misalign 触发前已调 (case 入口顺序: ea 算 → SYNC_COUNT() →
    // MISALIGN_CHECK → helper); trap 触发那条不算入 count_out, 跟 RV precise trap
    // 一致。
    //
    // 隐式捕获: hart (调 trap_raise_exception)。
    #define LOAD_MISALIGN_CHECK(ea, size)                                       \
        do { if (((ea) & ((size) - 1u)) != 0u)                                  \
                trap_raise_exception(hart, CAUSE_LOAD_ADDR_MISALIGNED, (ea));   \
        } while (0)
    #define STORE_MISALIGN_CHECK(ea, size)                                      \
        do { if (((ea) & ((size) - 1u)) != 0u)                                  \
                trap_raise_exception(hart, CAUSE_STORE_ADDR_MISALIGNED, (ea));  \
        } while (0)

    uint64_t count = 0;

    // 跨页软边界状态: 块入口 page 起点 (host_ram_base 4K 对齐 — ram.c mmap(NULL,...) 内核
    // 分配地址必然 page-aligned; gpa_to_hva_offset 也 4K 对齐 → invariant: (hva_pc & 0xFFF)
    // == (gva_pc & 0xFFF), 解释器在 hva 上判跨页等价于在 gva 上判, 不需单独保留 gva)。
    const uintptr_t page_mask  = ~(uintptr_t)0xFFFu;
    const uintptr_t entry_page = (uintptr_t)hva_pc & page_mask;

    while (count < BLOCK_INST_LIMIT) {
        // 取指: hva_pc 指向当前指令字节起点。块内每条指令完整在同一 4K page 内 (推进后
        // 跨页时退出 block, 见 fetch loop 末段)。memcpy 防 strict-aliasing / unaligned 风险
        // (RV32 指令必 4 字节对齐, 实际 hva_pc 也对齐, 但 memcpy 表达更通用; 编译器会优化
        // 为单 mov)。
        // 注: page 末 RVC (hva_pc & 0xFFF == 0xFFE) 时 memcpy 4 字节会 over-read 2 字节进
        // 下一 page。BARE / SV32 下 mmap 连续 128MB, over-read 物理无 segfault; decode_rvc
        // 只读低 16 位, over-read 字节不参与解码, 逻辑无害。RAM 末尾 4 字节是唯一例外
        // (fixture 不写到那)。
        uint32_t inst;
        memcpy(&inst, hva_pc, 4);

        const decoded_inst_t d = decode(inst);

        // 当前 PC = regs[0] (cpu.h: regs[0] 物理位置存 pc, x0 走 garbage 路径不碰这里)
        const uint32_t pc = hart->regs[0];

        // SLLI/SRLI/SRAI 的 shamt: decode 已把 5 位 shamt 放进 imm 低 5 位; R-type SLL/SRL/SRA
        // 用 rs2 寄存器值的低 5 位 (RV 规范要求; mini-rv32ima 同)。
        const uint32_t shamt_i = (uint32_t)d.imm & 0x1Fu;

        // ====================================================================
        // count_out 同步协议 (SYNC_COUNT 宏 doc 见上方)
        // ====================================================================
        //
        // count 跨 longjmp 跳走时, dispatcher 必须收到"trap 触发指令前已成功执行" 的
        // count_out 值; SYNC_COUNT() 是同步动作。两条同步路径覆盖所有 case:
        //
        //   (1) may-trap 路径 — case 内 SYNC_COUNT() 在 trap_raise / helper-call 之前
        //       覆盖: ECALL / EBREAK / UNSUPPORTED / CSR 6 case / load 5 case /
        //              store 3 case / WRITE_PC_OR_TRAP misalign (branch / jal / jalr
        //              共用宏)
        //       逻辑: 进 trap_raise / helper 后可能 _Noreturn longjmp, count_out 必须
        //              在 longjmp 之前已写, 否则 dispatcher 收到旧值 (跟 RV precise
        //              trap 一致, 触发指令本身不算入)
        //
        //   (2) boundary 路径 (托底) — case 末 break, fetch loop 末段 count++ +
        //       is_block_boundary_inst(&d) → goto out → out: 段 SYNC_COUNT()
        //       覆盖: CSR / SFENCE.VMA / MRET / SRET / 所有 branch / jal / jalr (不走
        //              trap 路径时); 跟 decode.h is_block_boundary_inst 列表一致
        //       逻辑: 这些"特殊指令"不需要 case 内 SYNC_COUNT 因为 csr_op /
        //              sfence_vma_helper 内 trap 路径自己 longjmp 前会 SYNC_COUNT
        //              (case 顶部已加); 不 trap 时 case 末 break, boundary check 必命中
        //              (因这些都是 hard boundary), 走 out 段 SYNC_COUNT 托底
        //
        // pure case (算术 / 逻辑 / 移位 / NOP / LUI / AUIPC) 既不 may-trap 也不 boundary,
        // case 末 break 后 fetch loop 末段 count++ 不 goto out, 继续下一轮 while; 这条路径
        // 不需要同步 (count 在 interp_one_block 栈帧上累加, 不影响 dispatcher 直到 block
        // 结束). 详见 SYNC_COUNT 宏 doc "不需要插入的位置" 段。
        // ====================================================================
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

            // ---- B-type BRANCH ----
            // 比较 rs1 / rs2 (按 funct3 决定有/无符号 + 比较方向); taken 走 pc + imm
            // (经 WRITE_PC_OR_TRAP 含对齐检查), not-taken 走 pc + 4 (固定 32-bit 分支)。
            case OP_BEQ:  BRANCH_IF(READ_REG(d.rs1)            == READ_REG(d.rs2));            break;
            case OP_BNE:  BRANCH_IF(READ_REG(d.rs1)            != READ_REG(d.rs2));            break;
            case OP_BLT:  BRANCH_IF((int32_t)READ_REG(d.rs1)   <  (int32_t)READ_REG(d.rs2));   break;
            case OP_BGE:  BRANCH_IF((int32_t)READ_REG(d.rs1)   >= (int32_t)READ_REG(d.rs2));   break;
            case OP_BLTU: BRANCH_IF(READ_REG(d.rs1)            <  READ_REG(d.rs2));            break;
            case OP_BGEU: BRANCH_IF(READ_REG(d.rs1)            >= READ_REG(d.rs2));            break;

            // ---- J-type JAL ----
            // rd = pc + d.pc_step (返回地址; 调用约定 rd=x1 是 ra, rd=x0 是无返回纯跳转;
            // d.pc_step = RV=4 / RVC=2 兼容 C.JAL ra=pc+2 spec); pc = pc + imm (经
            // WRITE_PC_OR_TRAP 含对齐检查)。
            //
            // 跟 BRANCH_IF taken 同形态: JAL/JALR 是 "总跳" control flow, case 自写 pc 后
            // 必须 goto out 跳过 fetch loop 末段 (避免 += d.pc_step 破坏 case 写的 jump
            // target); count++ + SYNC_COUNT 在 goto out 之前显式做 (跟 fetch loop 末段
            // 一致语义)。
            //
            // 写 rd / 写 pc 顺序 (precise trap 语义): 严格按 RV precise trap 应该 "算 target
            // → 检查对齐 → 写 rd → 写 pc"。当前 IALIGN=16 + jal/branch imm[0]=0 编码下
            // pc-misalign 永远不命中 (是 dead code), 写 rd / 写 pc 顺序在本项目不构成实际
            // 正确性 bug。简洁起见保持"写 rd 再 WRITE_PC_OR_TRAP"形态, 真做 IALIGN=32 时
            // 重构成严格顺序。
            case OP_JAL:
                WRITE_REG(d.rd, pc + d.pc_step);
                WRITE_PC_OR_TRAP(pc + (uint32_t)d.imm);
                count++;
                SYNC_COUNT();
                goto out;

            // ---- I-type JALR ----
            // 同 JAL, 区别: 目标 = (rs1 + imm) & ~1u (RV spec 强制 mask LSB; mask 后 bit[0]
            // 永远 0, 即 IALIGN=16 永远过)。
            case OP_JALR:
                WRITE_REG(d.rd, pc + d.pc_step);
                WRITE_PC_OR_TRAP((READ_REG(d.rs1) + (uint32_t)d.imm) & ~1u);
                count++;
                SYNC_COUNT();
                goto out;

            // ---- I-type SYSTEM CSR 6 变体 ----
            // 6 个 op_kind 在 csr 侧映射到 3 个内核操作 (csr_op_t) + new_val 来源:
            //   RW/RS/RC:    new_val = READ_REG(d.rs1)             /* rs1 是寄存器号 */
            //   RWI/RSI/RCI: new_val = (uint32_t)d.rs1             /* rs1 字段当 5-bit zimm */
            // csr_op 返回 read_old, WRITE_REG(d.rd, old) 写 rd (rd=x0 走 dummy.txt §2 dead store
            // 路径, 自然丢弃)。case 末 break (不 goto out), fetch loop 末 count++ 后
            // is_block_boundary_inst → 1 让 fetch loop 退出, dispatcher
            // 重派发 pc + 4 进下一块。
            //
            // d.imm 是 12-bit csr addr (decode 已无符号扩展放低 12 位, 高 20 位 0); 这里
            // (uint32_t)d.imm 强转去除 int32_t 符号扩展风险 (d.imm 实际值范围 [0, 0xFFF],
            // cast 等价于 d.imm & 0xFFF, 但语义清晰)。
            //
            // d.raw_inst 是 32-bit 原始指令编码 (decode 顶部已填), 给 csr_op 内 trap 路径
            // 填 mtval 用。
            case OP_CSRRW:
                SYNC_COUNT();
                WRITE_REG(d.rd, csr_op(hart, (uint32_t)d.imm, READ_REG(d.rs1),
                                       CSR_OP_RW, d.raw_inst));
                break;
            case OP_CSRRS:
                SYNC_COUNT();
                WRITE_REG(d.rd, csr_op(hart, (uint32_t)d.imm, READ_REG(d.rs1),
                                       CSR_OP_RS, d.raw_inst));
                break;
            case OP_CSRRC:
                SYNC_COUNT();
                WRITE_REG(d.rd, csr_op(hart, (uint32_t)d.imm, READ_REG(d.rs1),
                                       CSR_OP_RC, d.raw_inst));
                break;
            case OP_CSRRWI:
                SYNC_COUNT();
                WRITE_REG(d.rd, csr_op(hart, (uint32_t)d.imm, (uint32_t)d.rs1,
                                       CSR_OP_RW, d.raw_inst));
                break;
            case OP_CSRRSI:
                SYNC_COUNT();
                WRITE_REG(d.rd, csr_op(hart, (uint32_t)d.imm, (uint32_t)d.rs1,
                                       CSR_OP_RS, d.raw_inst));
                break;
            case OP_CSRRCI:
                SYNC_COUNT();
                WRITE_REG(d.rd, csr_op(hart, (uint32_t)d.imm, (uint32_t)d.rs1,
                                       CSR_OP_RC, d.raw_inst));
                break;

            // ---- I-type SYSTEM (ECALL / EBREAK / MRET) ----
            //
            // ECALL: 触发 environment-call trap, cause 按 priv 分流 (RV spec table 3.6):
            //          priv U → cause 8, priv S → cause 9, priv M → cause 11
            //          有意写成 (8 + hart->priv) 公式 — RV 编码巧合: PRIV_U=0, PRIV_S=1,
            //          PRIV_M=3 与 cause 8/9/11 差 8。Spike/QEMU 同做法。
            //          tval = 0 (RV spec §3.1.17: ECALL/EBREAK 的 mtval write 0)。
            //
            // EBREAK: cause 3 (breakpoint, RV spec); 不分 priv。tval 一般 = 0
            //          (实现 debug 子集时可填触发 PC, 项目当前 = 0)。
            //
            // MRET: 从 trap handler 回归; 不调 trap_raise_exception, 是 csr 路径 + trap_set_state
            //          的反操作:
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
            //          权限要求: MRET 仅在 priv >= M 时合法 (M-mode CSR 入口); MRET 本身不是
            //          csr 指令而是 system 指令, 没走 csr_op 入口判。U/S-mode 触发 MRET 应
            //          cause=2 (illegal instruction) — 当前 case 入口未做 priv 检查, 真做 OS
            //          隔离时在 case 入口加 priv 判 (跟 SRET 同形态)。
            case OP_ECALL:
                /* RV 编码巧合: PRIV_U=0/S=1/M=3 ↔ cause 8/9/11 (= CAUSE_ECALL_FROM_U + priv).
                 * Spike / QEMU 同写法; PRIV_H=2 在没 H 扩展下不会触发 (hart->priv ∉ {2}). */
                SYNC_COUNT();
                trap_raise_exception(hart, CAUSE_ECALL_FROM_U + hart->priv, /*tval*/0u);
                goto out;
            case OP_EBREAK:
                SYNC_COUNT();
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

            // ---- I-type SYSTEM SRET (跟 OP_MRET 同形态; S-mode 字段段 + sepc) ----
            //
            // RV Privileged Spec Vol II §3.3.2:
            //   priv = SPP (1-bit; 0=U, 1=S)
            //   SIE  = SPIE
            //   SPIE = 1
            //   SPP  = PRIV_U = 0 (least-priv reset)
            //   pc   = sepc (= trap.xepc[PRIV_S])
            //   in_trap = 0 (项目复位嵌套链, 跟 MRET 同)
            //
            // 权限要求 (跟 OP_MRET 同形态, 当前未实现):
            //   SRET 仅在 hart->priv >= S 时合法; U-mode SRET → cause=2 illegal instruction.
            //   mstatus.TSR=1 时 S-mode SRET 也 trap to M (cause=2). 真做 OS 隔离时跟 MRET
            //   一起补 (interpreter case 入口或 csr_op 风格的 priv 检查段)。
            case OP_SRET: {
                uint32_t mstatus_lo = (uint32_t)(hart->trap._mstatus & 0xFFFFFFFFu);

                /* hart->priv = SPP (1-bit; 0=U, 1=S) */
                hart->priv = (uint8_t)((mstatus_lo & MSTATUS_SPP) >> MSTATUS_SPP_SHIFT);

                /* mstatus.SIE = mstatus.SPIE */
                if (mstatus_lo & MSTATUS_SPIE) mstatus_lo |=  MSTATUS_SIE;
                else                           mstatus_lo &= ~MSTATUS_SIE;

                /* mstatus.SPIE = 1 (RV spec) */
                mstatus_lo |= MSTATUS_SPIE;

                /* mstatus.SPP = PRIV_U = 0 (RV spec least-priv reset; SPP 是 1-bit, 清 0) */
                mstatus_lo &= ~MSTATUS_SPP;

                hart->trap._mstatus = (hart->trap._mstatus & 0xFFFFFFFF00000000ULL)
                                    | (uint64_t)mstatus_lo;

                hart->trap.in_trap = 0;
                hart->regs[0]      = hart->trap.xepc[PRIV_S];   /* sepc */
                break;
            }

            // ---- I-type LOAD (5 op) ----
            //
            // 不对称设计 (a_02 session_004 P3 后): load 走 inline 顶层 lsu_load_helper
            // (isa/lsu.h); BARE 内联 RAM/MMIO 分流, SV32 TLB hit 直接 *hva (不调子 helper,
            // 因 TLB 缓存 hva + MMIO 不进 TLB → 命中路径结构不带分支, insight 1),
            // miss 调 mmu_walker_helper_load。
            //
            // 方案 A (helper 不知 signed): lsu_load_helper 返回低 size 字节有效 + 高位 0
            // 的 uint32_t, case 自做 sext (LB int8_t / LH int16_t cast → int32_t 再 cast
            // 回 uint32_t) 或 zext (LBU/LHU 直接传, 高位已是 0)。LW size=4 直传整 32 位。
            //
            // ea 算成 uint32_t (RV32 wraparound 算术; gva = rs1 + imm 自然 wrap)。imm 是
            // int32_t (decode 已 sign-ext), 加到 uint32_t 上要先 cast (避免 -Wsign-conversion;
            // 跟 BRANCH_IF / WRITE_PC_OR_TRAP 路径同模式)。
            //
            // misalign (gva & (size-1)) check 由 case 入口 LOAD_MISALIGN_CHECK 宏完成
            // (P3 后契约: lsu_load_helper / mmu_walker_helper_load 都信任 caller 已查);
            // 触发时 trap_raise_exception(cause 4, gva) 长跳。
            //
            // 其他错误路径 (lsu_load_helper / mmu_walker_helper_load / mmio_read_helper
            // 内部 trap_raise_exception _Noreturn longjmp):
            //   BARE PA 不在 RAM, MMIO 未命中 / device 拒绝 → cause 5 (access fault) / cause
            //   SV32 walker fault                          → cause 13 (page fault) / 5 (access fault)
            // 长跳走 dispatcher 落点; 这里 break 后的 fetch loop 末尾 += pc_step / count++ /
            // boundary 检查不会执行。
            case OP_LB: {
                uint32_t ea = READ_REG(d.rs1) + (uint32_t)d.imm;
                SYNC_COUNT();
                LOAD_MISALIGN_CHECK(ea, 1u);
                WRITE_REG(d.rd, (uint32_t)(int32_t)(int8_t) lsu_load_helper(hart, current_tlb, ea, 1u));
                break;
            }
            case OP_LH: {
                uint32_t ea = READ_REG(d.rs1) + (uint32_t)d.imm;
                SYNC_COUNT();
                LOAD_MISALIGN_CHECK(ea, 2u);
                WRITE_REG(d.rd, (uint32_t)(int32_t)(int16_t)lsu_load_helper(hart, current_tlb, ea, 2u));
                break;
            }
            case OP_LW: {
                uint32_t ea = READ_REG(d.rs1) + (uint32_t)d.imm;
                SYNC_COUNT();
                LOAD_MISALIGN_CHECK(ea, 4u);
                WRITE_REG(d.rd,                              lsu_load_helper(hart, current_tlb, ea, 4u));
                break;
            }
            case OP_LBU: {
                uint32_t ea = READ_REG(d.rs1) + (uint32_t)d.imm;
                SYNC_COUNT();
                LOAD_MISALIGN_CHECK(ea, 1u);
                WRITE_REG(d.rd,                              lsu_load_helper(hart, current_tlb, ea, 1u));
                break;
            }
            case OP_LHU: {
                uint32_t ea = READ_REG(d.rs1) + (uint32_t)d.imm;
                SYNC_COUNT();
                LOAD_MISALIGN_CHECK(ea, 2u);
                WRITE_REG(d.rd,                              lsu_load_helper(hart, current_tlb, ea, 2u));
                break;
            }

            // ---- S-type STORE (3 op) ----
            //
            // 不对称设计 (a_02 session_004 P3 后): store 走 inline 顶层 lsu_store_helper
            // (isa/lsu.h); BARE 内联 RAM/MMIO 分流, SV32 TLB hit 调 store_helper(hva,...)
            // (RAM 写 + LR/SC + SMC 副作用, insight 1), miss 调 mmu_walker_helper_store。
            //
            // SB/SH/SW 写多少字节由 size 决定; value = READ_REG(d.rs2) 整 32 位传给 helper,
            // 最终 memcpy size 字节 (SB 写低 8 位, SH 写低 16 位, SW 写全 32 位)。
            //
            // misalign 由 case 入口 STORE_MISALIGN_CHECK 宏完成 (P3 后契约同 load);
            // 触发 trap_raise_exception(cause 6, gva) 长跳。
            //
            // 其他错误路径 (lsu_store_helper / mmu_walker_helper_store / mmio_write_helper
            // 内部 trap_raise_exception _Noreturn longjmp):
            //   BARE PA 不在 RAM, MMIO 未命中 / device 拒绝 → cause 7 (access fault) / cause
            //   SV32 walker fault                          → cause 15 (page fault) / 7 (access fault)
            case OP_SB: {
                uint32_t ea = READ_REG(d.rs1) + (uint32_t)d.imm;
                SYNC_COUNT();
                STORE_MISALIGN_CHECK(ea, 1u);
                lsu_store_helper(hart, current_tlb, ea, READ_REG(d.rs2), 1u);
                break;
            }
            case OP_SH: {
                uint32_t ea = READ_REG(d.rs1) + (uint32_t)d.imm;
                SYNC_COUNT();
                STORE_MISALIGN_CHECK(ea, 2u);
                lsu_store_helper(hart, current_tlb, ea, READ_REG(d.rs2), 2u);
                break;
            }
            case OP_SW: {
                uint32_t ea = READ_REG(d.rs1) + (uint32_t)d.imm;
                SYNC_COUNT();
                STORE_MISALIGN_CHECK(ea, 4u);
                lsu_store_helper(hart, current_tlb, ea, READ_REG(d.rs2), 4u);
                break;
            }

            // ---- I-type SYSTEM SFENCE.VMA ----
            //
            // 接口设计: helper 接 4 个独立信息, 分两组 — 寄存器**值** (vaddr_val/asid_val,
            // 由 caller READ_REG 处理 x0 编码) 跟寄存器**编号**
            // (rs1/rs2, 0..31)。两组语义独立不可互相推导:
            //   - 值: 真做事用 (helper 4.a 简化下只用 asid_val; vaddr_val 是 (b)/(d) 精确实现
            //          的预留, 当前 (void) 抑制 unused)
            //   - 号: 判 RV spec "rs1=x0/rs2=x0" magic 编码 (= "忽略对应维度")
            // 不能用 vaddr_val=0 推断 rs1=x0 — 因 vaddr=0 是合法实值不等同"忽略"。详见
            // sfence.h 接口 doc。
            //
            // 透传 4 项 (按方案 B 参数顺序):
            //   READ_REG(d.rs1)  → vaddr_val   (READ_REG 处理 d.rs1==0 → 0 的 x0 编码)
            //   READ_REG(d.rs2)  → asid_val    (同上)
            //   d.rs1            → rs1 编号    (helper 内判 d.rs1==0 即 RV spec rs1=x0)
            //   d.rs2            → rs2 编号    (同上)
            //
            // d.rs1 = vaddr 寄存器号; d.rs2 = asid 寄存器号 (decode.h SFENCE.VMA 字段约定段)。
            //
            // case 末 break (不 goto out): fetch loop 末 += PC_STEP_RV (=4, decode 顶部
            // case 0x73 默认), count++, boundary 检查 (sfence.vma 是 boundary, decode.h
            // is_block_boundary_inst → 1) → goto out 退出 fetch loop, dispatcher 重派发
            // 时 block 1 重新算 (regime, current_tlb), 因为 TLB 状态已变。
            case OP_SFENCE_VMA:
                sfence_vma_helper(hart,
                                  READ_REG(d.rs1), READ_REG(d.rs2),
                                  d.rs1,           d.rs2);
                break;

            // ---- 兜底 ----
            case OP_UNSUPPORTED:
                // RV cause 2 = illegal instruction; tval = raw_inst (RV spec §3.1.16)。
                // helper 标 _Noreturn longjmp 跳回 dispatcher sigsetjmp 落点,
                // goto out 变 unreachable 但保留无害 (GCC -Wunreachable-code 默认 disabled)。
                SYNC_COUNT();
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

        // 跨页软边界: 推进后 hva_pc 进新 page → 退出 block, 让 dispatcher 重 mmu_translate_pc
        // 拿新 page 的 hva (SV32 下 walker 重做权限检查)。跟 BLOCK_INST_LIMIT 同性质 (软
        // 边界), 块入口指令必跑一次 (即使 hva_pc & 0xFFF 在 page 末) — entry_page 由块入口
        // 算出, 第一次进 fetch loop 时必命中本 page。
        if (((uintptr_t)hva_pc & page_mask) != entry_page) goto out;

        // 硬边界判定: branch/jal/jalr 类 op 已在上方 case 内写好新 pc, 此处
        // 退出 fetch loop, 让 dispatcher 重新 mmu_translate_pc 拿新入口的 hva 进下一块。
        // boundary 那条指令计入 count (precise: 已成功执行); trap 路径走 WRITE_PC_OR_TRAP
        // 内 goto out, 不到此处, count 不含 trap 那条。
        if (is_block_boundary_inst(&d)) goto out;
    }

out:
    /* boundary 路径 (case 末 break + fetch loop 末段 count++ + is_block_boundary_inst → goto out)
     * 的 count_out 同步; 跟 may-trap 路径的 SYNC_COUNT() 共用同一份同步语义 */
    SYNC_COUNT();

    #undef READ_REG
    #undef WRITE_REG
    #undef WRITE_PC_OR_TRAP
    #undef BRANCH_IF
    #undef SYNC_COUNT
    #undef LOAD_MISALIGN_CHECK
    #undef STORE_MISALIGN_CHECK
}
