//
// Created by liujilan on 2026/4/28.
// a_01 main。
// 当前形态: decode_test 单测 → ram_init → loader → cpu_create + 字段设置 →
//             dispatcher 一次 → 末尾 fprintf 关键寄存器 → cpu_destroy。
// dispatcher 当前 a_01_4 形态 (while(1) 多块循环, count==0 break + 单行 halted 总结);
// sigsetjmp / 完整 jit_cache 派发 / perf_advance 等 a_01_5 真接入。
//

#include "config.h"
#include "core/cpu.h"
#include "core/decode.h"
#include "core/dispatcher.h"
#include "core/mmu.h"   // mmu_walk + mmu_perm_t (a_01_8 Step 5b decision M sanity check)
#include "loader.h"
#include "platform/ram.h"
#include "riscv.h"

#include <stdio.h>
#include <string.h>

static int has_suffix(const char *s, const char *suffix) {
    size_t ns = strlen(s);
    size_t nsuf = strlen(suffix);
    if (ns < nsuf) return 0;
    return strcmp(s + ns - nsuf, suffix) == 0;
}

// ----------------------------------------------------------------------------
// decode 单元测试 (a_01_4 起步搭框架, sanity test 几个 a_01_3 已实现 op)
//
// decode 是纯函数, 不依赖 cpu_t / mmu / tlb / ram, 单测放 main.c 内 (tests/unit/ 框架
// 推迟到 a_01_5+, 现在直接 main 内一个 helper)。每跑 ./jit-emu 都顺手过一次, fail
// 就 return 1 不让 fixture 跑。
//
// 加新 op_kind 时 (a_01_4 加 BEQ/BNE/BLT/BGE/BLTU/BGEU + JAL/JALR; a_01_5 加 CSR* /
// ECALL / EBREAK / MRET; a_01_6 加 LB/LH/LW/LBU/LHU + SB/SH/SW), 顺手在这里加几个
// CASE() 验证。S/B/J 立即数解码位拼接奇怪, 必须按 spec 表对照 + 单测验证 (max+ /
// max- / 0 / off-by-one 各一个边界值)。
// ----------------------------------------------------------------------------
static int decode_test(void) {
    int fail = 0;
    int total = 0;

    #define CASE(raw, ek, erd, ers1, ers2, eimm, eraw, estep)                          \
        do {                                                                           \
            total++;                                                                   \
            decoded_inst_t _d = decode((uint32_t)(raw));                                \
            if (_d.kind != (ek) || _d.rd != (uint32_t)(erd) ||                          \
                _d.rs1 != (uint32_t)(ers1) || _d.rs2 != (uint32_t)(ers2) ||             \
                _d.imm != (int32_t)(eimm) || _d.raw_inst != (uint32_t)(eraw) ||         \
                _d.pc_step != (uint32_t)(estep)) {                                      \
                fprintf(stderr,                                                        \
                    "[decode_test] FAIL raw=0x%08x: kind=%d rd=%u rs1=%u rs2=%u imm=%d raw_inst=0x%x pc_step=%u\n", \
                    (uint32_t)(raw), _d.kind, _d.rd, _d.rs1, _d.rs2, _d.imm,            \
                    _d.raw_inst, _d.pc_step);                                           \
                fail++;                                                                \
            }                                                                          \
        } while (0)

    // ---- a_01_3 sanity (32-bit RV, pc_step = PC_STEP_RV = 4) ----
    //
    // 注: CASE 检查所有 7 个字段 (kind/rd/rs1/rs2/imm/raw_inst/pc_step), 含 decode 按
    //     通用 (inst>>X)&0x1F 提取的 garbage 字段 (I-type 的 rs2 字段实际是 imm 低 5 位;
    //     U-type 的 rs1/rs2 是 imm 中间位)。eraw 与 raw 一般相同, 仅 RVC 场景 raw_inst
    //     是低 16 位 (高 16 位是 fetch over-read 残值 → eraw 给低 16 位即可)。
    //
    // I-type OP-IMM: addi x1, x0, 42 = 0x02A00093 (rs2 字段 garbage = 0x02A & 0x1F = 10)
    CASE(0x02A00093, OP_ADDI,  /*rd*/1, /*rs1*/0, /*rs2*/10, 42, 0x02A00093, PC_STEP_RV);
    // I-type OP-IMM: addi x2, x0, 8 = 0x00800113 (rs2 garbage = 8)
    CASE(0x00800113, OP_ADDI,  /*rd*/2, /*rs1*/0, /*rs2*/8, 8, 0x00800113, PC_STEP_RV);
    // R-type OP: add x3, x1, x2 = 0x002081B3 (字段无 garbage)
    CASE(0x002081B3, OP_ADD,   /*rd*/3, /*rs1*/1, /*rs2*/2, 0, 0x002081B3, PC_STEP_RV);
    // U-type: lui x5, 1 = 0x000012B7 (imm = 0x1000, rs1/rs2 garbage = 0)
    CASE(0x000012B7, OP_LUI,   /*rd*/5, /*rs1*/0, /*rs2*/0, 0x1000, 0x000012B7, PC_STEP_RV);
    // I-type OP-IMM 负 imm: addi x6, x0, -8 = 0xFF800313 (rs2 garbage = 0xFF8 & 0x1F = 24)
    CASE(0xFF800313, OP_ADDI,  /*rd*/6, /*rs1*/0, /*rs2*/24, -8, 0xFF800313, PC_STEP_RV);
    // ecall = 0x00000073: a_01_3 时 OP_UNSUPPORTED, a_01_5_c 起细分到 OP_ECALL (decode 0x73
    //                      funct3=0 + imm[11:0]=0 → ECALL); pc_step=PC_STEP_NONE (trap 跳 xtvec,
    //                      不让 fetch loop 末 += pc_step 误推)
    CASE(0x00000073, OP_ECALL, /*rd*/0, /*rs1*/0, /*rs2*/0, 0, 0x00000073, PC_STEP_NONE);

    // ---- a_01_4 起步: 16-bit RVC (compressed), pc_step = PC_STEP_RVC = 2 ----
    //
    // 用户拍 viii 范围: 现在做 C.LI + C.ADDI 两个 (fixture 之前 gcc 触发的就是 C.LI;
    // C.ADDI 与 C.LI decode 路径相似一起做)。其他 RVC → OP_UNSUPPORTED + pc_step=
    // PC_STEP_RVC (fetch loop 仍 +2 推进位置正确)。
    //
    // RVC encoding (Spec Vol I §16): inst[1:0] != 11 → 16-bit; quadrant = inst[1:0],
    // funct3 = inst[15:13]。decode 翻译到 RV32I op_kind (RVC 是长度变化, 语义同源)。
    //
    // raw_inst 字段在 RVC 时是 (uint32_t)inst (低 16 位有效), 高 16 位是 0 (因为我们直接
    // pass uint16_t cast 后高位填 0; 注意 fetch loop 实际从 hva_pc memcpy 4 字节会读到
    // 下一条指令的 2 字节作为 over-read, 但 decode 只看低 16 位, 高 16 位不参与判定)。
    //
    // C.LI x1, 42 = 0x50A9 (010 1 00001 01010 01)
    //   funct3=010 (C.LI), imm[5]=1, rd=00001=1, imm[4:0]=01010=10
    //   imm6 = 0b101010 = 42 (no sign-ext, bit 5 = 1 但 imm 是无符号到有符号映射 (101010
    //   sign-ext 为正 42 因为 我们的 sign-ext 检查 bit 5 = 1, 应该 ext 为负)... 等等
    //   re-check: imm6 = 6-bit signed; bit 5 = 1 → 负数. 那 0b101010 = -22 (二补码), 不是 42!
    //   实际编码 C.LI x1, 42: 42 = 0b101010, 高 1 位 = 1 是 sign bit → 负数 -22? 不对!
    //   C.LI 立即数是 sign-extended 6 位, 42 不能用 6 位有符号表示 (范围 [-32, 31])。
    //   所以 gcc 不可能为 addi x1, x0, 42 emit C.LI x1, 42。让我重新算 fixture 之前那条:
    //   stub.S: addi x1, x0, 42, gcc 没生成 C.LI (因为 42 超 [-32,31])。
    //   gcc 生成的实际是 C.LI x2, 8 (= 0x4121), 因为 8 在 [-32, 31] 范围。
    //   所以 RVC test 用 8 而不是 42:
    //     C.LI x2, 8 = 0x4121: 010 0 00010 01000 01
    //       funct3=010, imm[5]=0, rd=2, imm[4:0]=01000=8 → imm6 = 0b001000 = 8 (正)
    //   预期 decode 输出: kind=OP_ADDI, rd=2, rs1=0, rs2=garbage, imm=8, raw_inst=0x4121,
    //                    pc_step=PC_STEP_RVC
    //   rs2 garbage: decode_rvc 设 d.rs2 = 0 (我们没用通用 inst>>20 提取)
    CASE(0x4121, OP_ADDI, /*rd*/2, /*rs1*/0, /*rs2*/0, 8, 0x4121, PC_STEP_RVC);
    // C.LI x1, -1 = 0x50FD: 010 1 00001 11111 01
    //   funct3=010, imm[5]=1, rd[4:0]=00001=1, imm[4:0]=11111=31 → imm6=0b111111 sign-ext = -1
    CASE(0x50FD, OP_ADDI, /*rd*/1, /*rs1*/0, /*rs2*/0, -1, 0x50FD, PC_STEP_RVC);
    // C.ADDI x3, 5 = 0x0195: 000 0 00011 00101 01
    //   funct3=000 (C.ADDI), imm[5]=0, rd=3, imm[4:0]=00101=5
    //   预期: kind=OP_ADDI, rd=3, rs1=3 (C.ADDI 是 rd=rd+imm), imm=5
    CASE(0x0195, OP_ADDI, /*rd*/3, /*rs1*/3, /*rs2*/0, 5, 0x0195, PC_STEP_RVC);

    // RVC OP_UNSUPPORTED (其他 RVC 指令 a_01_4 起步不识别): C.MV x3, x1 = 0x8186
    //   100 0 00011 00001 10 (funct3=100, op=10 即 C2 quadrant); 我们只翻译 C1 (op=01)
    //   decode_rvc C2 路径全部归 OP_UNSUPPORTED, 验证 fetch loop +=2 路径
    CASE(0x8186, OP_UNSUPPORTED, /*rd*/0, /*rs1*/0, /*rs2*/0, 0, 0x8186, PC_STEP_RVC);

    // ---- a_01_4 boundary 立即数解码 (8 case, max+ / max- / 0 / off-by-one 边界值)----
    //
    // 选取规则 (与之前 chat 拍的一致): B-type 4 case (近距正负 + 0 + max+) + J-type 3 case
    // (近距正负 + max+) + JALR 1 case (I-type 立即数, 复用 ADDI 路径)。max- (B=-4096 / J=
    // -1MiB) 不单独测, 因为 -8 / -0x100 已经覆盖 sign-ext 走通; 真担心 sign 漏 ext 那 -8
    // 这条就先 fail 了。
    //
    // 编码细节看 decode.c B-type / J-type 的 4 段位拼接注释; rd / rs1 / rs2 字段在 B/J 类型
    // 实际复用为 imm 段 (rd 字段 = imm[11]<<4|imm[4:1] 等), 所以 CASE 期望值含这些 garbage
    // (decode.c 用通用 (inst>>X)&0x1F 提取, 不特判)。

    // BEQ x1, x0, +8 = 0x00008463
    //   imm=8 (13-bit signed): imm[12]=0, imm[11]=0, imm[10:5]=0, imm[4:1]=4, imm[0]=0
    //   rd garbage (bits 11:7) = imm[4:1]<<1 | imm[11] = 4<<1 | 0 = 8
    CASE(0x00008463, OP_BEQ,  /*rd*/8,  /*rs1*/1, /*rs2*/0, 8,    0x00008463, PC_STEP_NONE);
    // BEQ x1, x0, -8 = 0xFE008CE3 (sign-ext 验证)
    //   imm=-8: imm[12]=1, imm[11]=1, imm[10:5]=63, imm[4:1]=12, imm[0]=0
    //   rd garbage = 12<<1 | 1 = 25
    CASE(0xFE008CE3, OP_BEQ,  /*rd*/25, /*rs1*/1, /*rs2*/0, -8,   0xFE008CE3, PC_STEP_NONE);
    // BNE x0, x0, 0 = 0x00001063 (零偏移 / 自跳, imm 全 0 验证不会"凭空"算出非 0)
    CASE(0x00001063, OP_BNE,  /*rd*/0,  /*rs1*/0, /*rs2*/0, 0,    0x00001063, PC_STEP_NONE);
    // BLTU x1, x2, +4094 = 0x7E20EFE3 (B-type max+, imm[12]=0 + 其他位全 1)
    //   imm=4094: imm[12]=0, imm[11]=1, imm[10:5]=63, imm[4:1]=15, imm[0]=0
    //   rd garbage = 15<<1 | 1 = 31
    CASE(0x7E20EFE3, OP_BLTU, /*rd*/31, /*rs1*/1, /*rs2*/2, 4094, 0x7E20EFE3, PC_STEP_NONE);

    // JAL x0, +0x100 = 0x1000006F
    //   imm=0x100=256: imm[20]=0, imm[19:12]=0, imm[11]=0, imm[10:1]=0x080 (bit 8 only), imm[0]=0
    //   rs1 garbage (bits 19:15) = imm[19:15] = 0; rs2 garbage (bits 24:20) = 0
    CASE(0x1000006F, OP_JAL,  /*rd*/0,  /*rs1*/0,  /*rs2*/0,  0x100,    0x1000006F, PC_STEP_NONE);
    // JAL x0, -0x100 = 0xF01FF06F (sign-ext 验证)
    //   imm=-256: imm[20]=1, imm[19:12]=0xFF, imm[11]=1, imm[10:1]=0x380, imm[0]=0
    //   rs1 garbage (bits 19:15) = imm[19:15] = 0x1F = 31
    //   rs2 garbage (bits 24:20) = imm[10:6] from inst[24:21]+inst[20] = ...inst[20]=imm[11]=1
    //                              + inst[24:21]=imm[10:7]=0b1110 → 0b00001 + 0b1110<<1
    //                              简单做: (inst >> 20) & 0x1F = 0xF01 & 0x1F = 1
    CASE(0xF01FF06F, OP_JAL,  /*rd*/0,  /*rs1*/31, /*rs2*/1,  -0x100,   0xF01FF06F, PC_STEP_NONE);
    // JAL x0, +max(0xFFFFE) = 0x7FFFF06F (J-type max+, imm[20]=0 + 其他位全 1)
    //   imm=0xFFFFE=1048574: imm[20]=0, imm[19:12]=0xFF, imm[11]=1, imm[10:1]=0x3FF, imm[0]=0
    //   rs1 garbage = 0x1F = 31; rs2 garbage = (inst>>20)&0x1F = 0x7FF & 0x1F = 0x1F = 31
    CASE(0x7FFFF06F, OP_JAL,  /*rd*/0,  /*rs1*/31, /*rs2*/31, 0xFFFFE,  0x7FFFF06F, PC_STEP_NONE);

    // JALR x0, x1, +4 = 0x00408067 (I-type 立即数, 与 ADDI sign-ext 路径同源, 此处仅验证
    //   opcode=0x67 + funct3=0 路由对 + pc_step=PC_STEP_NONE)
    //   rs2 garbage (bits 24:20) = imm[4:0] = 4
    CASE(0x00408067, OP_JALR, /*rd*/0,  /*rs1*/1,  /*rs2*/4,  4,        0x00408067, PC_STEP_NONE);

    // ---- a_01_5_a I-type SYSTEM CSR 6 变体 (7 case: 6 op_kind 各 1 + csr_addr=0 边界)----
    //
    // 编码 (RV spec): csr_addr inst[31:20] / rs1_or_zimm inst[19:15] / funct3 inst[14:12]
    //                  (001=RW 010=RS 011=RC 101=RWI 110=RSI 111=RCI) / rd inst[11:7] /
    //                  opcode 0x73。decoded_inst_t 字段约定 (decode.h enum 段已 doc):
    //                  d.imm = csr 12-bit addr (无符号扩展, 高 20 位 0); d.rs1 字段在 RWI/RSI/RCI
    //                  时是 5-bit zimm (interpreter 不查 regs 直接用数值), 在 RW/RS/RC 时是
    //                  rs1 寄存器号; 字段共用同一位置 (Spike / QEMU 同做法)。
    //                  d.rs2 是 garbage = (inst>>20) & 0x1F = csr_addr & 0x1F (csr decode 不
    //                  动 d.rs2, 走顶部统一提取); CASE 宏期望值按此填。
    //                  d.pc_step = PC_STEP_RV (csr 不是 control flow, fetch loop +4; 但是硬
    //                  边界, 由 is_block_boundary_inst 让 fetch loop 退出)。

    // CSRRW x1, mtvec, x2 = 0x305110F3 (csr=0x305 mtvec, rs1=2, rd=1)
    CASE(0x305110F3, OP_CSRRW,  /*rd*/1, /*rs1*/2,  /*rs2*/5,  0x305, 0x305110F3, PC_STEP_RV);
    // CSRRS x3, mstatus, x4 = 0x300221F3 (csr=0x300 mstatus, rs1=4, rd=3)
    CASE(0x300221F3, OP_CSRRS,  /*rd*/3, /*rs1*/4,  /*rs2*/0,  0x300, 0x300221F3, PC_STEP_RV);
    // CSRRC x0, mcause, x5 = 0x3422B073 (csr=0x342 mcause, rs1=5, rd=0; 验 rd=x0 路径)
    CASE(0x3422B073, OP_CSRRC,  /*rd*/0, /*rs1*/5,  /*rs2*/2,  0x342, 0x3422B073, PC_STEP_RV);
    // CSRRWI x6, mepc, 0 = 0x34105373 (csr=0x341 mepc, zimm=0 边界, rd=6; 验 zimm=0 不真写规则)
    CASE(0x34105373, OP_CSRRWI, /*rd*/6, /*rs1*/0,  /*rs2*/1,  0x341, 0x34105373, PC_STEP_RV);
    // CSRRSI x7, mtval, 31 = 0x343FE3F3 (csr=0x343 mtval, zimm=31 max, rd=7)
    CASE(0x343FE3F3, OP_CSRRSI, /*rd*/7, /*rs1*/31, /*rs2*/3,  0x343, 0x343FE3F3, PC_STEP_RV);
    // CSRRCI x0, 0xFFF, 31 = 0xFFFFF073 (csr_addr=0xFFF 边界, zimm=31 max, rd=0)
    CASE(0xFFFFF073, OP_CSRRCI, /*rd*/0, /*rs1*/31, /*rs2*/31, 0xFFF, 0xFFFFF073, PC_STEP_RV);
    // CSRRW x0, 0x000, x0 = 0x00001073 (csr_addr=0x000 边界, rs1=x0, rd=x0; 全零路径)
    CASE(0x00001073, OP_CSRRW,  /*rd*/0, /*rs1*/0,  /*rs2*/0,  0x000, 0x00001073, PC_STEP_RV);

    // ---- a_01_5_c I-type SYSTEM (ECALL / EBREAK / MRET) ----
    //
    // opcode 0x73, funct3=000, 由 imm[11:0] (= inst[31:20]) 进一步区分:
    //   imm = 0x000 → ECALL  (上方 a_01_3 sanity 那条已覆盖, 顺手验证 a_01_5_c 后路由对)
    //   imm = 0x001 → EBREAK
    //   imm = 0x302 → MRET
    // d.rs2 garbage = (inst>>20) & 0x1F = imm 低 5 位; pc_step = PC_STEP_NONE (case 自描述 pc)。

    // EBREAK = 0x00100073
    CASE(0x00100073, OP_EBREAK, /*rd*/0, /*rs1*/0, /*rs2*/1, 0x001, 0x00100073, PC_STEP_NONE);
    // MRET = 0x30200073
    CASE(0x30200073, OP_MRET,   /*rd*/0, /*rs1*/0, /*rs2*/2, 0x302, 0x30200073, PC_STEP_NONE);
    // SRET = 0x10200073 (a_01_8 Step 6; imm=0x102; rs2 garbage = imm[4:0] = 2)
    CASE(0x10200073, OP_SRET,   /*rd*/0, /*rs1*/0, /*rs2*/2, 0x102, 0x10200073, PC_STEP_NONE);

    // ---- a_01_6 LOAD 5 + STORE 3 (I-type / S-type 立即数边界值)----
    //
    // LOAD I-type 立即数复用 ADDI 同型 (sign-ext inst[31:20]); S-type 立即数 12 位拼接:
    //   imm[11:5] = inst[31:25] (高 7 位); imm[4:0] = inst[11:7] (低 5 位)。
    // 选取规则 (跟 a_01_4 boundary 风格一致): 5 LOAD 各 1 case (各 funct3 验路由);
    // 3 STORE 各 1 case + S-type 立即数 max+/max-/0 边界各 1 case = 6 个; 共 11 case。
    // 实际只测 8 case (5 load + 3 store, max+/max- 走 LW/SW, 0 走 LB), 简化覆盖。

    // LB x1, 0(x2) = 0x00010083 (rd=1, rs1=2, imm=0, funct3=0 LB; rs2 garbage = imm[4:0] = 0)
    CASE(0x00010083, OP_LB,  /*rd*/1, /*rs1*/2, /*rs2*/0,  0,    0x00010083, PC_STEP_RV);
    // LH x3, -2(x4) = 0xFFE21183 (imm=-2 = 0xFFE; rs2 garbage = imm[4:0] = 30)
    //   -2 编码 12 位 = 0xFFE = inst[31:20] = 0b111111111110
    //   rs2 字段 inst[24:20] = 0b11110 = 30
    CASE(0xFFE21183, OP_LH,  /*rd*/3, /*rs1*/4, /*rs2*/30, -2,   0xFFE21183, PC_STEP_RV);
    // LW x5, 2047(x6) = 0x7FF32283 (I-type max+, imm=0x7FF=2047; rs2 garbage = 0x1F)
    CASE(0x7FF32283, OP_LW,  /*rd*/5, /*rs1*/6, /*rs2*/31, 2047, 0x7FF32283, PC_STEP_RV);
    // LBU x7, 1(x8) = 0x00144383 (rd=7, rs1=8, imm=1, funct3=4; rs2 garbage = 1)
    CASE(0x00144383, OP_LBU, /*rd*/7, /*rs1*/8, /*rs2*/1,  1,    0x00144383, PC_STEP_RV);
    // LHU x9, -2048(x10) = 0x80055483 (I-type max-, imm=-2048=0x800; rs2 garbage = 0)
    CASE(0x80055483, OP_LHU, /*rd*/9, /*rs1*/10, /*rs2*/0, -2048, 0x80055483, PC_STEP_RV);

    // SB x1, 0(x2) = 0x00110023 (rs1=2, rs2=1, imm=0, funct3=0 SB; rd garbage = imm[4:0] = 0)
    CASE(0x00110023, OP_SB,  /*rd*/0,  /*rs1*/2,  /*rs2*/1, 0,    0x00110023, PC_STEP_RV);
    // SH x3, -4(x4) = 0xFE321E23 (imm=-4; imm[11:5]=0x7F=inst[31:25], imm[4:0]=0x1C=inst[11:7])
    //   -4 = 0xFFC = 0b111111111100; imm[11:5]=0b1111111=0x7F, imm[4:0]=0b11100=28
    //   rd garbage = inst[11:7] = 28
    CASE(0xFE321E23, OP_SH,  /*rd*/28, /*rs1*/4,  /*rs2*/3, -4,   0xFE321E23, PC_STEP_RV);
    // SW x5, 2047(x6) = 0x7E532FA3 (S-type max+, imm=0x7FF=2047)
    //   imm[11:5] = 0x3F = inst[31:25]=0b0111111; imm[4:0]=0x1F=inst[11:7]=0b11111
    //   rd garbage = inst[11:7] = 31
    CASE(0x7E532FA3, OP_SW,  /*rd*/31, /*rs1*/6,  /*rs2*/5, 2047, 0x7E532FA3, PC_STEP_RV);

    // ---- a_01_8 SFENCE.VMA (验 funct7=0x09 路由 + d.rs1/d.rs2 字段透传)----
    //
    // 编码 (RV Privileged Spec §10.6.1): opcode=0x73 + funct3=0 + funct7=0x09 + rd=0 +
    // rs1=vaddr_reg + rs2=asid_reg。decode.c funct3=0 子段先看 funct7=0x09 → SFENCE.VMA,
    // 否则按 imm[11:0] 区分 ECALL/EBREAK/MRET (跟 a_01_5_c 路径并存)。
    //
    // d.imm 字段是 decode 顶部 inst[31:20] 提取所得 (case 0x73 line 249), 即 funct7|rs2:
    //   d.imm = (0x09 << 5) | rs2 = 0x120 + rs2
    // sfence case 不读 d.imm (语义不需要), 仅供 raw_inst trap 路径备查; CASE 期望值按通用提取
    // 规则填。d.pc_step = PC_STEP_RV (sfence 不是 control flow, +4; 块边界由 is_block_boundary_inst
    // 处理, 见 decode.h)。

    // sfence.vma x0, x0 = 0x12000073 (全清形态 — a_01_8 简化方案 4.a 真用; rs1=0, rs2=0)
    //   funct7=0x09 + rs2=0 → imm = 0x09<<5 | 0 = 0x120
    CASE(0x12000073, OP_SFENCE_VMA, /*rd*/0, /*rs1*/0,  /*rs2*/0,  0x120, 0x12000073, PC_STEP_RV);
    // sfence.vma x1, x2 = 0x12208073 (普通形态; rs1=1, rs2=2 — 验 d.rs1/d.rs2 字段透传)
    //   funct7=0x09 + rs2=2 → imm = 0x09<<5 | 2 = 0x122
    CASE(0x12208073, OP_SFENCE_VMA, /*rd*/0, /*rs1*/1,  /*rs2*/2,  0x122, 0x12208073, PC_STEP_RV);

    // ---- 未来 RVC 扩展 (C.MV / C.ADD / C.LUI / C.SUB / ... 真翻译) ----

    #undef CASE

    if (fail == 0) {
        fprintf(stderr, "[decode_test] PASS (%d/%d)\n", total - fail, total);
    } else {
        fprintf(stderr, "[decode_test] FAIL (%d/%d)\n", total - fail, total);
    }
    return fail;
}

// ----------------------------------------------------------------------------
// mmu_walk_test (Step 5b decision M, a_01_session_010 末段拍; 临时调试 helper,
// Step 9 整 a_01_8 跑通后删 — 全部代码 + 调用 + #include "core/mmu.h" 一起删)
//
// 在 cpu_create + hart 字段初始化 之后, dispatcher 之前调用; sanity check mmu_walk
// 算法不依赖 fixture 真跑解释器路径。
//
// 流程: build PT bytes 在 RAM 内 (避开 fixture program 区, 选 1 MiB 偏移; fixture < 1 MiB);
// 临时设 hart->satp / priv 让 mmu_walk 走 SV32 路径; 调 mmu_walk 验返回值 + 验 walker
// 已 set A/D 写回 PT; 测完恢复 hart 字段 + 清 PT bytes 避免污染 fixture program。
//
// 8 个用例覆盖典型路径:
//   1. 4KB page typical (R access, U=1, PRIV_U)              → 成功, 验 pa + A=1 set
//   2. 4KB page W access (同 PT 槽, R+W+U=1)                 → 成功, 验 pa + A+D=1 set
//   3. V=0 PTE (level=0)                                     → fail, cause=13
//   4. level=0 leaf misformatted (V=1 + R/W/X 全 0)          → fail, cause=13
//   5. PRIV_U + PTE.U=0                                       → fail, cause=13
//   6. 4MB superpage misaligned (level=1 leaf, PPN[0]≠0)     → fail, cause=13
//   7. 4MB superpage 正常 (level=1 leaf, PPN[0]=0)           → 成功, 验 pa (4MB-aligned)
//   8. PT 不在 RAM (satp.PPN 指向 RAM 外)                     → fail, cause=5 (access fault)
// ----------------------------------------------------------------------------
static int mmu_walk_test(cpu_t *hart) {
    int fail  = 0;
    int total = 0;

    /* PT 位置: GUEST_RAM_START + 1 MiB 偏移 (避开 fixture program; fixture 都 < 1 MiB) */
    const uint32_t root_pt_pa   = GUEST_RAM_START + 0x00100000UL;
    const uint32_t leaf_pt_pa   = GUEST_RAM_START + 0x00101000UL;
    const uint32_t leaf_page_pa = GUEST_RAM_START + 0x00102000UL;

    /* 备份 hart 字段 (测完恢复) */
    const uint32_t orig_satp    = hart->satp;
    const uint8_t  orig_priv    = hart->priv;
    const uint64_t orig_mstatus = hart->trap._mstatus;

    /* 临时设 satp.MODE=1 (SV32) + ASID=0 + PPN=root_pt_pa>>12; mstatus 默认 (SUM=0, MXR=0) */
    hart->satp = (1u << 31) | (0u << 22) | (root_pt_pa >> 12);

    #define EXPECT_OK(_perm, _gva, _expected_pa, _exp_a, _exp_d) do {                  \
        total++;                                                                       \
        uint32_t _pa, _flags, _cause;                                                  \
        int _rc = mmu_walk(hart, (uint32_t)(_gva), (_perm), &_pa, &_flags, &_cause);   \
        int _a_set = ((_flags & PTE_A) != 0);                                          \
        int _d_set = ((_flags & PTE_D) != 0);                                          \
        if (_rc != 0 || _pa != (uint32_t)(_expected_pa)                                \
            || _a_set != (_exp_a) || _d_set != (_exp_d)) {                             \
            fprintf(stderr,                                                            \
                    "[mmu_walk_test] case %d FAIL: rc=%d pa=0x%08x flags=0x%x "        \
                    "(expected pa=0x%08x A=%d D=%d)\n",                                \
                    total, _rc, _pa, _flags,                                           \
                    (uint32_t)(_expected_pa), (_exp_a), (_exp_d));                     \
            fail++;                                                                    \
        }                                                                              \
    } while (0)

    #define EXPECT_FAIL(_perm, _gva, _expected_cause) do {                             \
        total++;                                                                       \
        uint32_t _pa, _flags, _cause;                                                  \
        int _rc = mmu_walk(hart, (uint32_t)(_gva), (_perm), &_pa, &_flags, &_cause);   \
        if (_rc == 0 || _cause != (uint32_t)(_expected_cause)) {                       \
            fprintf(stderr,                                                            \
                    "[mmu_walk_test] case %d FAIL: rc=%d cause=%u (expected fail cause=%u)\n", \
                    total, _rc, _cause, (uint32_t)(_expected_cause));                  \
            fail++;                                                                    \
        }                                                                              \
    } while (0)

    /* ---- case 1: 4KB page typical (R access, U=1, PRIV_U) → 成功, A=1 set ---- */
    {
        memset(gpa_to_hva_offset + root_pt_pa, 0, 4096);
        memset(gpa_to_hva_offset + leaf_pt_pa, 0, 4096);
        /* root PT[0] = pointer-to-next-level (V=1, R/W/X=0; PPN=leaf_pt>>12) */
        uint32_t pte1 = PTE_V | ((leaf_pt_pa >> 12) << 10);
        memcpy(gpa_to_hva_offset + root_pt_pa + 0, &pte1, 4);
        /* leaf PT[0] = leaf PTE (V=1, R=1, U=1; PPN=leaf_page>>12) */
        uint32_t pte0 = PTE_V | PTE_R | PTE_U | ((leaf_page_pa >> 12) << 10);
        memcpy(gpa_to_hva_offset + leaf_pt_pa + 0, &pte0, 4);

        hart->priv = PRIV_U;
        EXPECT_OK(MMU_PERM_R, /*gva=*/0x00000004UL,
                  /*pa=*/leaf_page_pa + 4UL, /*A=*/1, /*D=*/0);
    }

    /* ---- case 2: 4KB page W access → 成功, A+D set ---- */
    {
        /* 重置 leaf PTE (清 case 1 walker set 的 A bit), 改 R+W+U */
        memset(gpa_to_hva_offset + leaf_pt_pa, 0, 4096);
        uint32_t pte0 = PTE_V | PTE_R | PTE_W | PTE_U | ((leaf_page_pa >> 12) << 10);
        memcpy(gpa_to_hva_offset + leaf_pt_pa + 0, &pte0, 4);

        hart->priv = PRIV_U;
        EXPECT_OK(MMU_PERM_W, /*gva=*/0x00000008UL,
                  /*pa=*/leaf_page_pa + 8UL, /*A=*/1, /*D=*/1);
    }

    /* ---- case 3: V=0 PTE (level=0) → fail, cause=13 ---- */
    {
        memset(gpa_to_hva_offset + leaf_pt_pa, 0, 4096);
        /* leaf PT[0] = V=0 (整个 0; root PT[0] 仍指向 leaf_pt) */

        hart->priv = PRIV_U;
        EXPECT_FAIL(MMU_PERM_R, /*gva=*/0x00000004UL,
                    /*cause=*/CAUSE_LOAD_PAGE_FAULT);
    }

    /* ---- case 4: level=0 leaf misformatted (V=1 + R/W/X 全 0) → fail, cause=13 ---- */
    {
        memset(gpa_to_hva_offset + leaf_pt_pa, 0, 4096);
        /* level=0 必须 leaf, 但 R/W/X 全 0 在 level=0 是 misformatted (level=0 不能 pointer);
         * V=1 + R/W/X=0 在 level=0 → page fault */
        uint32_t pte0 = PTE_V | ((leaf_page_pa >> 12) << 10);
        memcpy(gpa_to_hva_offset + leaf_pt_pa + 0, &pte0, 4);

        hart->priv = PRIV_U;
        EXPECT_FAIL(MMU_PERM_R, /*gva=*/0x00000004UL,
                    /*cause=*/CAUSE_LOAD_PAGE_FAULT);
    }

    /* ---- case 5: PRIV_U + PTE.U=0 → fail, cause=13 (U 不能访问 kernel page) ---- */
    {
        memset(gpa_to_hva_offset + leaf_pt_pa, 0, 4096);
        /* leaf PT[0] = V=1 + R=1 + U=0 (kernel page); U=0 PTE 在 PRIV_U 不允许 */
        uint32_t pte0 = PTE_V | PTE_R | ((leaf_page_pa >> 12) << 10);
        memcpy(gpa_to_hva_offset + leaf_pt_pa + 0, &pte0, 4);

        hart->priv = PRIV_U;
        EXPECT_FAIL(MMU_PERM_R, /*gva=*/0x00000004UL,
                    /*cause=*/CAUSE_LOAD_PAGE_FAULT);
    }

    /* ---- case 6: 4MB superpage misaligned (level=1 leaf, PPN[0]≠0) → fail, cause=13 ---- */
    {
        memset(gpa_to_hva_offset + root_pt_pa, 0, 4096);
        /* root PT[0] = level=1 leaf (V=1, R=1, U=1; PPN[0]=非0 → misaligned superpage)
         * leaf_page_pa>>12 = 0x80102; PPN[0]=0x80102 & 0x3FF = 0x102 ≠ 0 → misaligned. */
        uint32_t pte1 = PTE_V | PTE_R | PTE_U | ((leaf_page_pa >> 12) << 10);
        memcpy(gpa_to_hva_offset + root_pt_pa + 0, &pte1, 4);

        hart->priv = PRIV_U;
        EXPECT_FAIL(MMU_PERM_R, /*gva=*/0x00000004UL,
                    /*cause=*/CAUSE_LOAD_PAGE_FAULT);
    }

    /* ---- case 7: 4MB superpage 正常 (PPN[0]=0) → 成功, 验 pa (4MB-aligned) ---- */
    {
        /* superpage_page_pa 选 GUEST_RAM_START + 4MiB = 0x80400000, 4MB 自然对齐 →
         * (super >> 12) = 0x80400; PPN[0] = 0x80400 & 0x3FF = 0x000 ✓ aligned;
         * PPN[1] = 0x80400 >> 10 = 0x201
         *
         * gva = VPN[1]=0, VPN[0]=4, offset=0xC → gva = 0x0000400C
         * 期望 pa = (PPN[1] << 22) | (VPN[0] << 12) | offset
         *        = 0x80400000 | 0x4000 | 0xC = 0x8040400C */
        const uint32_t super_page_pa = GUEST_RAM_START + 0x00400000UL;

        memset(gpa_to_hva_offset + root_pt_pa, 0, 4096);
        uint32_t pte1 = PTE_V | PTE_R | PTE_U | ((super_page_pa >> 12) << 10);
        memcpy(gpa_to_hva_offset + root_pt_pa + 0, &pte1, 4);

        hart->priv = PRIV_U;
        EXPECT_OK(MMU_PERM_R, /*gva=*/0x0000400CUL,
                  /*pa=*/0x8040400CUL, /*A=*/1, /*D=*/0);
    }

    /* ---- case 8: PT 不在 RAM (satp.PPN 指向 RAM 外) → fail, cause=5 (access fault) ---- */
    {
        /* 临时改 satp.PPN 指向 RAM 外: PPN=0x10000 → root_pa=0x10000000 (不在 [RAM_START,
         * RAM_END)); walker 第一步算 pte1_pa, ram check 失败 → access fault (跟 mmu.h doc
         * "PT 物理地址检查 — 用 RAM 区检查代替 PMP" 一致). 这条 fixture (Step 9 reject test)
         * 也会测, 但 5b sanity 先在这里 catch 一遍. */
        hart->satp = (1u << 31) | (0u << 22) | 0x10000UL;
        hart->priv = PRIV_U;

        EXPECT_FAIL(MMU_PERM_R, /*gva=*/0x00000004UL,
                    /*cause=*/CAUSE_LOAD_ACCESS_FAULT);
    }

    #undef EXPECT_OK
    #undef EXPECT_FAIL

    /* 恢复 hart 字段 */
    hart->satp          = orig_satp;
    hart->priv          = orig_priv;
    hart->trap._mstatus = orig_mstatus;

    /* 清 PT bytes (mmu_walk 已 set A/D 写回; 不清的话残留可能跨进 fixture program 区) */
    memset(gpa_to_hva_offset + root_pt_pa, 0, 4096);
    memset(gpa_to_hva_offset + leaf_pt_pa, 0, 4096);

    if (fail == 0) {
        fprintf(stderr, "[mmu_walk_test] PASS (%d/%d)\n", total - fail, total);
    } else {
        fprintf(stderr, "[mmu_walk_test] FAIL (%d/%d)\n", total - fail, total);
    }
    return fail;
}

// Debug 构建带 -fsanitize=address (含 LSan 的 exit-time 内存泄漏扫描)。
// 在 CLion Debug / gdb / strace 等 ptrace 环境下跑本程序, 必须设
//   ASAN_OPTIONS=abort_on_error=1:detect_leaks=0
// 否则 LSan 撞 ptrace 冲突会报 "LeakSanitizer has encountered a fatal error" 并 exit 1,
// 看起来像本程序的 bug 但其实是工具链限制。Run 模式(无 gdb)正常, 想查泄漏走 Run 即可。
int main(int argc, char **argv) {
    // a_01: 命令行参数 ./jit-emu <bin-or-elf-path>
    if (argc < 2) {
        fprintf(stderr, "usage: %s <bin-or-elf-path>\n", argv[0]);
        return 1;
    }

    // a_01_4 起步: decode 单元测试先跑 (decode 是纯函数, 不依赖 ram/cpu/mmu);
    // fail 直接 return 1 不让 fixture 跑 (test-driven 模式)。
    if (decode_test() != 0) {
        fprintf(stderr, "decode_test failed\n");
        return 1;
    }

    // 调用 ram_init();成功后 ram.h 暴露的全局 host_ram_base /
    // gpa_to_hva_offset 可用。报错风格见 src/dummy.txt §5。
    if (ram_init() != 0) {
        fprintf(stderr, "ram_init failed\n");
        return 1;
    }

    // 文件后缀分发(.bin / .elf / 猜)
    // a_01 简化:ELF 路径全部 stub 返回 -1(内部 fprintf "not implemented"),
    //             guess_is_elf stub silently 返回 0,实际只走 .bin。
    int err = 0;
    const char *path = argv[1];
    if (has_suffix(path, ".bin")) {
        // 注意, 后续改进中, 如果有起点参数, 则使用起点参数, 否则使用
        err = guest_load_bin(path, GUEST_RAM_START);
    } else if (has_suffix(path, ".elf")) {
        err = guest_load_elf(path);                // a_01: stub
    } else {
        if (guest_is_elf(path)) {                  // a_01: stub
            err = guest_load_elf(path);            // a_01: stub
        } else {
            err = guest_load_bin(path, GUEST_RAM_START);
        }
    }
    if (err != 0) {                                // loader 内部已 fprintf "why"
        fprintf(stderr, "load failed\n");
        return 1;
    }

    // hart 构造: misa 参数 a_01 不读, 仅作未来 misa 驱动初始化预留 (cpu.h 已 doc)。
    // tlb_t **tlb_table[4] 是 cpu_t 字段; sigjmp_buf *jmp_buf_ptr 暂留 NULL (memset 0 已置)。
    // tlb 容器 + M 共享 leaf 已由 cpu_create eager alloc;
    // [PRIV_S][asid] 的 entries 由 dispatcher 懒分配 (a_01 仍 #if 0)。
    cpu_t *hart = cpu_create(0);
    if (hart == NULL) {                            // cpu_create 内部已 fprintf "why"
        fprintf(stderr, "cpu_create failed\n");
        return 1;
    }

    // hart 字段初始化 (以后会设计函数 / 启动协议接管)。
    // (但是一定是在 dispatcher 外部, 因为 hart 的热插拔 = hart寄存器初始化 + 开始运行)
    // 注: regs[0] 是 pc (cpu_t 内 regs[0] 物理占 x0 位置, 实际存 pc; 见 cpu.h)。
    hart->regs[0] = GUEST_RAM_START;       // pc; 程序起点; 未来热插拔核时由外部参数设置
    hart->satp    = 0;                      // bare 模式 (MODE=0, ASID=0, PPN=0 全 0)
    hart->priv    = PRIV_M;                 // M 模式
    // 下面的来自参考 https://docs.kernel.org/arch/riscv/boot.html
    // hart->regs[10] = 0;  // a0 = hartid #0    (Linux RV boot protocol; x10 = a0)
    // hart->regs[11] = 0;  // a1 = device tree pointer (暂无 dtb; x11 = a1)
    // 注: regs 已被 memset 0; 以上两行显式赋值是 boot protocol 文档化, 等价于 memset 后的现状。
    //     真上 OS 跑 (hartid > 0 / 有 dtb) 时解开注释 + 改右值。

    // a_01_5_c 起 csr.c 真接 csr 读写, fixture 自己用 csrw mtvec, x.. 设 trap handler 地址,
    // main.c 不再 hard-code (a_01_5_b 时为方便测试临时设的, 现在 fixture 自包含)。
    // 启动协议: hart 创建后默认所有 trap 字段 0 (cpu_create memset 0); fixture 在执行 trap
    // 触发指令前必须先 csrw mtvec 设好, 否则 trap 跳 0 → fetch fail → triple fault → 退出。
    // 这跟真实 hart reset 后 mtvec=0 由 firmware 设的行为一致。

    // a_01_8 Step 5b decision M (临时调试 helper, Step 9 整 a_01_8 跑通后删):
    // sanity check mmu_walk 算法 — 不依赖 fixture 真跑解释器路径, 直接 build PT bytes +
    // 调 mmu_walk 验返回值。FAIL 时 return 1 不让 fixture 跑 (test-driven 模式)。
    if (mmu_walk_test(hart) != 0) {
        fprintf(stderr, "mmu_walk_test failed\n");
        cpu_destroy(hart);
        return 1;
    }

    // ------------------------------------------------------------------------
    // a_01_4: 跑 fixture, dispatcher 进 while(1) 多块循环, 每块调 interpret_one_block;
    // branch/jal/jalr 命中 is_block_boundary_inst 后退出 fetch loop, dispatcher 累加
    // count + 重做 block 1+2+3; 末尾 ecall 触发 OP_UNSUPPORTED 让解释器立刻 goto out
    // (count=0), dispatcher 看到 count==0 break, 末行 fprintf "halted: total_count=N pc="。
    // 然后回到 main 这里 fprintf 关心的寄存器值。
    //
    // 期望 (跑 tests/a_01/a01_3/01_arith_basic/out.bin, 算术 only):
    //   x1=42, x2=8, x3=50
    //   pc=0x8000000c (ecall 地址)
    //   total_count=3 (dispatcher halted 行打: 3 条算术 + 第二轮 ecall 这块 count=0 → break)
    //
    // 期望 (跑 tests/a_01/a01_3/02_arith_compressed/out.bin):
    //   x1=11, x2=5, x3=0; pc=0x80000006; total_count=3
    //
    // 期望 (跑 tests/a_01/a01_4/01_branch_loop/out.bin):
    //   x1=0 (倒计数到 0), x2=5 (循环累加 5 次)
    //   pc=0x80000014 (ecall 地址)
    //   total_count=17 (= 5 (init+iter1) + 3*4 (iter2-5 = 4 块每块 3 条) + 0 (ecall 块))
    // ------------------------------------------------------------------------
    int rc = dispatcher(hart);
    if (rc != 0) {
        fprintf(stderr, "dispatcher returned %d (fetch trap or other failure)\n", rc);
    }

    /* a_01_8 Step 6 (新): dump 范围扩到 x4-x15, 让 fixture 用的标记寄存器 (典型 x10-x15
     * 输出, x4-x7 临时) 都能看到。短期方案; 长期 user 拍 "用 0x800-0x8FF custom CSR 做
     * mtohost-style 输出" 留 csr 重组讨论一起做 (那时这个扩 dump 可缩回)。 */
    fprintf(stderr,
            "[main] result: pc=0x%08x x1=%u x2=%u x3=%u\n"
            "[main] regs (a_01_8 fixture 调试用): x4=0x%08x x5=0x%08x x6=0x%08x x7=0x%08x\n"
            "[main] regs (cont): x10=%u x11=%u x12=%u x13=%u x14=%u x15=%u\n",
            hart->regs[0],  hart->regs[1],  hart->regs[2],  hart->regs[3],
            hart->regs[4],  hart->regs[5],  hart->regs[6],  hart->regs[7],
            hart->regs[10], hart->regs[11], hart->regs[12], hart->regs[13],
            hart->regs[14], hart->regs[15]);

    // a_01_5_b: trap dump (验 trap_set_state 写字段对; 候选 A 早 return 行为下,
    // in_trap=3 时字段保留第 2 次状态作 root cause)。a_01 单 hart, 只 dump [PRIV_M] 槽
    // (a_01_5 v0 deliver_priv hard-code M; S 槽永远 0)。
    // 未来 reset 接入时, 这里换成"halt 原因分流 + reset hart" 的逻辑。
    fprintf(stderr,
            "[main] trap dump: in_trap=%u mcause=%u mtval=0x%08x mepc=0x%08x mtvec=0x%08x\n",
            hart->trap.in_trap,
            hart->trap.xcause[PRIV_M], hart->trap.xtval[PRIV_M],
            hart->trap.xepc[PRIV_M],   hart->trap.xtvec[PRIV_M]);

    // a_01_7: state dump (验 trap_set_state / OP_MRET 真切 priv + 写 mstatus.MPP/MPIE/MIE)。
    // 只 dump _mstatus 低 32 位 (mstatus 入口); 高 32 位 (mstatush) a_01 全 0, 不 dump 节省噪声。
    // priv 是当前 hart->priv (deliver_priv hard-code M 时, M 自我 trap 后 priv 仍 M; mret 后
    // 按 MPP 恢复, U/S-mode fixture 真激活时 priv 才会切非 M)。
    fprintf(stderr,
            "[main] state dump: priv=%u mstatus=0x%08x\n",
            (uint32_t)hart->priv,
            (uint32_t)(hart->trap._mstatus & 0xFFFFFFFFu));
    /* a_01_8 v01: tohost / privrd 改为 csr.c 内部直接 fprintf 流式输出 (csr_tohost_write
     * + csr_privrd_read), 不再 cpu_t 缓存, main.c 也不需要 dump (csrw/csrr 时输出已发生)。 */

    cpu_destroy(hart);
    return 0;
}
