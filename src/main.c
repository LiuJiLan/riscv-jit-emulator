//
// Created by liujilan on 2026/4/28.
// 入口。本文件做三件事:
//   1. 全局 ram_init (host mmap; ram.h 暴露的 host_ram_base / gpa_to_hva_offset 之后可用)
//   2. guest 程序加载 (按文件后缀 .bin / .elf 分发; .elf 当前 stub)
//   3. 运行一个 hart 线程 (hart 准备 + dispatcher; 当前单 hart, dispatcher 直接 main
//      调用; 多 hart 时换成 pthread_create(dispatcher_thread_fn, hart))
//
// hart 准备阶段写入启动协议: pc=GUEST_RAM_START / satp=0 (bare) / priv=PRIV_M /
// a0=mhartid / a1=dtb 占位 (仿 Linux RV boot 协议); mtvec / 其它 trap 字段保持 0,
// 由 fixture 自己 csrw 设, 跟真实 hart reset 后由 firmware 设 mtvec 同形态。
//
// 顶上 decode_test() 是 main 内嵌单测 (52 case, 纯函数 sanity), 每跑一次顺手过一遍,
// fail 立即 return 1; tests/unit/ 框架待 unit runner 真做时迁出。
//

#include "config.h"
#include "core/cpu.h"
#include "core/decode.h"
#include "core/dispatcher.h"
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

// ============================================================================
// decode 单元测试 (52 case, 主键 = 指令集 / 次键 = 指令类别)
//
// CASE 检查 7 字段 (kind/rd/rs1/rs2/imm/raw_inst/pc_step), 含 garbage 字段
// (例: I-type 的 rs2 字段实际是 imm 低 5 位; U-type 的 rs1/rs2 是 imm 中间位):
// decode 顶部按通用 (inst>>X)&0x1F 提取, 不为各类型特判, 所以期望值也照含 garbage
// 填; 单纯解码正确性下沉到这层校验。raw_inst 与 raw 一般相同, 仅 RVC 场景 raw_inst
// 是低 16 位 (高 16 位是 fetch over-read 残值, eraw 给低 16 位即可)。
//
// S/B/J 立即数解码位拼接奇怪, 必须按 spec 表对照 + 单测验证 (max+ / max- / 0 /
// off-by-one 各一个边界值)。
// ============================================================================
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

    // ---- RV 算数 (5 case, pc_step = PC_STEP_RV = 4) ----
    // I-type OP-IMM: addi x1, x0, 42 = 0x02A00093 (rs2 garbage = 0x02A & 0x1F = 10)
    CASE(0x02A00093, OP_ADDI,  /*rd*/1, /*rs1*/0, /*rs2*/10, 42, 0x02A00093, PC_STEP_RV);
    // I-type OP-IMM: addi x2, x0, 8 = 0x00800113 (rs2 garbage = 8)
    CASE(0x00800113, OP_ADDI,  /*rd*/2, /*rs1*/0, /*rs2*/8, 8, 0x00800113, PC_STEP_RV);
    // R-type OP: add x3, x1, x2 = 0x002081B3 (字段无 garbage)
    CASE(0x002081B3, OP_ADD,   /*rd*/3, /*rs1*/1, /*rs2*/2, 0, 0x002081B3, PC_STEP_RV);
    // U-type: lui x5, 1 = 0x000012B7 (imm = 0x1000, rs1/rs2 garbage = 0)
    CASE(0x000012B7, OP_LUI,   /*rd*/5, /*rs1*/0, /*rs2*/0, 0x1000, 0x000012B7, PC_STEP_RV);
    // I-type OP-IMM 负 imm: addi x6, x0, -8 = 0xFF800313 (rs2 garbage = 0xFF8 & 0x1F = 24)
    CASE(0xFF800313, OP_ADDI,  /*rd*/6, /*rs1*/0, /*rs2*/24, -8, 0xFF800313, PC_STEP_RV);

    // ---- RV branch (4 case, B-type 立即数 13-bit signed; rd 字段 garbage 复用为 imm 段) ----
    // BEQ x1, x0, +8 = 0x00008463
    //   imm=8: imm[12]=0, imm[11]=0, imm[10:5]=0, imm[4:1]=4, imm[0]=0
    //   rd garbage (bits 11:7) = imm[4:1]<<1 | imm[11] = 4<<1 | 0 = 8
    CASE(0x00008463, OP_BEQ,  /*rd*/8,  /*rs1*/1, /*rs2*/0, 8,    0x00008463, PC_STEP_RV);
    // BEQ x1, x0, -8 = 0xFE008CE3 (sign-ext 验证)
    //   imm=-8: imm[12]=1, imm[11]=1, imm[10:5]=63, imm[4:1]=12, imm[0]=0
    //   rd garbage = 12<<1 | 1 = 25
    CASE(0xFE008CE3, OP_BEQ,  /*rd*/25, /*rs1*/1, /*rs2*/0, -8,   0xFE008CE3, PC_STEP_RV);
    // BNE x0, x0, 0 = 0x00001063 (零偏移自跳, imm 全 0 验证不会"凭空"算出非 0)
    CASE(0x00001063, OP_BNE,  /*rd*/0,  /*rs1*/0, /*rs2*/0, 0,    0x00001063, PC_STEP_RV);
    // BLTU x1, x2, +4094 = 0x7E20EFE3 (B-type max+, imm[12]=0 + 其他位全 1)
    //   imm=4094: imm[12]=0, imm[11]=1, imm[10:5]=63, imm[4:1]=15, imm[0]=0
    //   rd garbage = 15<<1 | 1 = 31
    CASE(0x7E20EFE3, OP_BLTU, /*rd*/31, /*rs1*/1, /*rs2*/2, 4094, 0x7E20EFE3, PC_STEP_RV);

    // ---- RV jump (4 case, J-type imm 21-bit signed / JALR I-type imm 12-bit signed) ----
    // JAL x0, +0x100 = 0x1000006F
    //   imm=0x100=256: imm[20]=0, imm[19:12]=0, imm[11]=0, imm[10:1]=0x080, imm[0]=0
    //   rs1 garbage (bits 19:15) = imm[19:15] = 0; rs2 garbage (bits 24:20) = 0
    CASE(0x1000006F, OP_JAL,  /*rd*/0,  /*rs1*/0,  /*rs2*/0,  0x100,    0x1000006F, PC_STEP_RV);
    // JAL x0, -0x100 = 0xF01FF06F (sign-ext 验证)
    //   imm=-256: imm[20]=1, imm[19:12]=0xFF, imm[11]=1, imm[10:1]=0x380, imm[0]=0
    //   rs1 garbage = imm[19:15] = 0x1F = 31; rs2 garbage = (inst>>20)&0x1F = 0xF01 & 0x1F = 1
    CASE(0xF01FF06F, OP_JAL,  /*rd*/0,  /*rs1*/31, /*rs2*/1,  -0x100,   0xF01FF06F, PC_STEP_RV);
    // JAL x0, +max(0xFFFFE) = 0x7FFFF06F (J-type max+, imm[20]=0 + 其他位全 1)
    //   imm=0xFFFFE=1048574: imm[20]=0, imm[19:12]=0xFF, imm[11]=1, imm[10:1]=0x3FF, imm[0]=0
    //   rs1 garbage = 0x1F = 31; rs2 garbage = (inst>>20)&0x1F = 0x7FF & 0x1F = 0x1F = 31
    CASE(0x7FFFF06F, OP_JAL,  /*rd*/0,  /*rs1*/31, /*rs2*/31, 0xFFFFE,  0x7FFFF06F, PC_STEP_RV);
    // JALR x0, x1, +4 = 0x00408067 (I-type 立即数, 与 ADDI sign-ext 路径同源, 验
    //   opcode=0x67 + funct3=0 路由对; rs2 garbage = imm[4:0] = 4)
    CASE(0x00408067, OP_JALR, /*rd*/0,  /*rs1*/1,  /*rs2*/4,  4,        0x00408067, PC_STEP_RV);

    // ---- RV load/store (8 case; LOAD I-type / STORE S-type 立即数 12-bit signed) ----
    //
    // S-type 立即数 12 位拼接: imm[11:5] = inst[31:25] (高 7 位); imm[4:0] = inst[11:7]
    // (低 5 位)。LOAD 立即数复用 ADDI 同型 (sign-ext inst[31:20])。
    //
    // 选取 (max+ / max- / 0 / off-by-one 边界各 1):
    //   5 LOAD 各 1 case 验路由 (max+ 给 LW / max- 给 LHU / 0 给 LB);
    //   3 STORE 各 1 case + S-type 立即数边界 (max+ 给 SW / max- 给 SH / 0 给 SB)。

    // LB x1, 0(x2) = 0x00010083 (rd=1, rs1=2, imm=0, funct3=0; rs2 garbage = imm[4:0] = 0)
    CASE(0x00010083, OP_LB,  /*rd*/1, /*rs1*/2, /*rs2*/0,  0,    0x00010083, PC_STEP_RV);
    // LH x3, -2(x4) = 0xFFE21183 (imm=-2 = 0xFFE; rs2 garbage = imm[4:0] = 30)
    CASE(0xFFE21183, OP_LH,  /*rd*/3, /*rs1*/4, /*rs2*/30, -2,   0xFFE21183, PC_STEP_RV);
    // LW x5, 2047(x6) = 0x7FF32283 (I-type max+, imm=0x7FF=2047; rs2 garbage = 0x1F)
    CASE(0x7FF32283, OP_LW,  /*rd*/5, /*rs1*/6, /*rs2*/31, 2047, 0x7FF32283, PC_STEP_RV);
    // LBU x7, 1(x8) = 0x00144383 (rd=7, rs1=8, imm=1, funct3=4; rs2 garbage = 1)
    CASE(0x00144383, OP_LBU, /*rd*/7, /*rs1*/8, /*rs2*/1,  1,    0x00144383, PC_STEP_RV);
    // LHU x9, -2048(x10) = 0x80055483 (I-type max-, imm=-2048=0x800; rs2 garbage = 0)
    CASE(0x80055483, OP_LHU, /*rd*/9, /*rs1*/10, /*rs2*/0, -2048, 0x80055483, PC_STEP_RV);
    // SB x1, 0(x2) = 0x00110023 (rs1=2, rs2=1, imm=0, funct3=0; rd garbage = imm[4:0] = 0)
    CASE(0x00110023, OP_SB,  /*rd*/0,  /*rs1*/2,  /*rs2*/1, 0,    0x00110023, PC_STEP_RV);
    // SH x3, -4(x4) = 0xFE321E23 (imm=-4 = 0xFFC; imm[11:5]=0x7F, imm[4:0]=28; rd garbage = 28)
    CASE(0xFE321E23, OP_SH,  /*rd*/28, /*rs1*/4,  /*rs2*/3, -4,   0xFE321E23, PC_STEP_RV);
    // SW x5, 2047(x6) = 0x7E532FA3 (S-type max+, imm=0x7FF; imm[11:5]=0x3F, imm[4:0]=0x1F; rd garbage = 31)
    CASE(0x7E532FA3, OP_SW,  /*rd*/31, /*rs1*/6,  /*rs2*/5, 2047, 0x7E532FA3, PC_STEP_RV);

    // ---- RVC 算数 (10 case, 16-bit compressed, pc_step = PC_STEP_RVC = 2) ----
    //
    // RVC encoding (Spec Vol I §16): inst[1:0] != 11 → 16-bit; quadrant = inst[1:0],
    // funct3 = inst[15:13]。decode 翻译到 RV32I op_kind (RVC 是长度变化, 语义同源)。
    // raw_inst 在 RVC 时是 (uint32_t)inst (低 16 位有效, 高 16 位 0)。
    // d.rs2 / d.rd / d.rs1 由 decode_rvc 显式赋值 (不走 RV 顶部通用 (inst>>X)&0x1F),
    // 期望值无 garbage; 没用到的字段 = 0。

    // C.NOP = 0x0001 (C1 funct3=000 边界 case: rd=0+imm=0 → ADDI x0,x0,0)
    CASE(0x0001, OP_ADDI, /*rd*/0, /*rs1*/0, /*rs2*/0, 0, 0x0001, PC_STEP_RVC);
    // C.LI x2, 8 = 0x4121 (C1 funct3=010; imm6 = 0b001000 = 8; LI = ADDI rd,x0,imm)
    CASE(0x4121, OP_ADDI, /*rd*/2, /*rs1*/0, /*rs2*/0, 8, 0x4121, PC_STEP_RVC);
    // C.LI x1, -1 = 0x50FD (imm6 = 0b111111 sign-ext = -1, 验 sign-ext)
    CASE(0x50FD, OP_ADDI, /*rd*/1, /*rs1*/0, /*rs2*/0, -1, 0x50FD, PC_STEP_RVC);
    // C.ADDI x3, 5 = 0x0195 (C1 funct3=000; rd=rd+imm 形式, rs1=rd)
    CASE(0x0195, OP_ADDI, /*rd*/3, /*rs1*/3, /*rs2*/0, 5, 0x0195, PC_STEP_RVC);
    // C.ADDI4SPN x8, sp, 16 = 0x0800 (C0 funct3=000, CIW; rd'=0→x8, rs1=2(sp))
    CASE(0x0800, OP_ADDI, /*rd*/8, /*rs1*/2, /*rs2*/0, 16, 0x0800, PC_STEP_RVC);
    // C.ADDI16SP sp, 64 = 0x6121 (C1 funct3=011, rd=2; imm[6]=1 → 64)
    CASE(0x6121, OP_ADDI, /*rd*/2, /*rs1*/2, /*rs2*/0, 64, 0x6121, PC_STEP_RVC);
    // C.SLLI x4, 4 = 0x0212 (C2 funct3=000; rd!=0; shamt=4, RV32 shamt[5]=0)
    CASE(0x0212, OP_SLLI, /*rd*/4, /*rs1*/4, /*rs2*/0, 4, 0x0212, PC_STEP_RVC);
    // C.SUB x12, x9 = 0x8E05 (C1 funct3=100 sub=11 op_sel=00; rd_p=4→x12, rs2_p=1→x9)
    CASE(0x8E05, OP_SUB, /*rd*/12, /*rs1*/12, /*rs2*/9, 0, 0x8E05, PC_STEP_RVC);
    // C.MV x3, x1 = 0x8186 (C2 funct3=100 bit12=0+rs2!=0; 翻译 OP_ADD rd,x0,rs2)
    CASE(0x8186, OP_ADD, /*rd*/3, /*rs1*/0, /*rs2*/1, 0, 0x8186, PC_STEP_RVC);
    // C.LUI x18, 0x10 = 0x6941 (C1 funct3=011, rd!=0,2; nzimm = 0x10 << 12 = 0x10000)
    CASE(0x6941, OP_LUI, /*rd*/18, /*rs1*/0, /*rs2*/0, 0x10000, 0x6941, PC_STEP_RVC);

    // ---- RVC branch (1 case) ----
    // C.BEQZ x8, +4 = 0xC011 (C1 funct3=110, CB; rs1'=0→x8, rs2=x0, offset=4)
    CASE(0xC011, OP_BEQ, /*rd*/0, /*rs1*/8, /*rs2*/0, 4, 0xC011, PC_STEP_RVC);

    // ---- RVC jump (2 case) ----
    // C.J +4 = 0xA011 (C1 funct3=101, CJ; rd=x0, imm=4)
    CASE(0xA011, OP_JAL, /*rd*/0, /*rs1*/0, /*rs2*/0, 4, 0xA011, PC_STEP_RVC);
    // C.JR x1 = 0x8082 (C2 funct3=100 bit12=0+rs1!=0+rs2=0; rd=x0, imm=0)
    CASE(0x8082, OP_JALR, /*rd*/0, /*rs1*/1, /*rs2*/0, 0, 0x8082, PC_STEP_RVC);

    // ---- RVC load/store (4 case) ----
    // C.LW x10, 0(x8) = 0x4008 (C0 funct3=010, CL; rs1'=0→x8, rd'=2→x10, uimm=0)
    CASE(0x4008, OP_LW, /*rd*/10, /*rs1*/8, /*rs2*/0, 0, 0x4008, PC_STEP_RVC);
    // C.SW x9, 0(x8) = 0xC004 (C0 funct3=110, CS; rs1'=0→x8, rs2'=1→x9, uimm=0)
    CASE(0xC004, OP_SW, /*rd*/0, /*rs1*/8, /*rs2*/9, 0, 0xC004, PC_STEP_RVC);
    // C.LWSP x12, 16(sp) = 0x4642 (C2 funct3=010, CI uimm; rd=12, rs1=2(sp), uimm=16)
    CASE(0x4642, OP_LW, /*rd*/12, /*rs1*/2, /*rs2*/0, 16, 0x4642, PC_STEP_RVC);
    // C.SWSP x11, 16(sp) = 0xC82E (C2 funct3=110, CSS; rs1=2(sp), rs2=11, uimm=16)
    CASE(0xC82E, OP_SW, /*rd*/0, /*rs1*/2, /*rs2*/11, 16, 0xC82E, PC_STEP_RVC);

    // ---- CSR (7 case, I-type SYSTEM 6 变体, csr_addr 边界) ----
    //
    // 编码: csr_addr inst[31:20] / rs1_or_zimm inst[19:15] / funct3 inst[14:12]
    //   (001=RW 010=RS 011=RC 101=RWI 110=RSI 111=RCI) / rd inst[11:7] / opcode 0x73。
    // decoded_inst_t 字段约定 (decode.h enum 段已 doc):
    //   d.imm   = csr 12-bit addr (无符号扩展, 高 20 位 0)
    //   d.rs1   = RW/RS/RC 时是 rs1 寄存器号; RWI/RSI/RCI 时是 5-bit zimm 数值
    //             (interpreter 不查 regs 直接用) — 字段共用同一位置 (Spike / QEMU 同做法)
    //   d.rs2   = garbage = (inst>>20) & 0x1F = csr_addr & 0x1F (csr decode 不动 d.rs2)
    //   d.pc_step = PC_STEP_RV (csr 不是 control flow, fetch loop +4; 但是硬边界,
    //               由 is_block_boundary_inst 让 fetch loop 退出)。

    // CSRRW x1, mtvec, x2 = 0x305110F3
    CASE(0x305110F3, OP_CSRRW,  /*rd*/1, /*rs1*/2,  /*rs2*/5,  0x305, 0x305110F3, PC_STEP_RV);
    // CSRRS x3, mstatus, x4 = 0x300221F3
    CASE(0x300221F3, OP_CSRRS,  /*rd*/3, /*rs1*/4,  /*rs2*/0,  0x300, 0x300221F3, PC_STEP_RV);
    // CSRRC x0, mcause, x5 = 0x3422B073 (rd=x0 路径)
    CASE(0x3422B073, OP_CSRRC,  /*rd*/0, /*rs1*/5,  /*rs2*/2,  0x342, 0x3422B073, PC_STEP_RV);
    // CSRRWI x6, mepc, 0 = 0x34105373 (zimm=0 边界, 验 zimm=0 不真写规则)
    CASE(0x34105373, OP_CSRRWI, /*rd*/6, /*rs1*/0,  /*rs2*/1,  0x341, 0x34105373, PC_STEP_RV);
    // CSRRSI x7, mtval, 31 = 0x343FE3F3 (zimm=31 max)
    CASE(0x343FE3F3, OP_CSRRSI, /*rd*/7, /*rs1*/31, /*rs2*/3,  0x343, 0x343FE3F3, PC_STEP_RV);
    // CSRRCI x0, 0xFFF, 31 = 0xFFFFF073 (csr_addr=0xFFF 边界, zimm=31 max, rd=x0)
    CASE(0xFFFFF073, OP_CSRRCI, /*rd*/0, /*rs1*/31, /*rs2*/31, 0xFFF, 0xFFFFF073, PC_STEP_RV);
    // CSRRW x0, 0x000, x0 = 0x00001073 (csr_addr=0x000 边界, rs1=x0, rd=x0; 全零路径)
    CASE(0x00001073, OP_CSRRW,  /*rd*/0, /*rs1*/0,  /*rs2*/0,  0x000, 0x00001073, PC_STEP_RV);

    // ---- SYSTEM (7 case: ECALL/EBREAK/MRET/SRET + SFENCE.VMA×2 + C.EBREAK) ----
    //
    // 共享 SYSTEM opcode 0x73 + funct3=0, 由 imm[11:0] (= inst[31:20]) 区分:
    //   imm=0x000 → ECALL  / imm=0x001 → EBREAK
    //   imm=0x102 → SRET   / imm=0x302 → MRET
    //   funct7=0x09 → SFENCE.VMA (rs1=vaddr_reg, rs2=asid_reg; 共存路径)
    // d.rs2 garbage = (inst>>20) & 0x1F = imm 低 5 位; pc_step:
    //   ECALL/EBREAK/MRET/SRET → PC_STEP_NONE (case 自描述 pc, fetch loop 不 += pc_step)
    //   SFENCE.VMA            → PC_STEP_RV   (sfence 顺序 +4, 由 is_block_boundary_inst 让块结束)
    //   C.EBREAK              → PC_STEP_NONE (跟 32-bit EBREAK 同, RVC 长度由 case 自记)

    // ECALL = 0x00000073 (imm=0x000)
    CASE(0x00000073, OP_ECALL,  /*rd*/0, /*rs1*/0, /*rs2*/0, 0x000, 0x00000073, PC_STEP_NONE);
    // EBREAK = 0x00100073 (imm=0x001)
    CASE(0x00100073, OP_EBREAK, /*rd*/0, /*rs1*/0, /*rs2*/1, 0x001, 0x00100073, PC_STEP_NONE);
    // MRET = 0x30200073 (imm=0x302)
    CASE(0x30200073, OP_MRET,   /*rd*/0, /*rs1*/0, /*rs2*/2, 0x302, 0x30200073, PC_STEP_NONE);
    // SRET = 0x10200073 (imm=0x102; rs2 garbage = imm[4:0] = 2)
    CASE(0x10200073, OP_SRET,   /*rd*/0, /*rs1*/0, /*rs2*/2, 0x102, 0x10200073, PC_STEP_NONE);
    // sfence.vma x0, x0 = 0x12000073 (全清形态; funct7=0x09, rs2=0; imm = 0x09<<5 | 0 = 0x120)
    CASE(0x12000073, OP_SFENCE_VMA, /*rd*/0, /*rs1*/0, /*rs2*/0, 0x120, 0x12000073, PC_STEP_RV);
    // sfence.vma x1, x2 = 0x12208073 (普通形态, 验 d.rs1/d.rs2 字段透传; imm = 0x09<<5 | 2 = 0x122)
    CASE(0x12208073, OP_SFENCE_VMA, /*rd*/0, /*rs1*/1, /*rs2*/2, 0x122, 0x12208073, PC_STEP_RV);
    // C.EBREAK = 0x9002 (C2 funct3=100 bit12=1+rs1=0+rs2=0)
    CASE(0x9002, OP_EBREAK, /*rd*/0, /*rs1*/0, /*rs2*/0, 0, 0x9002, PC_STEP_NONE);

    #undef CASE

    if (fail == 0) {
        fprintf(stderr, "[decode_test] PASS (%d/%d)\n", total - fail, total);
    } else {
        fprintf(stderr, "[decode_test] FAIL (%d/%d)\n", total - fail, total);
    }
    return fail;
}

// Debug 构建带 -fsanitize=address (含 LSan 的 exit-time 内存泄漏扫描)。
// 在 CLion Debug / gdb / strace 等 ptrace 环境下跑本程序, 必须设
//   ASAN_OPTIONS=abort_on_error=1:detect_leaks=0
// 否则 LSan 撞 ptrace 冲突会报 "LeakSanitizer has encountered a fatal error" 并 exit 1,
// 看起来像本程序的 bug 但其实是工具链限制。Run 模式(无 gdb)正常, 想查泄漏走 Run 即可。
int main(int argc, char **argv) {
    // 命令行参数 ./jit-emu <bin-or-elf-path>
    if (argc < 2) {
        fprintf(stderr, "usage: %s <bin-or-elf-path>\n", argv[0]);
        return 1;
    }

    // decode 单测先跑 (decode 是纯函数, 不依赖 ram/cpu/mmu); fail 直接 return 1 不让
    // fixture 跑 (test-driven 模式)。
    if (decode_test() != 0) {
        fprintf(stderr, "decode_test failed\n");
        return 1;
    }

    // 全局 ram_init: 调用后 ram.h 暴露的 host_ram_base / gpa_to_hva_offset 可用。
    // 报错风格见 src/dummy.txt §5。
    if (ram_init() != 0) {
        fprintf(stderr, "ram_init failed\n");
        return 1;
    }

    // 文件后缀分发 (.bin / .elf / 猜): ELF 路径全部 stub 返回 -1 (内部 fprintf
    // "not implemented"), guess_is_elf stub 静默返回 0, 实际只走 .bin。
    int err = 0;
    const char *path = argv[1];
    if (has_suffix(path, ".bin")) {
        // 后续如果有起点参数, 用起点参数, 否则用 GUEST_RAM_START。
        err = guest_load_bin(path, GUEST_RAM_START);
    } else if (has_suffix(path, ".elf")) {
        err = guest_load_elf(path);
    } else {
        if (guest_is_elf(path)) {
            err = guest_load_elf(path);
        } else {
            err = guest_load_bin(path, GUEST_RAM_START);
        }
    }
    if (err != 0) {  // loader 内部已 fprintf "why"
        fprintf(stderr, "load failed\n");
        return 1;
    }

    // hart 构造: misa 参数当前未读取, 仅作 misa 驱动初始化预留 (cpu.h 已 doc)。
    // tlb 容器 + M 共享 leaf 已由 cpu_create eager alloc; [PRIV_S][asid] 的
    // entries 由 dispatcher 懒分配 (当前 #if 0)。
    cpu_t *hart = cpu_create(/*misa*/0, /*mhartid*/0);
    if (hart == NULL) {  // cpu_create 内部已 fprintf "why"
        fprintf(stderr, "cpu_create failed\n");
        return 1;
    }

    // hart 字段初始化 (启动协议; 必须在 dispatcher 外, 因为 hart 热插拔 = 寄存器
    // 初始化 + 开始运行)。regs[0] 在 cpu_t 内物理占 x0 位置, 但实际存 pc (见 cpu.h)。
    // 启动状态参考 https://docs.kernel.org/arch/riscv/boot.html
    hart->regs[0] = GUEST_RAM_START;       // pc; 程序起点; 未来热插拔核时由外部参数设置
    hart->satp    = 0;                      // bare 模式 (MODE=0, ASID=0, PPN=0 全 0)
    hart->priv    = PRIV_M;                 // M 模式
    hart->regs[10] = hart->per_hart_info.mhartid;  // a0 = hartid (Linux RV boot; x10 = a0)
    // hart->regs[11] = 0;  // a1 = device tree pointer (暂无 dtb; x11 = a1; 真做时改右值)

    // dispatcher() 跑一次 (返回时 hart 已 halt; halt 状态由 hart->trap.in_trap 表达,
    // 位段编码见 dispatcher.c 末尾 in_trap 位段编码段)。后续 dump 段印 reg / trap / state
    // 给 fixture 肉眼对照 (期望见各 fixture 目录注释)。
    dispatcher(hart);

    /* dump 格式:
     *   - reg 分 [reg dec] + [reg hex] 两段, 每段 x1-x31 (x0 跳过, 占位 pc)
     *   - ABI 标注 xN(abi), space padding 到 9 char 宽度对齐 (x8(s0/fp) 9 char 不单独优化)
     *   - 每行 4 reg (decimal %11u / hex 0x%08x), 行头 \t
     *   - pc 放 state dump 末行, hex 显示 */
    fprintf(stderr,
            "[main] reg dec:\n"
            "\tx1(ra)    = %11u  |  x2(sp)    = %11u  |  x3(gp)    = %11u  |  x4(tp)    = %11u\n"
            "\tx5(t0)    = %11u  |  x6(t1)    = %11u  |  x7(t2)    = %11u  |  x8(s0/fp) = %11u\n"
            "\tx9(s1)    = %11u  |  x10(a0)   = %11u  |  x11(a1)   = %11u  |  x12(a2)   = %11u\n"
            "\tx13(a3)   = %11u  |  x14(a4)   = %11u  |  x15(a5)   = %11u  |  x16(a6)   = %11u\n"
            "\tx17(a7)   = %11u  |  x18(s2)   = %11u  |  x19(s3)   = %11u  |  x20(s4)   = %11u\n"
            "\tx21(s5)   = %11u  |  x22(s6)   = %11u  |  x23(s7)   = %11u  |  x24(s8)   = %11u\n"
            "\tx25(s9)   = %11u  |  x26(s10)  = %11u  |  x27(s11)  = %11u  |  x28(t3)   = %11u\n"
            "\tx29(t4)   = %11u  |  x30(t5)   = %11u  |  x31(t6)   = %11u\n",
            hart->regs[1],  hart->regs[2],  hart->regs[3],  hart->regs[4],
            hart->regs[5],  hart->regs[6],  hart->regs[7],  hart->regs[8],
            hart->regs[9],  hart->regs[10], hart->regs[11], hart->regs[12],
            hart->regs[13], hart->regs[14], hart->regs[15], hart->regs[16],
            hart->regs[17], hart->regs[18], hart->regs[19], hart->regs[20],
            hart->regs[21], hart->regs[22], hart->regs[23], hart->regs[24],
            hart->regs[25], hart->regs[26], hart->regs[27], hart->regs[28],
            hart->regs[29], hart->regs[30], hart->regs[31]);

    fprintf(stderr,
            "[main] reg hex:\n"
            "\tx1(ra)    = 0x%08x  |  x2(sp)    = 0x%08x  |  x3(gp)    = 0x%08x  |  x4(tp)    = 0x%08x\n"
            "\tx5(t0)    = 0x%08x  |  x6(t1)    = 0x%08x  |  x7(t2)    = 0x%08x  |  x8(s0/fp) = 0x%08x\n"
            "\tx9(s1)    = 0x%08x  |  x10(a0)   = 0x%08x  |  x11(a1)   = 0x%08x  |  x12(a2)   = 0x%08x\n"
            "\tx13(a3)   = 0x%08x  |  x14(a4)   = 0x%08x  |  x15(a5)   = 0x%08x  |  x16(a6)   = 0x%08x\n"
            "\tx17(a7)   = 0x%08x  |  x18(s2)   = 0x%08x  |  x19(s3)   = 0x%08x  |  x20(s4)   = 0x%08x\n"
            "\tx21(s5)   = 0x%08x  |  x22(s6)   = 0x%08x  |  x23(s7)   = 0x%08x  |  x24(s8)   = 0x%08x\n"
            "\tx25(s9)   = 0x%08x  |  x26(s10)  = 0x%08x  |  x27(s11)  = 0x%08x  |  x28(t3)   = 0x%08x\n"
            "\tx29(t4)   = 0x%08x  |  x30(t5)   = 0x%08x  |  x31(t6)   = 0x%08x\n",
            hart->regs[1],  hart->regs[2],  hart->regs[3],  hart->regs[4],
            hart->regs[5],  hart->regs[6],  hart->regs[7],  hart->regs[8],
            hart->regs[9],  hart->regs[10], hart->regs[11], hart->regs[12],
            hart->regs[13], hart->regs[14], hart->regs[15], hart->regs[16],
            hart->regs[17], hart->regs[18], hart->regs[19], hart->regs[20],
            hart->regs[21], hart->regs[22], hart->regs[23], hart->regs[24],
            hart->regs[25], hart->regs[26], hart->regs[27], hart->regs[28],
            hart->regs[29], hart->regs[30], hart->regs[31]);

    // trap dump (M+S 两槽): in_trap=3 时字段保留 double fault 第二次状态作 root cause;
    // S 槽用于 medeleg delegate 路径验证 (medeleg=1 → trap deliver S, M 槽不写)。
    // 未来 reset 接入时, 这里换成"halt 原因分流 + reset hart" 逻辑。
    fprintf(stderr,
            "[main] trap dump (M): in_trap=%u mcause=%u mtval=0x%08x mepc=0x%08x mtvec=0x%08x\n",
            hart->trap.in_trap,
            hart->trap.xcause[PRIV_M], hart->trap.xtval[PRIV_M],
            hart->trap.xepc[PRIV_M],   hart->trap.xtvec[PRIV_M]);
    fprintf(stderr,
            "[main] trap dump (S): scause=%u stval=0x%08x sepc=0x%08x stvec=0x%08x\n",
            hart->trap.xcause[PRIV_S], hart->trap.xtval[PRIV_S],
            hart->trap.xepc[PRIV_S],   hart->trap.xtvec[PRIV_S]);

    // state dump: pc + priv + mstatus 低 32 位 (mstatush 当前全 0, 省略)。
    // priv 当前为 hart->priv (mret 按 MPP 恢复, U/S-mode fixture 激活时 priv 才会切非 M)。
    fprintf(stderr,
            "[main] state dump: pc=0x%08x  priv=%u  mstatus=0x%08x\n",
            hart->regs[0],
            (uint32_t)hart->priv,
            (uint32_t)(hart->trap._mstatus & 0xFFFFFFFFu));
    /* tohost / privrd 在 csr.c 内 csrw/csrr 时直接 fprintf 流式输出, 不缓存到 cpu_t,
     * main.c 不需要 dump。 */

    cpu_destroy(hart);
    return 0;
}
