//
// Created by liujilan on 2026/4/28.
// 入口。本文件按 reset 三层 lifecycle 组织:
//
//   POR (Power-On Reset) — 进程启动一次
//     ram_init / clint_init (atomic 字段 + bus 注册, **不发线程**) /
//     plic_init (字段 + ctx_map 全 -1 + plic_ctx_eip / plic_pending_bitmap_cache
//     atomic 清 0 + rwlock_init + bus 注册; PLIC 是 "monitor 但无辅助线程",
//     hot path 走 atomic 字段, set/clear 同步 wrlock; 详 plic.h 顶段 +
//     trade_off_log §T.6 演进 trail) /
//     test_dev_init (bus 注册; 无状态 + 无锁 + 无线程, fanout 调 plic.device_set/
//     clear_pending; 详 device/test_dev.h 顶段) /
//     uart_init (bus 注册 + mutex_init; **不发线程** — reader 线程由下方 uart_start_
//     reader_thread 显式起, 跟 clint 同形态; 详 device/uart.h 顶段) /
//     virtio_blk_init (--blk 路径下 open image + 双 mutex/双 cond_init + bus 注册;
//     blk_path==NULL 时退化为不存在; **不发线程** — io_worker 由下方
//     virtio_blk_start_io_worker_thread 显式起; 详 device/virtio_blk.h 顶段) /
//     cpu_create (内部已写硬件 reset 默认状态)
//     SRS=1 / SDS=1 (runtime.c 定义初值兜底 + 这里显式重设, lifecycle 可读)
//     runtime_install_signal_handlers (SIGINT/SIGTERM/SIGHUP sigaction; handler 内
//     atomic 写 external_signal_no + SDS bit EXTERNAL_SIGNAL; 装在 atomic_store
//     SRS/SDS=0 之后, 线程 spawn 之前; 详 runtime.h "external signal handler" 节) /
//     clint_start_timer_thread (main 起 timer 辅助线程; 跨 system reset 一直跑, 随
//     SDS 才退; dummy.txt §12 谁 spawn 谁 join) /
//     uart_start_rx_thread (main 起 RX reader 线程; 受 SDS 控制, 跨 system reset
//     一直跑; 详 device/uart.h 顶段) /
//     virtio_blk_start_io_worker_thread (--blk 路径下 spawn worker 异步 drain
//     avail ring + pread/pwrite; 退化路径不 spawn)
//
//   System reset — main while 每 iter
//     cpu_reset / clint_reset (mtimecmp/msip 清, mtime/timer 不动) /
//     plic_reset (device_line/claimed 清, plic_ctx_map 不动) /
//     test_dev_reset (no-op, 无状态) /
//     uart_reset (8 寄存器 + RX FIFO 清, lock + reader_thread 不动) /
//     virtio_blk_reset (寄存器 + InterruptStatus 清, image_fd + work queue +
//                       io_worker_thread 不动; 退化路径 no-op) /
//     dispatcher(hart) / SR-vs-shutdown 判断 (当前简化恒 shutdown; if(0) 骨架
//     占位预留真 SR 路径)
//
// (取消 "HART reset" 项目自定义概念: spec 只规定 reset 后状态, 不规定何时 reset
// hart; 所有 hart-internal 不可恢复路径都归并为 system reset, 受 SRS 控制。)
//
// 顶上 decode_test() 是 main 内嵌单测 (52 case, 纯函数 sanity), 每跑一次顺手过一遍,
// fail 立即 return 1; tests/unit/ 框架待 unit runner 真做时迁出。
//

#include "config.h"
#include "debug.h"              // debug_flush_local_trace (hart_exec_run 末尾兜底)
#include "device/test_dev.h"
#include "device/uart.h"
#include "device/virtio_blk.h"
#include "core/cpu.h"
#include "core/decode.h"
#include "core/dispatcher.h"
#include "core/wfi.h"          // wfi_init / wfi_destroy (WFI 唤醒框架; wfi_kick_all 不调用, 见 wfi.h doc)
#include "api/jit_api.h"       // jit_init / jit_shutdown (b_01 T3 JIT 子系统 lifecycle; 跟 lrsc_init/destroy 对偶位置)
#include "isa/lrsc.h"          // lrsc_init / lrsc_destroy (A 扩展 reservation 数据结构; bucket 锁数组 cap 配对)
#include "loader.h"
#include "platform/clint.h"
#include "platform/plic.h"
#include "platform/ram.h"
#include "riscv.h"
#include "runtime.h"

#include <pthread.h>    // pthread_create / pthread_join (s3 per-hart dispatcher 线程)
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>     // strtoul (--load ADDR= 解析)
#include <string.h>
#include <time.h>       // clock_gettime / struct timespec (main lifecycle 总耗时)

// has_suffix: 判 *s 是否以 *suffix 结尾 (大小写敏感)。--bios 三层 dispatch 用
// (后缀 .elf / .bin 显式优先; 详 main 内 --bios 调用点 doc)。
static int has_suffix(const char *s, const char *suffix) {
    size_t ns = strlen(s);
    size_t nsuf = strlen(suffix);
    if (ns < nsuf) { return 0; }
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
                    "[decode_test] FAIL raw=0x%08x: kind=%d rd=%u rs1=%u rs2=%u imm=%d raw_inst=0x%x pc_step=%u" EOL, \
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

    // ---- RV32M (10 case: 8 happy + 2 funct7 silent miscompile reject) ----
    //
    // R-type opcode 0x33 funct7=0x01 dispatch by funct3 (小 plan A.1):
    //   funct3=0 MUL    / 1 MULH  / 2 MULHSU / 3 MULHU
    //   funct3=4 DIV    / 5 DIVU  / 6 REM    / 7 REMU
    // imm = 0 (R-type 无 imm); pc_step = PC_STEP_RV (默认; M ext 非 boundary)。
    //
    // 编码: funct7[31:25]=0x01 / rs2[24:20] / rs1[19:15] / funct3[14:12] / rd[11:7] / opcode=0x33
    // 选 rd/rs1/rs2 不同寄存器号验字段透传不串。

    // MUL    x1, x2, x3 = 0x023100B3
    CASE(0x023100B3, OP_MUL,    /*rd*/1,  /*rs1*/2,  /*rs2*/3,  0, 0x023100B3, PC_STEP_RV);
    // MULH   x4, x5, x6 = 0x02629233
    CASE(0x02629233, OP_MULH,   /*rd*/4,  /*rs1*/5,  /*rs2*/6,  0, 0x02629233, PC_STEP_RV);
    // MULHSU x7, x8, x9 = 0x029423B3
    CASE(0x029423B3, OP_MULHSU, /*rd*/7,  /*rs1*/8,  /*rs2*/9,  0, 0x029423B3, PC_STEP_RV);
    // MULHU  x10, x11, x12 = 0x02C5B533
    CASE(0x02C5B533, OP_MULHU,  /*rd*/10, /*rs1*/11, /*rs2*/12, 0, 0x02C5B533, PC_STEP_RV);
    // DIV    x13, x14, x15 = 0x02F746B3
    CASE(0x02F746B3, OP_DIV,    /*rd*/13, /*rs1*/14, /*rs2*/15, 0, 0x02F746B3, PC_STEP_RV);
    // DIVU   x16, x17, x18 = 0x0328D833
    CASE(0x0328D833, OP_DIVU,   /*rd*/16, /*rs1*/17, /*rs2*/18, 0, 0x0328D833, PC_STEP_RV);
    // REM    x19, x20, x21 = 0x035A69B3
    CASE(0x035A69B3, OP_REM,    /*rd*/19, /*rs1*/20, /*rs2*/21, 0, 0x035A69B3, PC_STEP_RV);
    // REMU   x22, x23, x24 = 0x038BFB33
    CASE(0x038BFB33, OP_REMU,   /*rd*/22, /*rs1*/23, /*rs2*/24, 0, 0x038BFB33, PC_STEP_RV);

    // funct7 silent miscompile reject 2 case (修复验证):
    //   - funct7=0x02 + funct3=0: 之前老代码 funct3=0 不查 funct7 → 当 OP_ADD 跑乱; 现 OP_UNSUPPORTED。
    //   - funct7=0x20 + funct3=2: 之前 funct3=2 老代码不查 funct7 → 当 OP_SLT 跑乱; 现 OP_UNSUPPORTED
    //                              (RV ISA Table 24.1: funct7=0x20 仅 funct3=0/5 合法)。
    // 期望 d.kind=OP_UNSUPPORTED; 其他字段 (rd/rs1/rs2) 仍按 R-type 通用编码位置填 (decode 顶部统一提取);
    // d.imm=0, d.pc_step=PC_STEP_RV (decode 顶部默认, OP_UNSUPPORTED 路径不动)。
    CASE(0x043100B3, OP_UNSUPPORTED, /*rd*/1, /*rs1*/2, /*rs2*/3, 0, 0x043100B3, PC_STEP_RV);
    CASE(0x403120B3, OP_UNSUPPORTED, /*rd*/1, /*rs1*/2, /*rs2*/3, 0, 0x403120B3, PC_STEP_RV);

    #undef CASE

    if (fail == 0) {
        fprintf(stderr, "[decode_test] PASS (%d/%d)" EOL, total - fail, total);
    } else {
        fprintf(stderr, "[decode_test] FAIL (%d/%d)" EOL, total - fail, total);
    }
    return fail;
}


// hart_exec_run — per-hart pthread routine (file-static; 适配 pthread void *(void *)
// ABI ↔ dispatcher void(cpu_t *) 签名). cpu_t 是 hart 的"数据部分", 本函数是
// "执行部分" — 即每个 hart 启动一个本 routine 的 pthread, 内部调一次 dispatcher
// 跑到 hart halt (dispatcher 内部 while(SRS==0) 自带 hart 主循环 + sigsetjmp 落点)。
// 跟 timer_run / io_worker_run / uart_reader_run / uart_tx_drain_run 同 file-static
// wrapper 体例 (project 里所有 pthread routine 都走这层 ABI 适配)。
static void *hart_exec_run(void *arg) {
    cpu_t *hart = (cpu_t *)arg;
    hartid_self = hart->hartid;  /* 给 debug_flush_local_trace 拿来打 "[hart N trace] " prefix */
    dispatcher(hart);
    debug_flush_local_trace();   /* 兜底 flush (dispatcher 内 DEBUG_NEWLINE 已 flush; 空 buffer no-op) */
    return NULL;
}

// Debug 构建带 -fsanitize=address (含 LSan 的 exit-time 内存泄漏扫描)。
// 在 CLion Debug / gdb / strace 等 ptrace 环境下跑本程序, 必须设
//   ASAN_OPTIONS=abort_on_error=1:detect_leaks=0
// 否则 LSan 撞 ptrace 冲突会报 "LeakSanitizer has encountered a fatal error" 并 exit 1,
// 看起来像本程序的 bug 但其实是工具链限制。Run 模式(无 gdb)正常, 想查泄漏走 Run 即可。
int main(int argc, char **argv) {
    // 程序总耗时起点 (CLOCK_MONOTONIC 跟 clint.c timer_run 同 clock 源, 一致;
    // 不用 CLOCK_REALTIME 避免 wall clock 跳变干扰)。
    // 终点在 main return 0 之前; 失败 return 1 路径不打耗时 (无意义且 fail 已 fprintf)。
    struct timespec t_start;
    clock_gettime(CLOCK_MONOTONIC, &t_start);

    // ------------------------------------------------------------------------
    // 命令行参数解析:
    //
    //   --bios FILE       简便 alias 三层 dispatch: 后缀 .elf → load_elf;
    //                                            后缀 .bin → load_bin GUEST_RAM_START;
    //                                            其他/无后缀 → magic 探测 (guest_is_elf)
    //                                            命中走 load_elf, 否则 load_bin。
    //                                            后缀显式优先于 magic, 防 raw 文件首字节
    //                                            碰巧 0x7F 'E' 'L' 'F' 被误判 ELF.
    //   --load FILE       无 ADDR → guest_load_elf (按 ELF p_paddr 加载)
    //   --load ADDR=FILE  有 ADDR → guest_load_bin (raw 到 ADDR, 不解 ELF 头)
    //   --blk FILE        virtio-blk image (空 = 模块退化为不存在)
    //
    // 语义跟 QEMU `-device loader,file=FILE[,addr=ADDR]` addr 有/无两路 1:1;
    // 语法收敛双横不引 key=value 逗号分隔解析器 (workspace.xml python 脚本
    // 以后真升 QEMU 单横语法也就是脚本的事).
    //
    // 解析顺序: argv 顺序攒 load_ops 列表 + blk_path; 实际加载推迟到 ram_init
    // 之后. 后写覆盖按 QEMU loader device 同语义 (memcpy 不查冲突).
    //
    // backward compat: 不留无前缀 argv[1]; 一刀切要求 --bios / --load 显式
    // (workspace.xml 由 user python 脚本批量改).
    // ------------------------------------------------------------------------
    enum { LOAD_OP_MAX = 8 };  /* 典型 fixture 1-2 个 load; 8 富余 */
    struct { uint64_t addr; const char *file; int as_elf; } load_ops[LOAD_OP_MAX];
    int load_count = 0;
    const char *blk_path = NULL;
    // n_harts 是 cpu.{c,h} 的全局 (core/cpu.h extern; 默认 1)。--smp 解析进临时
    // smp_req 做语法+范围校验, 通过后写全局 n_harts (见 while 前)。不再用局部
    // 变量 (避免 shadow 全局)。
    unsigned smp_req = 1;  // --smp N 解析结果 (语法/范围校验用; 校验通过后写全局)

    for (int i = 1; i < argc; i++) {
        const char *arg = argv[i];

        if (strcmp(arg, "--bios") == 0) {
            if (++i >= argc) {
                fprintf(stderr, "--bios needs FILE" EOL);
                return 1;
            }
            if (load_count >= LOAD_OP_MAX) {
                fprintf(stderr, "too many load ops (max %d)" EOL, LOAD_OP_MAX);
                return 1;
            }
            const char *file = argv[i];
            load_ops[load_count].file = file;
            // 三层 dispatch:
            //   1. 后缀 .elf 显式 → load_elf (即使 magic 不命中也强走 load_elf; 文件
            //      坏掉报错让 user 知道, 不被 magic fallback 掩盖)
            //   2. 后缀 .bin 显式 → load_bin GUEST_RAM_START (即使内容碰巧 0x7F 'E'
            //      'L' 'F' 也按 raw 处理; user 显式意图优先于 magic 探测)
            //   3. 其他后缀 / 无后缀 → guest_is_elf magic 探测 fall back
            // guest_is_elf 开文件失败 silent 返 0, 报错延后到 guest_load_bin/elf 内
            // open 失败 (一致的 stderr 报错路径)。
            if (has_suffix(file, ".elf")) {
                load_ops[load_count].as_elf = 1;
                load_ops[load_count].addr = 0;
            } else if (has_suffix(file, ".bin")) {
                load_ops[load_count].as_elf = 0;
                load_ops[load_count].addr = GUEST_RAM_START;
            } else if (guest_is_elf(file)) {
                load_ops[load_count].as_elf = 1;
                load_ops[load_count].addr = 0;
            } else {
                load_ops[load_count].as_elf = 0;
                load_ops[load_count].addr = GUEST_RAM_START;
            }
            load_count++;

        } else if (strcmp(arg, "--load") == 0) {
            if (++i >= argc) {
                fprintf(stderr, "--load needs FILE or ADDR=FILE" EOL);
                return 1;
            }
            if (load_count >= LOAD_OP_MAX) {
                fprintf(stderr, "too many load ops (max %d)" EOL, LOAD_OP_MAX);
                return 1;
            }
            const char *spec = argv[i];
            const char *eq = strchr(spec, '=');
            if (eq == NULL) {
                /* 无 ADDR → ELF 路径 */
                load_ops[load_count].file = spec;
                load_ops[load_count].as_elf = 1;
                load_ops[load_count].addr = 0;
            } else {
                /* ADDR=FILE → raw 路径; strtoul base=0 自动识别 0x / 10 进制 */
                char *end = NULL;
                unsigned long addr = strtoul(spec, &end, 0);
                if (end != eq || end == spec) {
                    fprintf(stderr, "--load: bad ADDR in '%s'" EOL, spec);
                    return 1;
                }
                load_ops[load_count].file = eq + 1;
                load_ops[load_count].as_elf = 0;
                load_ops[load_count].addr = (uint64_t)addr;
            }
            load_count++;

        } else if (strcmp(arg, "--blk") == 0) {
            if (++i >= argc) {
                fprintf(stderr, "--blk needs FILE" EOL);
                return 1;
            }
            blk_path = argv[i];

        } else if (strcmp(arg, "--smp") == 0) {
            // 运行期 hart 数 (QEMU -smp 同义); 不给 = 默认 1。校验 1..MAX_HARTS,
            // 越界提前退出 (dummy.txt §5)。解析进 smp_req; 真写全局 n_harts 在
            // while 前。多 hart 时 main 起 n_harts 个 pthread 各跑 dispatcher
            // (详 main while 内 spawn/join + hart_exec_run wrapper)。
            if (++i >= argc) {
                fprintf(stderr, "--smp needs N (1..%u)" EOL, MAX_HARTS);
                return 1;
            }
            // strtoul base=0: 接受十进制 / 0x 十六进制; end 须停在 '\0' (整串是数字)。
            char *end = NULL;
            unsigned long v = strtoul(argv[i], &end, 0);
            smp_req = (unsigned)v;
            // 范围用 smp_req 判; v != smp_req 兜底 64→32 截断把超大值绕过上界。
            if (*end != '\0' || end == argv[i] || v != smp_req ||
                smp_req < 1u || smp_req > MAX_HARTS) {
                fprintf(stderr, "--smp: N must be 1..%u (got '%s')" EOL, MAX_HARTS, argv[i]);
                return 1;
            }

        } else {
            fprintf(stderr, "unknown arg: %s" EOL, arg);
            fprintf(stderr, "usage: %s --bios FILE [--load [ADDR=]FILE]... [--blk FILE] [--smp N]" EOL, argv[0]);
            return 1;
        }
    }

    if (load_count == 0) {
        fprintf(stderr, "usage: %s --bios FILE [--load [ADDR=]FILE]... [--blk FILE] [--smp N]" EOL, argv[0]);
        return 1;
    }

    // 写运行期 hart 数 (cpu.{c,h} 全局; --smp 解析值, 默认 1 if 未给)。必须在任何
    // 线程 spawn (最早 clint_start_timer_thread) 之前写定 — n_harts 非 atomic, 靠
    // spawn happens-before 屏障保证多 hart 只读安全 (详 cpu.h n_harts 段)。
    //
    // cap (MAX_HARTS=8) 编译期上限 + n_harts 运行期实际; 单 hart 时 n_harts=1
    // 仍走多线程路径 (单 pthread spawn/join), 跟 SMP 路径同构。详 dummy.txt §15
    // 两数模型 + cap-vs-n_harts 判据。
    n_harts = smp_req;

    // UART TX 走 tx_drain thread → write(STDOUT_FILENO) 直接 syscall, 不经 stdio
    // buffer; emulator 内部其他位置无 stdout 写路径 (fprintf 走 stderr / snprintf
    // 不写 fd). setvbuf 在新模型下无意义, 不调.

    // decode 单测先跑 (decode 是纯函数, 不依赖 ram/cpu/mmu); fail 直接 return 1 不让
    // fixture 跑 (test-driven 模式)。
    if (decode_test() != 0) {
        fprintf(stderr, "decode_test failed" EOL);
        return 1;
    }

    // 全局 ram_init: 调用后 ram.h 暴露的 host_ram_base / gpa_to_hva_offset 可用。
    // 报错风格见 src/dummy.txt §5。
    if (ram_init() != 0) {
        fprintf(stderr, "ram_init failed" EOL);
        return 1;
    }

    // CLINT 注册到 bus (ram_init 之后, dispatcher 启动之前)。本调用不发线程,
    // timer 辅助线程由下方 clint_start_timer_thread 显式起 (谁 spawn 谁 join, dummy.txt
    // §12)。详 platform/clint.h。
    if (clint_init() != 0) {
        fprintf(stderr, "clint_init failed" EOL);
        return 1;
    }

    // PLIC 注册到 bus (clint_init 之后)。PLIC 是 "monitor 但无辅助线程" —
    // hot path 通过 atomic 字段优化 (plic_ctx_eip / plic_pending_bitmap_cache),
    // set/clear 走同步 wrlock; 演进 trail 详 trade_off_log §T.6.
    if (plic_init() != 0) {
        fprintf(stderr, "plic_init failed" EOL);
        return 1;
    }

    // test_dev 注册到 bus (plic_init 之后)。无内部状态 + 无锁; 写 TEST_DEV_SET_OFF /
    // TEST_DEV_CLEAR_OFF 直 fanout 调 plic.device_set/clear_pending。详 device/test_dev.h。
    if (test_dev_init() != 0) {
        fprintf(stderr, "test_dev_init failed" EOL);
        return 1;
    }

    // UART (ns16550a) 注册到 bus (test_dev_init 之后)。**不发线程** — reader 线程由
    // 下方 uart_start_rx_thread 显式起 (谁 spawn 谁 join, dummy.txt §12)。
    // 详 device/uart.h 顶段。
    if (uart_init() != 0) {
        fprintf(stderr, "uart_init failed" EOL);
        return 1;
    }

    // virtio-blk (legacy v1.0) 注册到 bus (uart_init 之后)。blk_path==NULL 时模块
    // 退化 (不 bus 注册, lifecycle 各函数 no-op); 否则 open image_fd + 双 mutex/
    // 双 cond_init + bus 注册。**不发线程** — io_worker 由下方
    // virtio_blk_start_io_worker_thread 显式起 (跟 uart/plic/clint 同形态)。
    // 详 device/virtio_blk.h 顶段。
    if (virtio_blk_init(blk_path) != 0) {
        fprintf(stderr, "virtio_blk_init failed" EOL);
        return 1;
    }

    // 执行所有加载操作 (argv 顺序攒的 load_ops 列表). 推迟到此处而不是 argv
    // 解析时直接调 — loader 写 RAM 需要 ram_init 已 mapped。后写覆盖按 QEMU
    // loader device 同语义, memcpy 不查冲突。
    for (int li = 0; li < load_count; li++) {
        int err;
        if (load_ops[li].as_elf) {
            err = guest_load_elf(load_ops[li].file);
        } else {
            err = guest_load_bin(load_ops[li].file, load_ops[li].addr);
        }
        if (err != 0) {  // loader 内部已 fprintf "why"
            fprintf(stderr, "load failed: %s" EOL, load_ops[li].file);
            return 1;
        }
    }

    // hart 构造: per-hart cpu_t 指针数组初始全 NULL, for 循环 cpu_create n_harts 个,
    // 每个传 mhartid=i (cpu_create 内部双存储 per_hart_info.mhartid CSR mirror +
    // cpu_t.hartid index)。失败回滚: 前 j<i 已 alloc 的 cpu_destroy 后 return 1。
    //
    // misa 参数当前未读取, 仅作 misa 驱动初始化预留 (cpu.h 已 doc)。
    // tlb 容器 + M 共享 leaf 已由 cpu_create eager alloc; [PRIV_S][asid] 的 entries
    // 由 dispatcher 懒分配。cpu_create 内部已写入硬件 reset 后默认状态 (regs[0]=
    // GUEST_RAM_START / priv=PRIV_M / satp=0 / regs[10]=hartid), 跟 cpu_reset 序列
    // 一致, 不需要 main 端再写 hart 字段。
    //
    // pthread_t 句柄按 cap 静态分配 (跟 lifecycle 配对体例; spawn 路径只用
    // 0..n_harts-1)。harts 数组同, 但只 cpu_create 0..n_harts-1, 余下保 NULL
    // (cleanup 循环按 NULL 跳过 — 兜底未来 lazy alloc 路径)。
    cpu_t *harts[MAX_HARTS] = { NULL };
    pthread_t hart_threads[MAX_HARTS] = { 0 };
    for (uint32_t i = 0; i < n_harts; i++) {
        harts[i] = cpu_create(/*misa*/0, /*mhartid*/(uxlen_t)i);
        if (harts[i] == NULL) {  // cpu_create 内部已 fprintf "why"
            fprintf(stderr, "cpu_create(hart %u) failed" EOL, i);
            for (uint32_t j = 0; j < i; j++) { cpu_destroy(harts[j]); }
            return 1;
        }
    }

    // WFI 唤醒框架 init (core/wfi.h; 每 hart 一 pthread_mutex + cond)。
    // 必须在 clint_start_timer_thread 之前 — timer thread 一旦开始 tick 就会
    // 调 clint_recompute_all_mtip → 可能 wfi_kick(i), 那时 wfi_slots 必须已 init。
    // 同理 plic 任何 device_set_pending 路径会调 plic_recompute_ctx_eip_locked →
    // wfi_kick; 当前 PLIC 不在 init 时 set pending (都是 guest 写或 device 后续调),
    // 但保险起见也放在 plic_init 之后。
    if (wfi_init() != 0) {
        fprintf(stderr, "wfi_init failed" EOL);
        for (uint32_t i = 0; i < n_harts; i++) { cpu_destroy(harts[i]); }
        return 1;
    }

    // lrsc 模块 init (A 扩展 reservation 数据结构 + bucket 锁数组 K=64).
    // 必须在 hart 线程 spawn (clint_start_timer_thread 之前) 之前 — 任何 hart 跑
    // LR/SC/store/AMO 都要锁/扫 reservation 数组. lrsc_init 内部 fprintf + 不
    // propagate fail (跟 wfi_init 一致); 真 init fail (pthread_mutex_init OOM 之类)
    // 路径下后续 hart 跑到 pthread_mutex_lock 会撞 EINVAL 暴露问题.
    lrsc_init();

    // JIT 子系统 init (jit_entry.cc 实装; backend.init + jit_cache_init 顺序;
    // T3 stub 阶段 backend.init 返 0 + jit_cache_init 纯 atomic store 也不失败,
    // 不传播 fail; 真做 emit 时 backend.init 失败语义按 wfi_init / lrsc_init 体例
    // fprintf + 不传播, 那时改本处签名接 int 返码 + main 接 fail 退).
    // 必须在 hart 线程 spawn 之前 — JIT 子系统是 hart 共享资源 (cap 配对内部数据 +
    // RCU 等 grace period 都依赖 init 完成).
    jit_init();

    // ------------------------------------------------------------------------
    // POR runtime lifecycle 初始化 (cpu_create 之后, while 之前)
    //
    // SRS / SDS 极性: 0=允许执行, 非0=触发对应停机路径 (bitmap, 详 runtime.h)。
    // runtime.c 定义时已初值 0 兜底, 这里显式 store 0 表达 lifecycle 可读 (进
    // while 前明确把两个 bitmap 设到允许状态)。
    //
    // 顺序无关 — 写 0 是"清空所有 bit", 不存在 race 路径 (此时辅助线程未 spawn)。
    // 不调 set_bit 接口 (那是 set 路径, 写非 0 bit)。
    // ------------------------------------------------------------------------
    atomic_store_explicit(&system_reset_signal, 0u, memory_order_release);
    atomic_store_explicit(&shutdown_signal,     0u, memory_order_release);

    // host signal handler 安装 (SIGINT / SIGTERM / SIGHUP → shutdown_signal_set_bit
    // (EXTERNAL_SIGNAL) + external_signal_no signum)。装在 atomic_store SRS/SDS=0
    // 之后, 线程 spawn 之前 — 这样 handler 装好时 atomic 字段已显式 0, 接到信号
    // 才正确写 bit; 也保证 spawn 线程之前 handler 已就位 (线程内 100ms poll 走
    // SDS 协同退出). 详 runtime.h 顶段 "external signal handler" 一节。
    if (runtime_install_signal_handlers() != 0) {
        return 1;
    }

    // main 起 timer 辅助线程 (受 SDS 控制, 跨 system reset 一直跑; 见 dummy.txt
    // §12 谁 spawn 谁 join + clint.h 顶段 doc)。
    //
    // 错误不走分支 — clint_start_timer_thread 内部 fail 路径 fprintf + set SRS=0 +
    // SDS=0 (release); 下面 while 因 SRS=0 自然不进, 直接走 cleanup 路径。
    // 不分 spawn-fail error path, destroy chain 只在 while 外写一次。
    clint_start_timer_thread();

    // main 起 UART RX reader 辅助线程 (受 SDS 控制, 跟 clint timer 同形态; 内部
    // blocking read(STDIN) + poll 100ms timeout cooperative shutdown)。错误处理跟
    // clint_start_timer_thread 同 — fail 内部 fprintf + set SRS=0 + SDS=0; while
    // 不进; cleanup 在外写一次。
    uart_start_rx_thread();

    // main 起 UART TX drain 辅助线程 (受 SDS 控制; 内部 cond_timedwait
    // (tx_not_empty, 10ms) cooperative shutdown + drain 整 FIFO batch write
    // (STDOUT_FILENO))。hart 写 THR 走 enqueue + cond_signal 立即 wake; FIFO 满
    // silent drop 字节跟真 16550A 一致。错误处理跟 reader 同形态。
    uart_start_tx_thread();

    // main 起 virtio-blk io_worker 辅助线程 (受 SDS 控制, 跟 plic refresh / uart
    // reader 同形态; 内部 cond_timedwait(not_empty, 100ms) + SDS 检 cooperative
    // shutdown; drain avail ring + pread/pwrite + 写 used + device_set_pending)。
    // blk_path==NULL 时 image_fd<0, 函数直接 return 不 spawn (跟 init 的退化对偶)。
    virtio_blk_start_io_worker_thread();

    // ------------------------------------------------------------------------
    // 主 while: system reset 每 iter 进一次
    //
    // per-hart dispatcher 走 pthread_create + pthread_join 循环 (main while 内
    // spawn N pthread + 全 join, 跟 SRS-controlled "占位段" 对偶): system_reset
    // trigger 后所有 hart 跑到 SRS != 0 自然退 dispatcher → join 完 → cpu_reset
    // + 各设备 reset + 重 spawn 下一 iter (sifive 0x7777 路径); ABORT 路径下
    // join 完 break → cleanup chain。
    //
    // spawn fail 处理: 中途某 hart pthread_create 失败 → set SYSRESET_BIT_HART_
    // SPAWN_FAIL → break spawn for; 已 spawn 的看 SRS 自然退 dispatcher → join 完
    // → break main while → cleanup。HART_SPAWN_FAIL 在 ABORT_MASK 内 (runtime.h)。
    // ------------------------------------------------------------------------
    while (atomic_load_explicit(&system_reset_signal, memory_order_acquire) == 0u) {
        // per-hart cpu_reset (idempotent, 保留 hartid/mhartid; 详 cpu.c cpu_reset)
        for (uint32_t i = 0; i < n_harts; i++) { cpu_reset(harts[i]); }
        (void)clint_reset();
        (void)plic_reset();
        (void)test_dev_reset();
        (void)uart_reset();
        (void)virtio_blk_reset();

        // ====================================================================
        // SRS-controlled 线程 spawn — per-hart pthread_create, 跟 system reset
        // 周期绑定 (跟 SDS-controlled "跨 system reset 持续运行" 的 timer/uart/
        // io_worker 分开 — 那类在 POR 段 spawn / POR 收尾段 join, 不在本 while 内)。
        //
        // pthread routine = hart_exec_run (file-static wrapper, 适配 pthread void *(void *)
        // ABI ↔ dispatcher void(cpu_t *) 签名; 详顶部 hart_exec_run 段)。
        //
        // spawn fail: 中途某 i 失败 → set SYSRESET_BIT_HART_SPAWN_FAIL (ABORT) +
        // break for; 已 spawn 的 0..spawned-1 看 SRS 自然退 dispatcher。spawned 记
        // 真起成功的数, 下面 join 段按 spawned 跑 (失败时只 join 已 spawn 的, 防
        // 用 hart_threads[i>=spawned] 的 0 句柄撞 pthread_join)。
        // ====================================================================
        uint32_t spawned = 0;
        for (uint32_t i = 0; i < n_harts; i++) {
            int rc = pthread_create(&hart_threads[i], NULL, hart_exec_run, harts[i]);
            if (rc != 0) {
                fprintf(stderr, "pthread_create(hart %u) failed: %s" EOL, i, strerror(rc));
                system_reset_signal_set_bit(SYSRESET_BIT_HART_SPAWN_FAIL);
                break;
            }
            spawned++;
        }
        // ====================================================================
        // SRS-controlled 线程 join — 跟上面 spawn 对偶。spawn 全部成功时
        // spawned == n_harts; 部分失败时 spawned < n_harts, 只 join 已 spawn 的。
        //
        // 哲学 = "所有 hart self-poll SRS 自己停" (dispatcher 内部 while(SRS==0)
        // 退); shutdown 路径下 hart 在 wfi 时靠 wfi.c cond_timedwait 兜底
        // WFI_TIMEOUT_NS (config.h 500ms) 自醒 + predicate 重检 SRS 退出, 接受
        // 此 tail latency. signal handler / 非 signal handler 路径无差异, 没有
        // "通知者" 角色. 真撞 shutdown tail 延迟问题再回看 wfi.h 顶段 +
        // wfi_kick_all (当前唯一活调用点 = runtime_fatal 紧急停机)。
        // ====================================================================
        for (uint32_t i = 0; i < spawned; i++) {
            pthread_join(hart_threads[i], NULL);
        }

        // ====================================================================
        // 区分 system reset 重 iter vs cleanup-and-exit
        //
        // dispatcher 退出后 system_reset_signal 非 0, 其 bit 编码退出原因 (详
        // runtime.h)。规则:
        //   - SRS & ABORT_MASK 命中 (SHUTDOWN_TRIGGER / DEVICE_FAIL / HART_MDT)
        //     → cleanup + return exit_code (按 sifive_test 写的 exit_code, 其他
        //     fatal 路径默认 0)
        //   - 仅 SYSRESET_BIT_TEST_RESET 命中 (0x7777 sifive_test reset)
        //     → try_clear_if_shutdown_zero CAS; 清成功 → continue 进下一 iter
        //       (timer/uart/io_worker 跨 reset 一直跑); 清失败 (shutdown 非 0,
        //        即同时发生设备 fail 等) → 走 cleanup
        //
        // 取消 test_dev_consume_reset_request — reset 触发改用 SRS bit TEST_RESET
        // 表达; main while 直接读 SRS bit (acquire 顺路同步 release-store)。
        // ====================================================================
        {
            uint32_t srs_state = atomic_load_explicit(&system_reset_signal,
                                                     memory_order_acquire);
            if ((srs_state & SYSRESET_BIT_TEST_RESET) &&
                !(srs_state & SYSRESET_ABORT_MASK) &&
                system_reset_signal_try_clear_if_shutdown_zero()) {
                continue;
            }
        }
        break;
    }

    // ------------------------------------------------------------------------
    // POR 退出段统一通知 SDS-controlled 线程退出: shutdown_signal_set_bit
    // (NORMAL_EXIT) — 若 shutdown bit 已被别人设 (e.g. timer fail 设
    // DEVICE_FAIL / test_dev PASS/FAIL 设 NORMAL_EXIT), 这步 fetch_or 不丢已设
    // bit; 若是 ABORT_MASK 路径 (HART_MDT / 设备 fail 通过 SRS bit 触发)
    // shutdown 还是 0, 这步是 main 顶层决定退出主动设 NORMAL_EXIT 让 thread 退。
    // 调用前置 dummy.txt §12 谁 spawn 谁 join — set 完才 join, 否则 join 死等。
    // ------------------------------------------------------------------------
    shutdown_signal_set_bit(SHUTDOWN_BIT_NORMAL_EXIT);

    // hart 退出协议走 SRS self-poll, 不需要主动 wfi_kick — 上方 dispatcher 返回
    // 时 hart 已退 wfi 退 main loop. 哲学详上方 dispatcher 返回后占位段; wfi_kick_all
    // 函数保留备多 hart / 灵感 (详 wfi.h).

    // ------------------------------------------------------------------------
    // POR 退出段 (while 外): 回收 SDS 控制的辅助线程 → 三 destroy
    //
    // clint_join_timer_thread 调用前置 (SDS 非 0) 已由上方 shutdown_signal_set_bit
    // (NORMAL_EXIT) 保证; spawn fail path 下 thread handle 是 BSS 0, pthread_join
    // 在 glibc/musl 下返 ESRCH "No such process", fprintf 一行不 fatal。详
    // clint.h clint_join_timer_thread 顶段 doc。
    //
    // CPU dump 已挪进 cpu_destroy (DEBUG_CPU_DUMP_ON gate); 见 cpu.c cpu_dump。
    // ------------------------------------------------------------------------
    clint_join_timer_thread();
    uart_join_rx_thread();
    uart_join_tx_thread();
    virtio_blk_join_io_worker_thread();

    // ------------------------------------------------------------------------
    // 程序总耗时 (main lifecycle scope; CLOCK_MONOTONIC delta)
    //
    // 不进 debug.{c,h} — debug.{c,h} scope = interpreter / dispatcher 内部
    // char-stream trace (per-block 'f' / 'E' 等); 程序总耗时是 main 边界级,
    // 不是 hot path 内部状态。未来 per-hart wall clock 时再考虑放 cpu_info /
    // debug。
    //
    // 输出: "X min Y.YYY s (total Z.ZZZ s)"
    //   - 前段 min + s 拆分给人类视角直观 (跑了几分钟还是几秒)
    //   - 括号内总秒数 (double) 给后期 fixture 自动解析友好
    // ------------------------------------------------------------------------
    struct timespec t_end;
    clock_gettime(CLOCK_MONOTONIC, &t_end);
    double total_s = (double)(t_end.tv_sec - t_start.tv_sec)
                   + (double)(t_end.tv_nsec - t_start.tv_nsec) / 1e9;
    int    e_min   = (int)(total_s / 60.0);
    double e_sec   = total_s - (double)e_min * 60.0;
    fprintf(stderr, "[main] elapsed: %d min %.3f s (total %.3f s)" EOL,
            e_min, e_sec, total_s);

    clint_destroy();
    plic_destroy();
    test_dev_destroy();
    uart_destroy();
    virtio_blk_destroy();
    // WFI 框架 destroy (各 hart pthread_mutex_destroy + cond_destroy)。
    // 必须在所有 wfi_kick 来源 (timer thread / io workers 已 join + clint/plic destroy
    // 也已 atomic 清字段) 之后 — clint_destroy / plic_destroy 都不再调 wfi_kick。
    wfi_destroy();
    // lrsc destroy (bucket 锁数组 destroy; 跟 wfi_destroy 同顺位 — 都是 cap 配对
    // pure cleanup 不发线程不 join, 在所有 hart 线程已 join 之后调).
    lrsc_destroy();
    // JIT 子系统 shutdown (跟 jit_init 反序配对; jit_cache_destroy → backend.destroy
    // 走 jit_entry.cc jit_shutdown 内部顺序). 在所有 hart 线程已 join 之后调 —
    // 任何 hart 还在跑就 invalidate / unmap host_code mmap 区会 segfault.
    jit_shutdown();
    // per-hart cpu_destroy. NULL 跳过 — 当前 cpu_create 失败时已 early return,
    // 这里循环只跑成功初始化的; NULL 检查兜底未来 lazy alloc 路径。
    for (uint32_t i = 0; i < n_harts; i++) {
        if (harts[i] != NULL) { cpu_destroy(harts[i]); }
    }
    ram_destroy();

    // 还原 SIG_DFL — main 退出后子进程 / 后续 process state 干净。
    runtime_restore_signal_handlers();

    // exit code 分支:
    //   - EXTERNAL_SIGNAL (SIGINT/SIGTERM/SIGHUP) 命中: 按 POSIX 惯例返 128 + signum
    //     (130 / 143 / 129); stderr 顺手打印 test_dev_exit_code (默认 0 = "还没跑
    //     到 FINISHER", 非 0 = test_dev 已设的 FAIL arg), 给 fixture 自动化/人工
    //     debug 留 trail (EXTERNAL_SIGNAL > NORMAL_EXIT 优先;
    //     "user 硬意图退出 不应被 test_dev 抢先写的 0 盖过")
    //   - 否则 (NORMAL_EXIT 命中 / DEVICE_FAIL 命中 / HART_MDT 命中): 沿用旧路径
    //     返 test_dev_get_exit_code() — sifive_test FINISHER PASS=0 / FAIL=arg;
    //     DEVICE_FAIL / HART_MDT 路径下 test_dev_exit_code 默认 0
    // happens-before: test_dev_write 写 exit_code (plain) 后 atomic_store SDS=...
    // release; main while 读 SRS!=0 acquire 退出后再读 exit_code, plain 字段通过
    // atomic release-acquire 边界跨线程可见。
    uint32_t sds_state = atomic_load_explicit(&shutdown_signal, memory_order_acquire);
    if (sds_state & SHUTDOWN_BIT_EXTERNAL_SIGNAL) {
        uint32_t sig = atomic_load_explicit(&external_signal_no, memory_order_acquire);
        int test_code = test_dev_get_exit_code();
        fprintf(stderr,
                "[main] external signal %u -> exit %d; "
                "test_dev exit code at interrupt: %d" EOL,
                sig, 128 + (int)sig, test_code);
        return 128 + (int)sig;
    }
    return test_dev_get_exit_code();
}
