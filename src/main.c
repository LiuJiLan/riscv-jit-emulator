//
// Created by liujilan on 2026/4/28.
// a_01 main。
// 当前形态: ram_init → loader → cpu_create + 字段设置 →
//             多次 dispatcher self-check (a01_2 mmu_translate_pc 端到端验证) → cpu_destroy。
// dispatcher 当前 a01_2 简化形态 (block 1+2 + 临时输出, 无循环), 完整 dispatcher
// (sigsetjmp + while loop + block 3 派发) 等 a01_5 真接入。
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
    // ecall = 0x00000073: opcode 0x73 a_01_3 不识别 → OP_UNSUPPORTED, pc_step 仍 PC_STEP_RV
    CASE(0x00000073, OP_UNSUPPORTED, /*rd*/0, /*rs1*/0, /*rs2*/0, 0, 0x00000073, PC_STEP_RV);

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

    // ---- a_01_4 待加 (B-type / J-type 立即数解码 + boundary case) ----
    // ---- a_01_5 待加 (CSR* / ECALL / EBREAK / MRET) ----
    // ---- a_01_6 待加 (LB/LH/LW/LBU/LHU + SB/SH/SW; S-type 立即数 bit[31:25, 11:7]) ----
    // ---- 未来 RVC 扩展 (C.MV / C.ADD / C.LUI / C.SUB / ... 真翻译) ----

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

    // ------------------------------------------------------------------------
    // a01_3: 跑 fixture 一次, dispatcher 内部 if(1) 包 block 1+2+3, 调
    // interpret_one_block 解释执行, fixture 末尾 ecall 触发 OP_UNSUPPORTED 让解释器
    // break 出 fetch loop, 回 dispatcher, dispatcher 报告 count + pc, 然后回到 main
    // 这里 fprintf 关心的寄存器值。
    //
    // 期望 (跑 tests/a_01/a01_3/01_arith_basic/out.bin):
    //   x1  = 42                             (addi x1, x0, 42)
    //   x2  = 8                              (addi x2, x0, 8)
    //   x3  = 50  ← 主测试目标               (add  x3, x1, x2)
    //   pc  = 0x8000000c                     (ecall 地址, OP_UNSUPPORTED 时 pc 不前进)
    //   count = 3 (dispatcher 那一行打)      (3 条算术, 不含 ecall)
    // ------------------------------------------------------------------------
    int rc = dispatcher(hart);
    if (rc != 0) {
        fprintf(stderr, "dispatcher returned %d (fetch trap or other failure)\n", rc);
    }

    fprintf(stderr,
            "[main] result: pc=0x%08x x1=%u x2=%u x3=%u\n",
            hart->regs[0], hart->regs[1], hart->regs[2], hart->regs[3]);

    cpu_destroy(hart);
    return 0;
}
