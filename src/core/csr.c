//
// Created by liujilan on 2026/5/4.
// a_01_5_c csr 模块实现 (csr_op 大 helper + 6 个小 r/w helper 真读写 hart->trap 字段)。
//
// 顶部模块文档见 csr.h; 跨文件协议见 src/dummy.txt §1。
//
// 演进:
//   a_01_5_a: csr_op 框架 + 6 小 helper fprintf 占位 (read 返 0, write 仅 fprintf)
//   a_01_5_c: 6 小 helper 改真读写 hart->trap 的对应字段; 删 fprintf; 大 switch 不变
//
// csr 编号 → trap_csrs_t 字段映射:
//   mstatus  (0x300) → _mstatus 低 32 位 (mstatus 物理 64 位被 RV32 拆 mstatus + mstatush 两 csr)
//   mstatush (0x310) → _mstatus 高 32 位 (RV32-only csr 入口)
//   mtvec    (0x305) → xtvec[PRIV_M]; write WARL mask 低 2 位 (项目不实现 Vectored, 强制 Direct)
//   mepc     (0x341) → xepc[PRIV_M];  write WARL mask 低 IALIGN_MASK 位 (RV spec mepc[0]=0
//                       when IALIGN=16; mepc[1:0]=0 when IALIGN=32)
//   mcause   (0x342) → xcause[PRIV_M]
//   mtval    (0x343) → xtval[PRIV_M]
//
// csr_op 入口的 priv / RO 检查路径仍是注释占位 (fixture 不构造非法访问, a_01_5_b 起
//   trap_raise_exception 已可调用, 但当前未真接到入口判路径; 真接见下方 csr_op 注释段)。
//

#include "csr.h"

#include "config.h"     // IALIGN_MASK
#include "cpu.h"
#include "riscv.h"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>


// ============================================================================
// 小 r/w helper —— 每个 csr 一对, file-static, csr_op 大 switch 调用
//
// a_01_5_c 真接 hart->trap 字段。
// 命名规则 (与 trap.h 一致):
//   - mstatus / mstatush 操作 _mstatus (uint64_t) 的低/高 半边
//   - mtvec / mepc / mcause / mtval 操作 xxx[PRIV_M] (priv-indexed, x 前缀)
// ============================================================================

// ---- mstatus 半边 (mstatus 物理 64 位, mstatus = 低 32, mstatush = 高 32) ----
//
// read 取对应半边; write 通过 mask 改半边保留另半边。
// sstatus 是 _mstatus 的 masked view, a_01_5_c 不实现 (S-mode 真做时加 csr_sstatus_*)。

static uint32_t csr_mstatus_read(cpu_t *hart) {
    return (uint32_t)(hart->trap._mstatus & 0xFFFFFFFFu);
}

static void csr_mstatus_write(cpu_t *hart, uint32_t v) {
    // 低 32 位换成 v, 高 32 位保留。
    // RV spec mstatus 字段 WARL: MIE/MPIE/MPP 等是合法位, 当前 a_01 全部接受不截断
    // (a_03 真做中断时按 spec 加 WARL 截断: e.g. MPP 写非法 priv 编码时落到合法值)。
    hart->trap._mstatus = (hart->trap._mstatus & 0xFFFFFFFF00000000ULL)
                        | (uint64_t)v;
}

static uint32_t csr_mstatush_read(cpu_t *hart) {
    return (uint32_t)((hart->trap._mstatus >> 32) & 0xFFFFFFFFu);
}

static void csr_mstatush_write(cpu_t *hart, uint32_t v) {
    // 高 32 位换成 v, 低 32 位保留。
    // mstatush 字段 (SBE / MBE 等) a_01 全部 0, 不真触发 endian 切换。
    hart->trap._mstatus = (hart->trap._mstatus & 0x00000000FFFFFFFFULL)
                        | ((uint64_t)v << 32);
}

// ---- mtvec / mepc / mcause / mtval (映射到 hart->trap.{xtvec,xepc,xcause,xtval}[PRIV_M]) ----

static uint32_t csr_mtvec_read(cpu_t *hart) {
    return hart->trap.xtvec[PRIV_M];
}

static void csr_mtvec_write(cpu_t *hart, uint32_t v) {
    // WARL 截断 MODE 位 (项目不实现 Vectored, 强制 Direct):
    //   - mtvec[1:0] = MODE: 00 = Direct (全部 trap 跳 BASE; 项目支持)
    //                         01 = Vectored (async/interrupt 跳 BASE+4*cause; 项目不支持)
    //                         10/11 = reserved by RV spec
    //   - mtvec[31:2] = BASE
    // RV spec WARL 允许实现"不支持的 MODE 写入 → 落到合法值"; 我们选 mask 低 2 位为 0
    // (即 MODE 永远存 00 = Direct), 跟 trap_set_state 内 hart->regs[0] = xtvec[deliver_priv]
    // 直接赋值 (不算 BASE+4*cause) 一致。
    // 未来 a_03 真做中断时, 解开本 mask 接受 MODE; trap_set_state 按 cause 分流 (sync 跳 BASE,
    // async 跳 BASE + 4*cause)。
    hart->trap.xtvec[PRIV_M] = v & ~0x3u;
}

static uint32_t csr_mepc_read(cpu_t *hart) {
    return hart->trap.xepc[PRIV_M];
}

static void csr_mepc_write(cpu_t *hart, uint32_t v) {
    // WARL 截断 IALIGN 对齐位:
    //   - IALIGN=16 (项目当前): mepc[0] = 0 (低 1 位强制 0)
    //   - IALIGN=32: mepc[1:0] = 0 (低 2 位强制 0)
    // config.h IALIGN_MASK = IALIGN_BYTES - 1 (= 1 当 IALIGN=16; = 3 当 IALIGN=32),
    // & ~IALIGN_MASK 即"截断到 IALIGN 对齐"。
    // RV spec §3.1.15 mepc: "the low bit of mepc (mepc[0]) is always zero. ... If an
    // implementation supports only IALIGN=32, then the two low bits (mepc[1:0]) are always
    // zero." 我们按 config.h 编译期 IALIGN 配置自动适配。
    hart->trap.xepc[PRIV_M] = v & ~IALIGN_MASK;
}

static uint32_t csr_mcause_read(cpu_t *hart) {
    return hart->trap.xcause[PRIV_M];
}

static void csr_mcause_write(cpu_t *hart, uint32_t v) {
    // mcause 字段: bit[31] = Interrupt (1) vs Exception (0); bit[30:0] = Exception/Interrupt code。
    // RV spec §3.1.16 没强制 WARL (除了 MSB; "implementations may further restrict"), 我们当前
    // 接受全 32 位写入。a_01 fixture 一般也不直接写 mcause (handler 只读, trap_set_state 写)。
    hart->trap.xcause[PRIV_M] = v;
}

static uint32_t csr_mtval_read(cpu_t *hart) {
    return hart->trap.xtval[PRIV_M];
}

static void csr_mtval_write(cpu_t *hart, uint32_t v) {
    // mtval RV spec §3.1.17 没强制 WARL, 接受任意值。
    hart->trap.xtval[PRIV_M] = v;
}


// ============================================================================
// csr_op —— 大 helper, decode 分发入口 (与 a_01_5_a 形态相同, 不动)
// ============================================================================

uint32_t csr_op(cpu_t *hart, uint32_t csr_addr, uint32_t new_val,
                csr_op_t op, uint32_t raw_inst) {
    (void)raw_inst;     // a_01_5_c 入口判 priv/RO 仍是注释占位; 参数透传留未来真激活

    // ----------------------------------------------------------------------------
    // 入口判: priv 要求 + RO 写检查 (csr 编号自带权限位段, riscv.h CSR_ADDR_PRIV_*)
    //
    // 注释占位 (fixture 不构造非法 csr 访问, 跳过这两条检查):
    //
    //   uint32_t required_priv = (csr_addr >> CSR_ADDR_PRIV_SHIFT) & CSR_ADDR_PRIV_MASK;
    //   if (hart->priv < required_priv) {
    //       trap_raise_exception(hart, /*cause*/2, raw_inst);  // _Noreturn longjmp
    //   }
    //
    //   uint32_t is_ro = ((csr_addr >> CSR_ADDR_RO_SHIFT) & CSR_ADDR_RO_MASK) == CSR_ADDR_RO_VALUE;
    //   uint32_t is_write = (op == CSR_OP_RW) || (new_val != 0);
    //   if (is_ro && is_write) {
    //       trap_raise_exception(hart, /*cause*/2, raw_inst);  // _Noreturn longjmp
    //   }
    //
    // 真用到 (例如 fixture 写 mhartid 这种 RO csr) 时解开。
    // ----------------------------------------------------------------------------

    // ----------------------------------------------------------------------------
    // 大 switch: 按 csr_addr 分发到具体 csr 的小 r/w helper, 算 read_old + write_back。
    //
    // 加新 csr 时只在这里加 case + 写一对 csr_<name>_<r/w> file-static helper (上方)。
    // 不存在的 csr addr 走 default → 当前 fprintf + 返回 0 (一时占位, 真用 trap 还没接);
    // 未来某个 fixture 写不存在的 csr 时, 解开 trap_raise_exception(2, raw_inst)。
    // ----------------------------------------------------------------------------
    uint32_t read_old;
    switch (csr_addr) {
        case CSR_MSTATUS:  read_old = csr_mstatus_read (hart); break;
        case CSR_MSTATUSH: read_old = csr_mstatush_read(hart); break;
        case CSR_MTVEC:    read_old = csr_mtvec_read   (hart); break;
        case CSR_MEPC:     read_old = csr_mepc_read    (hart); break;
        case CSR_MCAUSE:   read_old = csr_mcause_read  (hart); break;
        case CSR_MTVAL:    read_old = csr_mtval_read   (hart); break;
        default:
            // 占位: fprintf + 返回 0; 未来真用 trap 时换成:
            //   trap_raise_exception(hart, /*cause*/2, raw_inst);  // _Noreturn
            fprintf(stderr,
                    "[csr] unknown csr addr=0x%03" PRIx32 " (would trap cause 2)\n",
                    csr_addr);
            return 0;
    }

    // 算 new + write_back; switch on csr_op_t (3 case 全覆盖, -Wswitch-enum 强制)
    uint32_t to_write;
    switch (op) {
        case CSR_OP_RW: to_write = new_val;             break;
        case CSR_OP_RS: to_write = read_old |  new_val; break;
        case CSR_OP_RC: to_write = read_old & ~new_val; break;
    }

    // RV spec §2.1.2: RS/RC + new_val=0 不写 (副作用不发生)。RW 总是写。
    // 这条同时为 RO csr 提供保护 (RS/RC + 0 不触发"写 RO" trap; 当然入口判已经放过了)。
    int do_write = (op == CSR_OP_RW) || (new_val != 0);
    if (do_write) {
        switch (csr_addr) {
            case CSR_MSTATUS:  csr_mstatus_write (hart, to_write); break;
            case CSR_MSTATUSH: csr_mstatush_write(hart, to_write); break;
            case CSR_MTVEC:    csr_mtvec_write   (hart, to_write); break;
            case CSR_MEPC:     csr_mepc_write    (hart, to_write); break;
            case CSR_MCAUSE:   csr_mcause_write  (hart, to_write); break;
            case CSR_MTVAL:    csr_mtval_write   (hart, to_write); break;
            default:
                // 上面 read 路径的 default 已 fprintf + return 0, 不会到这里。
                break;
        }
    }

    return read_old;
}
