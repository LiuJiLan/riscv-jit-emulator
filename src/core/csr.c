//
// Created by liujilan on 2026/5/4.
// a_01_5_a csr 模块实现起步: csr_op 大 helper 框架 + 5 个小 r/w helper (mstatus / mstatush
//   / mtvec / mepc / mcause / mtval; 后两半边 mstatus/mstatush 共享物理字段 _mstatus)。
//
// 顶部模块文档见 csr.h; 跨文件协议见 src/dummy.txt §1 (sigsetjmp / 大入口 vs 直调辅助函数)。
//
// a_01_5_a 阶段范围:
//   - 不依赖 hart->trap.* 字段 (a_01_5_b 才加 trap_csrs_t 内嵌 cpu_t)
//   - 小 r/w helper 全部是 fprintf 占位 + 返回 0 (read) / fprintf 参数 (write); a_01_5_b
//     改成真读写 hart->trap._xxx
//   - csr_op 入口的 priv / RO 检查路径用注释占位 (trap_raise_exception 还没真接 longjmp,
//     a_01_5_c 才接); 当前 fixture 不构造非法 csr 访问, 这条路径不会被触发
//   - 不存在的 csr addr 走 default 路径, 现在用 fprintf + 返回 0; a_01_5_c 改 trap
//

#include "csr.h"

#include "cpu.h"
#include "riscv.h"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

// ============================================================================
// 小 r/w helper —— 每个 csr 一对, file-static, csr_op 大 switch 调用
//
// a_01_5_a: 全部 fprintf 占位 + read 返 0 / write fprintf 参数。设计保留接口形态稳定,
// a_01_5_b 改实现 (内部读写 hart->trap._xxx) 时不动 csr_op 大 switch 的调用现场。
// ============================================================================

// ---- mstatus 半边 (mstatus 物理 64 位, mstatus = 低 32, mstatush = 高 32) ----
//
// a_01_5_b 真接 hart->trap._mstatus (uint64_t) 后, read 取低/高 32 位, write 通过 mask
// 改半边保留另半边。当前占位:
static uint32_t csr_mstatus_read(cpu_t *hart) {
    (void)hart;
    fprintf(stderr, "[csr_a stub] read mstatus -> 0\n");
    return 0;
}

static void csr_mstatus_write(cpu_t *hart, uint32_t v) {
    (void)hart;
    fprintf(stderr, "[csr_a stub] write mstatus = 0x%08" PRIx32 "\n", v);
}

static uint32_t csr_mstatush_read(cpu_t *hart) {
    (void)hart;
    fprintf(stderr, "[csr_a stub] read mstatush -> 0\n");
    return 0;
}

static void csr_mstatush_write(cpu_t *hart, uint32_t v) {
    (void)hart;
    fprintf(stderr, "[csr_a stub] write mstatush = 0x%08" PRIx32 "\n", v);
}

// ---- mtvec / mepc / mcause / mtval (a_01_5_b 后映射到 hart->trap._tvec[PRIV_M] 等) ----

static uint32_t csr_mtvec_read(cpu_t *hart) {
    (void)hart;
    fprintf(stderr, "[csr_a stub] read mtvec -> 0\n");
    return 0;
}

static void csr_mtvec_write(cpu_t *hart, uint32_t v) {
    (void)hart;
    fprintf(stderr, "[csr_a stub] write mtvec = 0x%08" PRIx32 "\n", v);
}

static uint32_t csr_mepc_read(cpu_t *hart) {
    (void)hart;
    fprintf(stderr, "[csr_a stub] read mepc -> 0\n");
    return 0;
}

static void csr_mepc_write(cpu_t *hart, uint32_t v) {
    (void)hart;
    fprintf(stderr, "[csr_a stub] write mepc = 0x%08" PRIx32 "\n", v);
}

static uint32_t csr_mcause_read(cpu_t *hart) {
    (void)hart;
    fprintf(stderr, "[csr_a stub] read mcause -> 0\n");
    return 0;
}

static void csr_mcause_write(cpu_t *hart, uint32_t v) {
    (void)hart;
    fprintf(stderr, "[csr_a stub] write mcause = 0x%08" PRIx32 "\n", v);
}

static uint32_t csr_mtval_read(cpu_t *hart) {
    (void)hart;
    fprintf(stderr, "[csr_a stub] read mtval -> 0\n");
    return 0;
}

static void csr_mtval_write(cpu_t *hart, uint32_t v) {
    (void)hart;
    fprintf(stderr, "[csr_a stub] write mtval = 0x%08" PRIx32 "\n", v);
}

// ============================================================================
// csr_op —— 大 helper, decode 分发入口
// ============================================================================

uint32_t csr_op(cpu_t *hart, uint32_t csr_addr, uint32_t new_val,
                csr_op_t op, uint32_t raw_inst) {
    (void)raw_inst;     // a_01_5_a 不真调 trap_raise_exception, 参数透传留 a_01_5_b 激活

    // ----------------------------------------------------------------------------
    // 入口判: priv 要求 + RO 写检查 (csr 编号自带权限位段, riscv.h CSR_ADDR_PRIV_*)
    //
    // a_01_5_a 注释占位; a_01_5_b cpu_t 加 trap_csrs_t + helper 真路径后激活:
    //
    //   uint32_t required_priv = (csr_addr >> CSR_ADDR_PRIV_SHIFT) & CSR_ADDR_PRIV_MASK;
    //   if (hart->priv < required_priv) {
    //       trap_raise_exception(hart, /*cause*/2, raw_inst);  // 不返回 (longjmp)
    //   }
    //
    //   uint32_t is_ro = ((csr_addr >> CSR_ADDR_RO_SHIFT) & CSR_ADDR_RO_MASK) == CSR_ADDR_RO_VALUE;
    //   // RV spec §2.1.2: CSRRS/RC/RSI/RCI + new_val=0 不视为写 (副作用不发生);
    //   //                  CSRRW/RWI 总是写 (rs1=x0 / zimm=0 时也写)
    //   uint32_t is_write = (op == CSR_OP_RW) || (new_val != 0);
    //   if (is_ro && is_write) {
    //       trap_raise_exception(hart, /*cause*/2, raw_inst);  // 不返回 (longjmp)
    //   }
    //
    // a_01_5_a 阶段假设 fixture 不构造非法 csr 访问, 跳过这两条检查; helper 真接 longjmp
    // 后这条入口判会比小 helper 调用先触发 trap, 路径切换。
    // ----------------------------------------------------------------------------

    // ----------------------------------------------------------------------------
    // 大 switch: 按 csr_addr 分发到具体 csr 的小 r/w helper, 算 read_old + write_back。
    //
    // 加新 csr 时只在这里加 case + 写一对 csr_<name>_<r/w> file-static helper (上方)。
    // 不存在的 csr addr 走 default → fprintf 占位 (a_01_5_a) / a_01_5_b 改
    // trap_raise_exception(2, raw_inst)。
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
            // a_01_5_a 占位: fprintf + 返回 0; a_01_5_b 改:
            //   trap_raise_exception(hart, /*cause*/2, raw_inst);  // 不返回
            fprintf(stderr,
                    "[csr_a stub] unknown csr addr=0x%03" PRIx32 " (a_01_5_b: trap cause 2)\n",
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
    // 对 csr_<name>_write 而言, 写 read_old 等价 NO-OP, 但有 fprintf 副作用 (a_01_5_a 占位
    // 期间会多一行 stderr); 真路径下这条优化是为了 RO csr 不误触发 RO-write trap (上面
    // 入口判已经过了, 这里也不再触发, 但保持 RV 语义最严)。
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
