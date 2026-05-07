//
// Created by liujilan on 2026/5/4.
// a_01_5_c csr 模块实现 (csr_op 大 helper + 6 个小 r/w helper 真读写 hart->trap 字段)。
//
// 顶部模块文档见 csr.h; 跨文件协议见 src/dummy.txt §1。
//
// 演进:
//   a_01_5_a: csr_op 框架 + 6 小 helper fprintf 占位 (read 返 0, write 仅 fprintf)
//   a_01_5_c: 6 小 helper 改真读写 hart->trap 的对应字段; 删 fprintf; 大 switch 不变
//   a_01_7:   csr_op 入口判 priv/RO 真激活 (替换 a_01_5_a 注释占位); csr_mstatus_write 加 MPP
//             WARL (PRIV_H=2 → PRIV_U=0); 大 switch default 解开为 fprintf+trap_raise (访问
//             未实现 csr → cause 2)
//   a_01_8:   加 satp 入口 (csr_satp_read/write + 大 switch case); satp_write WARL ASID 截断
//             到 ASID_MASK 位 — dummy.txt §3 satp 合法性契约的"生产者"职责
//
// csr 编号 → 字段映射 (a_01_8 起 satp 不在 trap_csrs_t 里, 改名"字段映射"):
//   mstatus  (0x300) → trap._mstatus 低 32 位 (mstatus 物理 64 位被 RV32 拆 mstatus + mstatush 两 csr)
//   mstatush (0x310) → trap._mstatus 高 32 位 (RV32-only csr 入口)
//   mtvec    (0x305) → trap.xtvec[PRIV_M]; write WARL mask 低 2 位 (项目不实现 Vectored, 强制 Direct)
//   mepc     (0x341) → trap.xepc[PRIV_M];  write WARL mask 低 IALIGN_MASK 位 (RV spec mepc[0]=0
//                       when IALIGN=16; mepc[1:0]=0 when IALIGN=32)
//   mcause   (0x342) → trap.xcause[PRIV_M]
//   mtval    (0x343) → trap.xtval[PRIV_M]
//   satp     (0x180) → hart->satp (cpu_t 直接持有字段, 不在 trap_csrs_t — satp 不属于 trap-related
//                       CSR 范畴; write WARL ASID 截断到 ASID_MASK 位, 见 dummy.txt §3)
//

#include "csr.h"

#include "config.h"     // IALIGN_MASK
#include "cpu.h"
#include "riscv.h"
#include "trap.h"       // trap_raise_exception (a_01_7 csr_op 入口判 priv/RO 失败时长跳)

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
    // 低 32 位换成 v (经 WARL 截断), 高 32 位保留。
    //
    // a_01_7 WARL 截断 (按字段递增加, 当前只截 MPP; MIE/MPIE 等不截):
    //   MPP (bits 12:11): 项目支持 priv 集 = {PRIV_U=0, PRIV_S=1, PRIV_M=3}; PRIV_H=2 在没 H
    //     扩展时非法。RV spec WARL 允许实现自由选择 fallback; 项目选 PRIV_U=0 落点 ("least-priv
    //     风险最小": mret 切到 U 比切到 M 安全)。
    //
    // !!! 未来 H 扩展 / SVxx-mode 真做时这里要改 !!!
    //   - H 扩展: PRIV_H=2 编码不再保留, 项目 priv 集变成 {U=0, S=1, H=2, M=3}, 写 2 不截
    //   - 严格按 spec 的实现可能选 fallback 到"上次合法值"而不是 PRIV_U; 那时 csr.c 需要保留
    //      MPP 历史状态 (file-static prev_mpp 之类)
    //   - S-mode + sstatus 入口加进来时, sstatus.SPP 也要类似 WARL (1-bit, 只允许 0/1)
    //
    // MIE / MPIE 当前不截 (RV spec 允许实现接受任意位值; 只是行为体现是中断使能)。
    // a_03 真做中断时, 如果 MIE 字段需要更细的 WARL (比如某些子字段保留), 在这里加。
    {
        uint32_t mpp = (v >> MSTATUS_MPP_SHIFT) & MSTATUS_MPP_MASK;
        if (mpp == PRIV_H) {
            /* PRIV_H = 2, 项目当前不支持 → 落 PRIV_U = 0
             * 操作: 清 MPP 字段位; 因 PRIV_U = 0, 不需要再 OR 设新值。 */
            v &= ~MSTATUS_MPP;
        }
    }

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

// ---- satp (a_01_8 加; dummy.txt §3 satp 合法性契约的"生产者"职责) ----
//
// satp 物理存储在 hart->satp (cpu_t 顶层字段, 不在 trap_csrs_t 内 — satp 不属于
// trap-related CSR 范畴, 设计意图见 cpu.h)。

static uint32_t csr_satp_read(cpu_t *hart) {
    return hart->satp;
}

static void csr_satp_write(cpu_t *hart, uint32_t v) {
    // satp 字段 (RV32 Sv32; RV Privileged Spec Vol II §4.1.11 fig 4.11):
    //   bit  31     = MODE  (1-bit; 0 = Bare 恒等, 1 = Sv32)
    //   bits 30:22 = ASID  (Sv32 spec 9 位; 项目 ASIDLEN = TLB_ASID_BITS = 4 位)
    //   bits 21:0  = PPN   (root page table physical page number; root PA = PPN << 12)
    //
    // WARL 截断 (dummy.txt §3):
    //   - ASID 截到 ASID_MASK 位 (= 0xF, 4 位); dispatcher 直接按 cpu->satp.ASID 索引
    //     tlb_table[priv][asid] 无 bounds check, 本截断是 host 内存安全的根防线。
    //   - MODE 是 RV32 单 bit, 0/1 都合法, 不需截 (RV64 4-bit MODE 才需要落到合法值)。
    //   - PPN 不截; RV spec 不强制 WARL, 非法 PPN 由 walker 访问时按权限/范围检查触发 fault。
    //
    // satp 写不自动 sfence.vma (RV spec + plan §1.8 + dummy.txt §3): guest 软件必须显式
    // sfence.vma 才让新 ASID/MODE 在 TLB 生效。本 helper 只更新 hart->satp 字段, 不动 TLB;
    // dispatcher 下次 block 入口算 (regime, current_tlb) 时按新 satp 选叶 TLB, 但旧 ASID 槽
    // 的 entries 不被 invalidate (RV spec 设计 — 切回原 ASID 时缓存仍可用)。
    //
    // 未来 mstatus.TVM 检查 (a_01_8 不实现; 留 a_03+):
    //   RV spec §3.1.6.5: TVM=1 + S-mode 写 satp → trap cause 2 (illegal). a_01_8 fixture
    //   不构造 TVM=1, 真做 OS 隔离时在本 helper 入口或 csr_op 入口判段加。

    uint32_t mode = (v >> 31) & 0x1u;
    uint32_t asid = ((v >> 22) & 0x1FFu) & ASID_MASK;     /* WARL 截 ASID 到 ASID_MASK 位 */
    uint32_t ppn  = v & 0x3FFFFFu;                         /* PPN 22 位, 不截 */

    hart->satp = (mode << 31) | (asid << 22) | ppn;
}


// ---- sstatus / sepc (a_01_8 Step 6 加; sret 必带的最小集) ----
//
// sstatus 是 _mstatus 的 masked view (SSTATUS_MASK = SIE | SPIE | SPP | SUM | MXR; 详见
// riscv.h SSTATUS_MASK 段); 物理存储跟 mstatus 共用 trap._mstatus, csr_sstatus_read/write
// 通过 mask 操作只访问 sstatus 视图字段位, mstatus M-mode-only 字段 (MIE/MPIE/MPP) 不被影响。
//
// sepc 物理存储 = trap.xepc[PRIV_S] (按 priv 索引数组的 [PRIV_S] 槽, 跟 mepc=xepc[PRIV_M]
// 同形态)。WARL 截 IALIGN 对齐位 (跟 mepc 同)。

static uint32_t csr_sstatus_read(cpu_t *hart) {
    /* 取 _mstatus 低 32 位 (mstatus 视图) ∩ SSTATUS_MASK = sstatus 视图 */
    return (uint32_t)(hart->trap._mstatus & 0xFFFFFFFFu) & SSTATUS_MASK;
}

static void csr_sstatus_write(cpu_t *hart, uint32_t v) {
    /* 只改 sstatus mask 内的位; 保留 _mstatus 其余字段 (M-mode-only + 高 32 位) */
    const uint64_t keep = hart->trap._mstatus & ~(uint64_t)SSTATUS_MASK;
    const uint64_t set  = (uint64_t)(v & SSTATUS_MASK);
    hart->trap._mstatus = keep | set;
}

static uint32_t csr_sepc_read(cpu_t *hart) {
    return hart->trap.xepc[PRIV_S];
}

static void csr_sepc_write(cpu_t *hart, uint32_t v) {
    /* WARL 截 IALIGN 对齐位 (跟 mepc 同; RV spec §3.1.15 / sepc 同样要求) */
    hart->trap.xepc[PRIV_S] = v & ~IALIGN_MASK;
}


// ---- tohost (a_01_8 临时 fixture 流式输出; CSR 0x800; 等 uart 实装后删除) ----
//
// 设计意图: csrw 0x800 立即 fprintf 输出到 stderr, 不存 cpu_t 字段; fixture 用作"流式
// 调试输出" 不污染 GPR (跟 spike tohost / qemu semihosting 风格类似但 user-level)。
// 跟 csr_privrd_read 同形态 — 都是"接 csr.c 入口直接 console 输出, 不缓存"。
// 任何 priv 都能 csrw/csrr (priv >= U=0; csr_op 入口判通过); 不影响 trap_csrs_t 任何字段.

static uint32_t csr_tohost_read(cpu_t *hart) {
    /* read 不缓存, 不存值; return 0 让 csrr 不 trap (csr_op 大 switch case 必须有
     * read helper, 否则 default → fprintf+trap_raise illegal). fixture 不应 csrr 0x800. */
    (void)hart;
    return 0;
}

static void csr_tohost_write(cpu_t *hart, uint32_t v) {
    /* 直接 fprintf 流式输出, 不存字段. cpu_t 不再有 tohost 字段 (a_01_8 v01 删). */
    (void)hart;
    fprintf(stderr, "[tohost] 0x%08" PRIx32 "\n", v);
}


// ---- privrd (a_01_8 临时"作弊" CSR; 0xCC0; 等 uart 实装后删除) ----
//
// 设计意图: csrr 0xCC0 立即 fprintf "[priv] X" 输出 (X = M/S/H/U 之一), 同时 return
// (uint32_t)hart->priv 让 fixture 也能 GPR 读到 (兼容性). RV spec 不允许 User-mode 直接
// 知道 priv; 项目 backdoor 用作 fixture 验证 MSU 三态切换 — fprintf 让控制台可见, GPR
// return 仍可用 (fixture 选择用 GPR 标记还是 console 输出都行)。
// RO csr — csr_op 入口判 [11:10]=0b11 时 trap 写; 没 write helper.

static char priv_to_char(uint8_t priv) {
    /* priv encoding (riscv.h PRIV_*): U=0, S=1, H=2 (项目占位 reserved), M=3 */
    static const char chars[4] = { 'U', 'S', 'H', 'M' };
    return chars[priv & 0x3u];
}

static uint32_t csr_privrd_read(cpu_t *hart) {
    fprintf(stderr, "[priv] %c\n", priv_to_char(hart->priv));
    return (uint32_t)hart->priv;
}


// ============================================================================
// csr_op —— 大 helper, decode 分发入口
//
// 演进: a_01_5_a 框架 + 注释占位入口判; a_01_5_c 6 小 helper 真读写 hart->trap;
//       a_01_7 入口判 priv/RO 真激活 (替换注释占位; 真接 trap_raise_exception)。
// ============================================================================

uint32_t csr_op(cpu_t *hart, uint32_t csr_addr, uint32_t new_val,
                csr_op_t op, uint32_t raw_inst) {
    // ----------------------------------------------------------------------------
    // 入口判: priv 要求 + RO 写检查 (csr 编号自带权限位段, riscv.h CSR_ADDR_PRIV_*)
    //
    // a_01_7 真激活 (替换 a_01_5_a 注释占位):
    //   - csr_addr[9:8] = 最低 priv 要求 (RV Privileged Spec §2.1):
    //       00=U, 01=S, 10=H/VS, 11=M
    //     hart->priv < required_priv → cause 2 illegal instruction trap
    //     注意: PRIV_M=3 是最高, 数字最大; 比较用 `<` 即可
    //   - csr_addr[11:10] = RO 标识: 0b11 表 RO; 写 RO csr → cause 2
    //     "写"判断: CSR_OP_RW 永远算写; CSR_OP_RS/RC + new_val=0 不写 (RV spec §2.1.2 副作用
    //     不发生 — 跟下方 do_write 标志一致)
    //
    // tval = raw_inst (RV spec §3.1.16: illegal instruction trap 的 mtval = 触发指令编码)
    // ----------------------------------------------------------------------------
    {
        uint32_t required_priv = (csr_addr >> CSR_ADDR_PRIV_SHIFT) & CSR_ADDR_PRIV_MASK;
        if (hart->priv < required_priv) {
            trap_raise_exception(hart, CAUSE_ILLEGAL_INSTRUCTION, raw_inst);  // _Noreturn longjmp
        }

        uint32_t is_ro = ((csr_addr >> CSR_ADDR_RO_SHIFT) & CSR_ADDR_RO_MASK)
                         == CSR_ADDR_RO_VALUE;
        uint32_t is_write = (op == CSR_OP_RW) || (new_val != 0);
        if (is_ro && is_write) {
            trap_raise_exception(hart, CAUSE_ILLEGAL_INSTRUCTION, raw_inst);  // _Noreturn longjmp
        }
    }

    // ----------------------------------------------------------------------------
    // 大 switch: 按 csr_addr 分发到具体 csr 的小 r/w helper, 算 read_old + write_back。
    //
    // 加新 csr 时只在这里加 case + 写一对 csr_<name>_<r/w> file-static helper (上方)。
    // 不存在的 csr addr 走 default: a_01_7 末解开为 fprintf 提示 + trap_raise_exception 真路径
    // (跟 lsu.h/c BARE 不在 RAM 路径同风格 — fprintf 留下 dev-friendly 定位信息, trap 走 RV
    // spec §2.1 "访问未实现 csr → illegal instruction" 路径)。
    // ----------------------------------------------------------------------------
    uint32_t read_old;
    switch (csr_addr) {
        case CSR_MSTATUS:  read_old = csr_mstatus_read (hart); break;
        case CSR_MSTATUSH: read_old = csr_mstatush_read(hart); break;
        case CSR_MTVEC:    read_old = csr_mtvec_read   (hart); break;
        case CSR_MEPC:     read_old = csr_mepc_read    (hart); break;
        case CSR_MCAUSE:   read_old = csr_mcause_read  (hart); break;
        case CSR_MTVAL:    read_old = csr_mtval_read   (hart); break;
        case CSR_SATP:     read_old = csr_satp_read    (hart); break;   /* a_01_8 */
        case CSR_SSTATUS:  read_old = csr_sstatus_read (hart); break;   /* a_01_8 Step 6 */
        case CSR_SEPC:     read_old = csr_sepc_read    (hart); break;   /* a_01_8 Step 6 */
        case CSR_TOHOST:   read_old = csr_tohost_read  (hart); break;   /* a_01_8 临时, 删除时一起 */
        case CSR_PRIVRD:   read_old = csr_privrd_read  (hart); break;   /* a_01_8 临时 RO, RO 写 trap 由入口判 */
        default:
            fprintf(stderr,
                    "[csr] unknown csr addr=0x%03" PRIx32 " → trap cause 2\n",
                    csr_addr);
            trap_raise_exception(hart, CAUSE_ILLEGAL_INSTRUCTION, raw_inst);  // _Noreturn longjmp
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
            case CSR_SATP:     csr_satp_write    (hart, to_write); break;   /* a_01_8 */
            case CSR_SSTATUS:  csr_sstatus_write (hart, to_write); break;   /* a_01_8 Step 6 */
            case CSR_SEPC:     csr_sepc_write    (hart, to_write); break;   /* a_01_8 Step 6 */
            case CSR_TOHOST:   csr_tohost_write  (hart, to_write); break;   /* a_01_8 临时 */
            default:
                // a_01_7 末: 上面 read 路径的 default 已 fprintf + trap_raise_exception
                // (_Noreturn longjmp), 控制流到此不可达; 保留 default break 作 -Wswitch
                // 默认防御 (switch on uint32_t 不受 -Wswitch-enum 约束, 但写出来更稳)。
                break;
        }
    }

    return read_old;
}
