//
// Created by liujilan on 2026/5/2.
//
// RISC-V 架构级别定义。集中收纳"来自规范的 encoding / 位段 / mask",避免在各模块里
// 散用裸数字 (`3` = M 模式之类)。
//
// 与 config.h 的职能划分:
//   - config.h: 项目自身的全局配置 (GUEST_RAM_*, TLB_*, ASID_*) —— "我们怎么配"
//   - riscv.h:  RISC-V 规范定义 (priv 编码, CSR 地址, PTE 位段 等) —— "规范怎么定"
//
// 增量原则: 不一次到位; 每条按"哪个模块第一次需要"加进来, 配 1-2 行 spec 引文。
//
// 当前 (a01_2) 收纳:
//   - PRIV_U / PRIV_S / PRIV_M (cpu.c / dispatcher / tlb.h 都要用)
//
// 后续按需增量加 (示意, 真到那一步再加):
//   - SATP_MODE_BARE / SATP_MODE_SV32 + SATP_MODE_SHIFT (csr.c 读写 satp.MODE 时)
//   - MSTATUS_MPP / SSTATUS_SPP / MIE / SIE / 中断使能位 (csr.c)
//   - PTE_V / PTE_R / PTE_W / PTE_X / PTE_U / PTE_G / PTE_A / PTE_D (mmu walker)
//   - CSR 编号 (csr.c 大 switch 时增量加 — a_01_5_a 加 trap 5 个; OS / S-mode 时再加 sstatus / sepc / sscratch / scause / stval / stvec / sip / sie 等)
//

#ifndef RISCV_H
#define RISCV_H

// ----------------------------------------------------------------------------
// Privilege mode encoding
// RISC-V Privileged Spec Vol II, table 1.1
//
//   00 = User       (U)
//   01 = Supervisor (S)
//   10 = Reserved   (规范本身留空; 项目内部 tlb_table[2] 占用 → 见 PRIV_H 警告)
//   11 = Machine    (M)
// ----------------------------------------------------------------------------
#define PRIV_U  0U
#define PRIV_S  1U
#define PRIV_H  2U      // 项目内部占位, 不是规范定义的 priv 编码!
                        //
                        // 用途: 只为给 tlb_table[2] 这个槽位提供宏化命名, 避免 PRIV_U/S/M
                        //       已经宏化但 [2] 用裸 2 这种混用风格 (未来读代码会混淆)。
                        //
                        // 不严谨之处:
                        //   - 规范将编码 2 标记为 Reserved
                        //   - H 扩展通过 V 位 + S 级别 (V=1, priv=PRIV_S) 表达 VS-mode,
                        //     不在 priv 字段直接放 2
                        //
                        // 当前 a_01: cpu->priv 只取 PRIV_M, PRIV_H 不会真被赋值给 priv。
                        //           真上 H 扩展时, 这个宏要么换为基于 V 位的派发逻辑,
                        //           要么改名为 PRIV_VS_SLOT 之类来强调 "槽位索引" 而非
                        //           "priv 编码值" 语义。
#define PRIV_M  3U

// ----------------------------------------------------------------------------
// Sv32 PTE 位段 (Privileged Spec Vol II, table 4.18 / fig 4.16-4.18)
//
// PTE bit:  9 8 7 6 5 4 3 2 1 0
//          [RSW][D][A][G][U][X][W][R][V]
//
// 增量加: 当前用到 V (TLB entry 有效位) + R/W/X (取指/读/写权限位; M/bare TLB fill 合成全权限);
// U/G/A/D 后续真用到时 (Sv32 walker 接入) 再加宏化。
// ----------------------------------------------------------------------------
#define PTE_V  (1U << 0)
#define PTE_R  (1U << 1)
#define PTE_W  (1U << 2)
#define PTE_X  (1U << 3)

// ----------------------------------------------------------------------------
// CSR 编号 (Privileged Spec Vol II, table of Machine-Level CSRs / §2.2)
//
// 增量原则: 真用到一个加一个 (csr.c 大 switch 同步加 case)。a_01_5_a 加 trap 路径用的
// 5 个 csr (mstatus 物理 64 位 = mstatus + mstatush 双 csr 入口)。
//
// 12-bit csr 地址位段 (RV Privileged Spec §2.1):
//   bits [11:10] = 访问性质: 00/01/10 = RW, 11 = RO  (csr.c 入口判 RO 写 → trap)
//   bits [9:8]   = 最低特权要求: 00=U, 01=S, 10=H/VS, 11=M  (csr.c 入口判 priv < 要求 → trap)
//   bits [7:0]   = 在该 (RW/RO, priv) 范围内的编号
// ----------------------------------------------------------------------------

// 编码位段 (csr.c 入口判权限 / RO 用)
#define CSR_ADDR_PRIV_SHIFT  8U
#define CSR_ADDR_PRIV_MASK   0x3U          /* (addr >> 8) & 3 == 最低 priv 要求 */
#define CSR_ADDR_RO_SHIFT    10U
#define CSR_ADDR_RO_MASK     0x3U          /* (addr >> 10) & 3 == 0b11 时为 RO */
#define CSR_ADDR_RO_VALUE    0x3U          /* (addr >> 10) & 3 == 0x3 → 只读 */

// Machine-level trap setup / handling CSRs (a_01_5_a 加; trap 路径必备)
//
// 注: mstatus 在 RV32 物理 64 位, RV 规范拆 mstatus (低 32) + mstatush (高 32) 两个 csr
//     入口访问。本项目 cpu_t 内只存一份 _mstatus (uint64_t), csr.c 的 mstatus / mstatush
//     read/write 都映射到同一个物理字段的不同半边 (dummy.txt §1 第一类). sstatus = 0x100
//     是 mstatus 的 masked view, a_01_5_a 不实现, 等真做 S-mode 时加。
#define CSR_MSTATUS    0x300U          /* mstatus 低 32 位 (MIE/MPIE/MPP/SUM/MXR/...) */
#define CSR_MTVEC      0x305U          /* M-mode trap vector base (+ MODE 低 2 位) */
#define CSR_MSTATUSH   0x310U          /* mstatus 高 32 位 (RV32 only; SBE/MBE 等大端控制) */
#define CSR_MEPC       0x341U          /* M-mode 异常 / 中断的"返回 PC" */
#define CSR_MCAUSE     0x342U          /* M-mode 触发 trap 的 cause code */
#define CSR_MTVAL      0x343U          /* M-mode trap 附属信息 (cause-specific) */

// ----------------------------------------------------------------------------
// mstatus 字段位段 (RV Privileged Spec Vol II, fig 3.6 mstatus register)
//
// a_01_7 增量: 只展开 trap_set_state / OP_MRET 真切 priv 用到的字段:
//   MIE  (bit  3): M-mode global interrupt enable
//   MPIE (bit  7): saved MIE on trap entry; mret 时恢复 MIE = MPIE
//   MPP  (bits 11:10? 不, 11:11 +12; 实际 MPP 占 bits 12:11 即 2-bit 字段, shift=11):
//                  saved priv on trap entry; mret 时 priv = MPP
//
// 实际 mstatus bit 编号 (RV Spec):
//   bit  0  = SD (Status Dirty, RV32 dirty 派生); a_01 不实现 F/D, SD=0
//   bit  1  = SIE (S-mode interrupt enable, S-mode 真做时加)
//   bit  3  = MIE
//   bit  5  = SPIE
//   bit  7  = MPIE
//   bit  8  = SPP (1-bit, S-mode 真做时加)
//   bits 12:11 = MPP (2-bit, 编码 PRIV_U=00, PRIV_S=01, PRIV_M=11; PRIV_H=10 在没 H 扩展时
//                      非法, 项目 WARL 落 PRIV_U=0 — csr.c csr_mstatus_write 内做)
//   bit 17 = SUM (S-mode 真做 SV32 时加)
//   bit 19 = MXR (S-mode 真做 SV32 时加)
//   ... 其余 (FS/VS/XS/UBE/SBE/MBE/MPV/...) a_01 全部 0, 相关 fixture 不构造非零写入
//
// 增量原则: SIE/SPIE/SPP/SUM/MXR 留 a_01_8 SV32 真用时再展开; 当前 a_01_7 只暴露 trap 必需的
// MIE/MPIE/MPP 三组宏。
//
// 命名风格: <field>_SHIFT (bit position) + <field> (single-bit mask) 单 bit 字段;
//            <field>_SHIFT + <field>_MASK 多 bit 字段。
// ----------------------------------------------------------------------------
#define MSTATUS_MIE_SHIFT   3U
#define MSTATUS_MIE         (1U << MSTATUS_MIE_SHIFT)        /* = 0x00000008 */

#define MSTATUS_MPIE_SHIFT  7U
#define MSTATUS_MPIE        (1U << MSTATUS_MPIE_SHIFT)       /* = 0x00000080 */

#define MSTATUS_MPP_SHIFT   11U
#define MSTATUS_MPP_MASK    0x3U                              /* 2-bit field */
#define MSTATUS_MPP         (MSTATUS_MPP_MASK << MSTATUS_MPP_SHIFT)  /* = 0x00001800 */

// ----------------------------------------------------------------------------
// Exception Code (RV Privileged Spec Vol II, table 3.6 sync exception code)
//
// 用途: trap_raise_exception / trap_set_state 的 cause 参数 + mcause/scause 字段值。
// 命名: CAUSE_<event> (跟 Spike riscv-sim CAUSE_* 一致; 跟 mcause 字段名直接对应)。
//
// 部分编号 (10/14/17/20+) 是 RV spec reserved, 项目不暴露宏 (避免误用); 真用到时再加。
// 部分扩展 trap (16 Smdbltrp double trap, 18 Smcsrind software check, 19 hardware error)
// 项目暂不实现, 但宏先列出方便未来引用 + grep 时直接知道编号语义。
//
// trap_raise_exception 调用方应该用 CAUSE_* 宏 (不再写裸数字), 跟 PRIV_* / MSTATUS_* 风格一致。
// ----------------------------------------------------------------------------
#define CAUSE_INST_ADDR_MISALIGNED   0U   /* Instruction address misaligned   */
#define CAUSE_INST_ACCESS_FAULT      1U   /* Instruction access fault         */
#define CAUSE_ILLEGAL_INSTRUCTION    2U   /* Illegal instruction              */
#define CAUSE_BREAKPOINT             3U   /* Breakpoint                       */
#define CAUSE_LOAD_ADDR_MISALIGNED   4U   /* Load address misaligned          */
#define CAUSE_LOAD_ACCESS_FAULT      5U   /* Load access fault                */
#define CAUSE_STORE_ADDR_MISALIGNED  6U   /* Store/AMO address misaligned     */
#define CAUSE_STORE_ACCESS_FAULT     7U   /* Store/AMO access fault           */
#define CAUSE_ECALL_FROM_U           8U   /* Environment call from U-mode     */
#define CAUSE_ECALL_FROM_S           9U   /* Environment call from S-mode     */
/* 10: Reserved */
#define CAUSE_ECALL_FROM_M          11U   /* Environment call from M-mode     */
#define CAUSE_INST_PAGE_FAULT       12U   /* Instruction page fault           */
#define CAUSE_LOAD_PAGE_FAULT       13U   /* Load page fault                  */
/* 14: Reserved */
#define CAUSE_STORE_PAGE_FAULT      15U   /* Store/AMO page fault             */
#define CAUSE_DOUBLE_TRAP           16U   /* Double trap (Smdbltrp 扩展, 项目不实现) */
/* 17: Reserved */
#define CAUSE_SOFTWARE_CHECK        18U   /* Software check                   */
#define CAUSE_HARDWARE_ERROR        19U   /* Hardware error                   */
/* 20+: Reserved / Designated for custom use */

// 巧合: PRIV_U=0 / PRIV_S=1 / PRIV_M=3 与 ECALL cause 8/9/11 差 8 (PRIV_H=2 跟 cause 10
// reserved 对应也"巧合"); 调用方可写 (CAUSE_ECALL_FROM_U + hart->priv) 一行覆盖三 priv。
// Spike / QEMU 同写法 (跟 RV spec 编码巧合一致, 不是项目自定的"魔法")。

#endif //RISCV_H
