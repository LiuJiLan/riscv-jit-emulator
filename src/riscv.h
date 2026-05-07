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
// 增量加 (按"真用到一个加一个"原则):
//   a_01_2:  V (TLB entry 有效位) + R/W/X (取指/读/写权限位; M/bare TLB fill 合成全权限)
//   a_01_8:  U (User-mode 访问标记, walker check_perm 用) + G (Global, 项目 G-agnostic 不读
//             但暴露宏让 fixture 可设) + A (Accessed, hw-managed walker 顺手 set) + D (Dirty,
//             hw-managed W 访问时 set, fast path 检查决定 fall back)
// RSW (bits 9:8) 是 sw 保留, 不暴露宏 (fixture 真用到时再加)。
// ----------------------------------------------------------------------------
#define PTE_V  (1U << 0)
#define PTE_R  (1U << 1)
#define PTE_W  (1U << 2)
#define PTE_X  (1U << 3)
#define PTE_U  (1U << 4)
#define PTE_G  (1U << 5)
#define PTE_A  (1U << 6)
#define PTE_D  (1U << 7)

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

// Supervisor-level CSRs (a_01_8 加; SV32 walker 路径真用; csr.c 大 switch 同步加 case)
//
// satp 字段布局 (Sv32; RV Privileged Spec Vol II §4.1.11 fig 4.11):
//   bit  31     = MODE  (0 = Bare 恒等映射, 1 = Sv32; 项目仅支持这两值, csr 写 helper
//                         应做 WARL 截断到 {0,1})
//   bits 30:22 = ASID  (Sv32 ASIDMAX=9 位; 项目 ASIDLEN=TLB_ASID_BITS=4, csr 写 helper
//                         必须 WARL 截断到 ASID_MASK 位 — dummy.txt §3 satp 合法性契约,
//                         dispatcher 直接按截断后的 ASID 索引 tlb_table[priv][asid] 不再
//                         做 bounds check)
//   bits 21:0  = PPN   (root page table physical page number; root PT 起点 PA = PPN << 12)
//
// 编码位段验证 (csr_addr 自带的权限位段, riscv.h CSR_ADDR_*):
//   0x180 → bits[11:10] = 01 (RW), bits[9:8] = 00 (U-min)。等等, satp 是 S-level CSR, 应该
//   是 bits[9:8] = 01 (S-min)。0x180 = 0001_1000_0000, [9:8] = 01 ✓ S-mode minimum;
//   [11:10] = 00 (RW 普通)。csr_op 入口判 priv >= S 才能访问; M 模式访问也合法 (priv 数字
//   M=3 > S=1)。
#define CSR_SATP       0x180U

// Supervisor-level CSRs (a_01_8 Step 6 加; sret 必带的最小集 — sstatus + sepc)
//
// sstatus (0x100): mstatus 的 masked view; 物理存储 = trap._mstatus 的同一份, sstatus
// 入口只能访问 SSTATUS_MASK 内的字段位 (SIE/SPIE/SPP/SUM/MXR + 未来 FS/VS/XS/SD/UBE)。
// csr_sstatus_read = _mstatus & SSTATUS_MASK; csr_sstatus_write = (_mstatus & ~MASK) |
// (v & MASK) — 不影响 mstatus M-mode-only 字段 (MIE/MPIE/MPP)。
//
// sepc (0x141): S-mode 异常 / 中断的"返回 PC"; 物理存储 = trap.xepc[PRIV_S] (按 priv 索引
// 数组的 [PRIV_S] 槽, 跟 mepc=xepc[PRIV_M] 同形态)。WARL: 同 mepc 截 IALIGN 对齐位。
// a_01_8 deliver_priv hard-code PRIV_M, trap_set_state 不写 sepc; sepc 由 fixture (sret
// 路径) 通过 csrw sepc 显式设。
//
// 不做 (留下个 session): stvec/scause/stval/sscratch (其余 S-mode CSR), medeleg, S-handler
// delivery (deliver_priv 仍 hard-code PRIV_M)。
#define CSR_SSTATUS    0x100U
#define CSR_SEPC       0x141U

// ----------------------------------------------------------------------------
// Unprivileged and User-Level Custom CSRs (a_01_8 临时, 等 uart 实装后删除)
//
// 0x800-0x8FF 范围是 RV Privileged Spec Vol II §2.1 table 2.1 "Unprivileged and
// User-Level Custom Read/Write" CSRs:
//   bits[11:10] = 0b10 → Custom RW
//   bits[9:8]   = 0b00 → U-level priv min (任何 priv 都能访问 — priv >= U=0 永远成立)
//
// CSR_TOHOST 用作 fixture 流式输出 (跟 spike tohost / qemu semihosting 风格类似):
// fixture `csrw 0x800, x..` 立即触发 fprintf "[tohost] 0x..." 输出到 stderr (csr.c
// csr_tohost_write 内直接 fprintf, 不缓存 cpu_t 字段; 跟 CSR_PRIVRD 同形态 "csrw 即输出")。
// 不污染 guest GPR (避免 fixture 用 x10/x11 标记跟 fixture 内部计算冲突)。
//
// 删除时机: a_03+ uart 实装 + ROM-based putchar 接入后, fixture 用 putchar 输出, 这条
// CSR + csr.c csr_tohost_* helper 一起删。
// ----------------------------------------------------------------------------
#define CSR_TOHOST     0x800U

// ----------------------------------------------------------------------------
// CSR_PRIVRD (a_01_8 临时, 等 uart 实装后删除) —— "作弊" 寄存器, 读当前 priv
//
// 0xCC0-0xCFF 范围是 RV Privileged Spec Vol II §2.1 table 2.1 "User-level Custom
// Read-Only" CSRs:
//   bits[11:10] = 0b11 → Custom RO
//   bits[9:8]   = 0b00 → U-level priv min (任何 priv 都能读 — priv >= U=0 永远成立)
//   bits[7:6]   = 0b11 → Custom 编号段
//
// 写检查: csr_op 入口判 [11:10]=0b11 = RO → 写时 trap cause=2; csrr (RO 路径) OK.
//
// CSR_PRIVRD 真"作弊" 用途: csrr x.., 0xCC0 立即触发 fprintf "[priv] X" 输出 (X = M/S/H/U
// 之一, csr.c csr_privrd_read 直接 fprintf 不缓存; 跟 CSR_TOHOST 同形态 "csrr 即输出"),
// 同时 return (uint32_t)hart->priv 让 GPR 也能拿到 (兼容性)。
// RV spec 不允许 User-mode 知道当前 priv; 项目 backdoor 用作 fixture 验证 MSU 三态切换
// 正确性 — 控制台直接看到 priv 字符比构造 ecall+trap+handler 路径验更直接。
//
// 删除时机: 跟 CSR_TOHOST 一起 (a_03+ uart 实装后, fixture 用 putchar + 真 trap 路径
// 验 priv 状态, 不需要这个 backdoor)。
// ----------------------------------------------------------------------------
#define CSR_PRIVRD     0xCC0U

// ----------------------------------------------------------------------------
// mstatus 物理存储 + 字段位段 (RV Privileged Spec Vol II §3.1.6)
//
// 物理存储: trap_csrs_t._mstatus (uint64_t, cpu.h 内嵌)。csr 入口拆访问 (RV32 ABI):
//   csr 0x300 mstatus  → _mstatus[31:0]  (RV32 mstatus 形态, fig 3.6)
//   csr 0x310 mstatush → _mstatus[63:32] (RV32 mstatush 形态, fig 3.7; RV64 不存在此入口)
//   csr 0x100 sstatus  → _mstatus 的 masked view (S-mode 真做时加 csr_sstatus_*)
// 未来切 RV64: csr 0x300 mstatus 直接 64 位访问 _mstatus 整体, mstatush 入口废弃;
//   部分字段位号迁移 (主要是 SD 从 bit 31 → bit 63, 加 UXL/SXL); 详见末尾 RV64-only 段。
//
// === RV32 mstatus low 32 bits (= _mstatus[31:0]; fig 3.6) ===
//   bit  0     = WPRI
//   bit  1     = SIE       (S-mode global interrupt enable)
//   bit  2     = WPRI
//   bit  3     = MIE       (M-mode global interrupt enable)
//   bit  4     = WPRI
//   bit  5     = SPIE      (saved SIE on S-trap entry; sret 时恢复)
//   bit  6     = UBE       (User-mode big-endian; 项目 LE 不真用)
//   bit  7     = MPIE      (saved MIE on M-trap entry; mret 时恢复)
//   bit  8     = SPP       (saved priv on S-trap; 1-bit; sret 时恢复)
//   bits 10:9  = VS[1:0]   (V extension state; 项目不实现 V)
//   bits 12:11 = MPP[1:0]  (saved priv on M-trap; 2-bit, 编码 PRIV_*; mret 时恢复)
//   bits 14:13 = FS[1:0]   (F/D extension state; 项目不实现 F/D)
//   bits 16:15 = XS[1:0]   (extension state aggregate)
//   bit  17    = MPRV      (Modify Privilege; load/store 用 MPP 指定的 priv 检查)
//   bit  18    = SUM       (Supervisor User Memory access; SV32 walker S-priv 路径用)
//   bit  19    = MXR       (Make eXecutable Readable; SV32 walker load 路径用)
//   bit  20    = TVM       (Trap Virtual Memory; satp / sfence.vma trap to M)
//   bit  21    = TW        (Timeout Wait; wfi 超时 trap)
//   bit  22    = TSR       (Trap SRET)
//   bit  23    = SPELP     (Supervisor Preserved ELP, Smcfilp 扩展)
//   bit  24    = SDT       (Supervisor Double-Trap, Ssdbltrp 扩展)
//   bits 30:25 = WPRI
//   bit  31    = SD (RV32) (Status Dirty; FS/VS/XS 任一 dirty 派生)
//
// === RV32 mstatush high 32 bits (= _mstatus[63:32]; fig 3.7) ===
// 物理位置在 _mstatus 64 位字段里 = mstatush 字段位号 + 32:
//   _mstatus bit 32 (mstatush[0])  = WPRI
//   _mstatus bit 36 (mstatush[4])  = SBE       (Supervisor Big-Endian)
//   _mstatus bit 37 (mstatush[5])  = MBE       (Machine Big-Endian)
//   _mstatus bit 38 (mstatush[6])  = GVA       (Guest Virtual Address; H 扩展)
//   _mstatus bit 39 (mstatush[7])  = MPV       (Machine Previous Virt mode; H 扩展)
//   _mstatus bit 41 (mstatush[9])  = MPELP     (M-mode Preserved ELP, Smcfilp)
//   _mstatus bit 42 (mstatush[10]) = MDT       (M-mode Double-Trap, Smdbltrp)
//
// === RV64-only 字段 (切 RV64 时启用; RV32 模式下这些位置或在 mstatush 是 WPRI, 或位号
//     迁移) ===
//   bits 33:32 = UXL[1:0]  (U-mode XLEN; RV64 才有; RV32 在 mstatush[1:0] 是 WPRI)
//   bits 35:34 = SXL[1:0]  (S-mode XLEN; RV64 才有; RV32 在 mstatush[3:2] 是 WPRI)
//   bit  63    = SD (RV64) (RV32 SD 在 mstatus[31]; RV64 SD 迁移到 mstatus[63])
// 其余 RV64 高位字段 (SBE/MBE/GVA/MPV/MPELP/MDT) 跟 RV32 mstatush 物理位置完全一致
// (因 RV64 mstatus 高 32 位 = RV32 mstatush 内容 + UXL/SXL 扩展 + SD 顶位迁移)。
//
// ----------------------------------------------------------------------------
// 字段宏增量原则: 真用到一个加一个 (csr.c 大 switch 同步加 case)。
//   - a_01_5_a 加: trap 路径必备 5 csr 编号 (CSR_MSTATUS / CSR_MTVEC / ... 见上方)
//   - a_01_7   加: MIE / MPIE / MPP    (trap_set_state + OP_MRET 真切 priv 用)
//   - a_01_8   加: SIE / SPIE / SPP / SUM / MXR    (SV32 walker / fixture 切 S 用)
//   - a_01_8   future-proof 加: SD (RV32 真启用 + RV64 #if 0); UXL/SXL (RV64 #if 0)
//   其余 (UBE / MPRV / TVM / TW / TSR / mstatush.SBE/MBE/GVA/MPV/MPELP/MDT 等) 真用时再加。
//
// 命名风格:
//   单 bit 字段: <field>_SHIFT + <field>            (single-bit mask)
//   多 bit 字段: <field>_SHIFT + <field>_MASK + <field>   (合成 mask)
//   shift > 31 的字段 (mstatush 内容 / RV64 高位): 用 1ULL 替代 1U (因 _mstatus 是 64 位)
// ----------------------------------------------------------------------------
#define MSTATUS_MIE_SHIFT   3U
#define MSTATUS_MIE         (1U << MSTATUS_MIE_SHIFT)        /* = 0x00000008 */

#define MSTATUS_MPIE_SHIFT  7U
#define MSTATUS_MPIE        (1U << MSTATUS_MPIE_SHIFT)       /* = 0x00000080 */

#define MSTATUS_MPP_SHIFT   11U
#define MSTATUS_MPP_MASK    0x3U                              /* 2-bit field */
#define MSTATUS_MPP         (MSTATUS_MPP_MASK << MSTATUS_MPP_SHIFT)  /* = 0x00001800 */

// a_01_8 增量 (SV32 walker / sret / fixture 切 S 路径真用; 命名风格跟 MIE/MPIE/MPP 一致):
//   SIE  (bit  1): S-mode global interrupt enable。a_01_8 中断不真做 (留 a_03+); 字段先暴露,
//                   让 fixture csrw mstatus, x.. 设值时不被 WARL 误截断丢。
//   SPIE (bit  5): saved SIE on S-mode trap entry (sret 时恢复 SIE = SPIE)。a_01_8 不接 sret,
//                   字段先暴露。
//   SPP  (bit  8): saved priv on S-mode trap entry (1-bit; sret 时 priv = SPP)。a_01_8 fixture
//                   走 mret 切 S (mstatus.MPP = PRIV_S), 不走 sret, 但字段暴露不亏。
//   SUM  (bit 18): permit Supervisor User Memory access。SV32 walker 在 S 模式访问 PTE.U=1
//                   page 时按本位决定: SUM=0 → 不放过 (cause 13/15 page fault); SUM=1 → 放过
//                   (S 模式可读写 U-page)。默认 SUM=0 (跟 fixture "S 不该碰 U-page" 直觉一致)。
//   MXR  (bit 19): make eXecutable Readable。SV32 walker load 路径按本位决定:
//                   MXR=0 → 严格按 PTE.R 位 (X=1 R=0 page 不可读, 触发 cause 13);
//                   MXR=1 → PTE.X=1 page 也算可读 (X 兼任 R)。默认 MXR=0。
#define MSTATUS_SIE_SHIFT   1U
#define MSTATUS_SIE         (1U << MSTATUS_SIE_SHIFT)        /* = 0x00000002 */

#define MSTATUS_SPIE_SHIFT  5U
#define MSTATUS_SPIE        (1U << MSTATUS_SPIE_SHIFT)       /* = 0x00000020 */

#define MSTATUS_SPP_SHIFT   8U
#define MSTATUS_SPP         (1U << MSTATUS_SPP_SHIFT)        /* = 0x00000100 */

#define MSTATUS_SUM_SHIFT   18U
#define MSTATUS_SUM         (1U << MSTATUS_SUM_SHIFT)        /* = 0x00040000 */

#define MSTATUS_MXR_SHIFT   19U
#define MSTATUS_MXR         (1U << MSTATUS_MXR_SHIFT)        /* = 0x00080000 */

// ----------------------------------------------------------------------------
// SSTATUS_MASK (a_01_8 Step 6 加) —— sstatus 是 mstatus 的 masked view
//
// sstatus 入口可访问 _mstatus 的字段位; mstatus M-mode-only 字段 (MIE/MPIE/MPP/TVM/TW/TSR
// 等) 不在 sstatus 视图内, 写入忽略读出 0。
//
// 项目当前真用的 S-mode 字段: SIE / SPIE / SPP / SUM / MXR (Step 1 已暴露宏); 跟 a_01_8
// fixture (a) sret 切 priv + walker SUM/MXR 配套。其他 S-mode 字段 (UBE bit 6, VS bits 10:9,
// FS bits 14:13, XS bits 16:15, SD bit 31) 项目不实现 F/D/V/XS, mask 暂不覆盖 — 跟"真用到
// 一个加一个" 原则一致; 未来真做 F/D/V 扩展时把对应位加进 mask。
//
// 注: SD 是 read-only derived from FS/VS/XS dirty 状态 (RV spec); 严格说 sstatus 应能 read
// 到 SD 但 write 时 ignore — 项目不实现 FS/VS/XS, SD 永远 0, 不进 mask 也不影响行为。
#define SSTATUS_MASK   (MSTATUS_SIE | MSTATUS_SPIE | MSTATUS_SPP | MSTATUS_SUM | MSTATUS_MXR)

// ----------------------------------------------------------------------------
// future-proof 字段宏 (a_01_8 加; user 主导拍, 见 a_01_session_011 — RV64 切换时启用)
//
// 原则: _mstatus 物理 64 位, 但当前 RV32 ABI 拆 mstatus + mstatush 入口暴露; 字段位号
// 在 RV32 / RV64 视角下不一样的, 同时给出两版宏 (RV64 版用 #if 0 包占位)。a_01_8 真用的
// 5 个字段 (SIE/SPIE/SPP/SUM/MXR) 在 RV32 / RV64 位置一致, 已在上方加宏, 不需要双版本。
//
// 当前 a_01_8 不真激活 SD / UXL / SXL (fixture 不构造 FS/VS dirty, 也不切 RV64);
// 字段宏先暴露便于未来切 RV64 / 接 F/D 扩展时一处启用, 不需要回头加。
// ----------------------------------------------------------------------------

// SD (Status Dirty): RV32 在 mstatus[31], RV64 在 mstatus[63]。
#define MSTATUS_SD_RV32_SHIFT   31U
#define MSTATUS_SD_RV32         (1U << MSTATUS_SD_RV32_SHIFT)         /* = 0x80000000 */

#if 0  /* RV64 切换时启用 */
#define MSTATUS_SD_RV64_SHIFT   63U
#define MSTATUS_SD_RV64         (1ULL << MSTATUS_SD_RV64_SHIFT)
#endif

// UXL / SXL: RV64 才有的 XLEN 字段; RV32 同位置在 mstatush[1:0] / mstatush[3:2] 是 WPRI。
// _mstatus 64 位字段里 UXL = bits[33:32], SXL = bits[35:34]。
#if 0  /* RV64 切换时启用 */
#define MSTATUS_UXL_SHIFT       32U
#define MSTATUS_UXL_MASK        0x3ULL                                  /* 2-bit field */
#define MSTATUS_UXL             (MSTATUS_UXL_MASK << MSTATUS_UXL_SHIFT)

#define MSTATUS_SXL_SHIFT       34U
#define MSTATUS_SXL_MASK        0x3ULL
#define MSTATUS_SXL             (MSTATUS_SXL_MASK << MSTATUS_SXL_SHIFT)
#endif

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
