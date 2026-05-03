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
//   - CSR 编号 (csr.c 大 switch 时一次填全)
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

#endif //RISCV_H
