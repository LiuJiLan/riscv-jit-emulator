# a_01_8 / 01_sret_m_s_u_chain
# 验证 sret 切 priv (S→U) + sstatus / sepc CSR 入口 (Step 6 加).
#
# 路径 (user "M→S→U→M (没代理, 先同一回 M)" — 不依赖 SV32 walker, 不用 lw/sw):
#   1. M-mode 起步: 设 mtvec / mepc / mstatus.MPP=PRIV_S → mret → 切 S
#   2. S-mode: 设 sepc / sstatus.SPP=PRIV_U → sret → 切 U
#   3. U-mode: ecall (cause=8 from U) → trap deliver M (deliver_priv hard-code PRIV_M)
#   4. M-handler: 验 cause=8; csrw mtvec=0 + ecall 触发 triple fault 退出 dispatcher
#
# 覆盖范围:
#   1. sret 切 priv (S→U): 验 _mstatus.SPP/SPIE/SIE 字段操作 + sepc 拿 PC
#   2. sstatus CSR 入口 (csrw sstatus 写 mask 内字段; csrw sepc 写 sepc)
#   3. mret 切 priv (M→S): 跟 a_01_7 mret 切 U 同形态 (不同 MPP 值)
#   4. 跨 priv ecall: U-mode ecall 走 cause=8 (跟 a_01_7/04 cause=8 路径一致, 但走 sret)
#
# 22 条指令 (la×2 算 4 条) = 88 字节 (32-bit only, .option norvc).
#
# 字节地址布局 (.text 钉 0x80000000):
#   0x00 la x4, m_handler   (8B; gcc auipc+addi)
#   0x08 csrw mtvec, x4
#   0x0c la x5, s_program   (8B)
#   0x14 csrw mepc, x5
#   0x18 addi x6, x0, 1     (PRIV_S 编码值 = 1)
#   0x1c slli x6, x6, 11    (x6 = 1 << 11 = 0x800, MPP_S 在 mstatus bits 12:11)
#   0x20 csrw mstatus, x6
#   0x24 mret               (M→S; priv=MPP=S, MIE=MPIE=0, MPIE=1, MPP=U)
#   ---- s_program (0x28) ----
#   0x28 addi x10, x0, 1    (S 起步标记)
#   0x2c la x7, u_program   (8B)
#   0x34 csrw sepc, x7
#   0x38 csrw sstatus, x0   (SPP=PRIV_U=0; sstatus 整体清 0 — sret 切 U 关键)
#   0x3c sret               (S→U; priv=SPP=U, SIE=SPIE=0, SPIE=1, SPP=U=0; pc=sepc)
#   ---- u_program (0x40) ----
#   0x40 addi x11, x0, 2    (U 起步标记 — sret 真生效证据)
#   0x44 ecall              (cause=8 from U; trap deliver M)
#   ---- m_handler (0x48) ----
#   0x48 csrr x12, mcause   (期望 8)
#   0x4c addi x13, x0, 8
#   0x50 bne x12, x13, halt (offset = halt - 0x50 = 4; 实际 fall through 即可, 但
#                             加这条是防御 — 万一 cause != 8 也跳到 halt 停机)
#   ---- halt (0x54) ----
#   0x54 csrw mtvec, x0     (mtvec=0)
#   0x58 ecall              (cause=11 from M; trap → mtvec=0 → fetch 0 fail → triple fault)
#
# 期望 (main.c dump):
#   x10 = 1                  (S 起步标记)
#   x11 = 2                  (U 起步标记 — sret 真生效证据!)
#   x12 = 8                  (mcause from U ecall, csrr 进 handler 时读)
#   x13 = 8                  (比较寄存器)
#   pc = 0                   (handler 末 ecall trap 跳 mtvec=0; 第 3 次 trap 候选 A 早 return)
#   in_trap = 3              (triple fault)
#   mcause = 11              (handler 末 ecall = ecall from M; 字段保留第 2 次)
#   mtval = 0                (ecall tval=0)
#   mepc = 0x80000058        (handler 末 ecall PC)
#   mtvec = 0                (handler 自己清的)
#   priv = 3                 (M; deliver_priv hard-code PRIV_M)
#   mstatus = 0x1820         (推算: 详见末段; 实际跑出来核对)
#     - MPP = M = 0b11 << 11 = 0x1800   (handler 末 ecall trap 设 MPP=caller=M)
#     - MPIE = 0                          (旧 MIE = 0 → MPIE = 0)
#     - MIE = 0                           (trap 入口 disable interrupt)
#     - SPP = 0                           (sret 末段 reset)
#     - SPIE = 1                          (sret 末段设; deliver M 路径不动 S 字段, 残留)
#     - 0x1800 | 0x20 = 0x1820 ✓
#   total_count: 待算 (跑出来核对)
#
# mstatus 推算时序 (注释方便 debug):
#   阶段 1 起步:                          mstatus = 0
#   阶段 2 csrw mstatus, 0x800:          mstatus = 0x800   (MPP=01=S)
#   阶段 3 mret (M→S, MPP→priv 等):       mstatus = 0x80    (MPP=U=0; MPIE=1; MIE=0)
#   阶段 4 csrw sstatus, x0:              mstatus = 0x80    (SSTATUS_MASK 不覆盖 MPIE)
#   阶段 5 sret (S→U):                    mstatus = 0xA0    (SPIE=1; SPP=U=0; +MPIE 残留)
#   阶段 6 ecall (U→M trap, deliver M):   mstatus = 0x20    (MPP=caller=U=0; MPIE=MIE=0;
#                                                              MIE=0; SPIE 残留)
#   阶段 7 handler 跑 (csrr/addi/bne):    mstatus = 0x20    (不动)
#   阶段 8 ecall (M→M trap):              mstatus = 0x1820  (MPP=caller=M=3; MPIE=MIE=0;
#                                                              MIE=0; SPIE 残留)
#   阶段 9 fetch 0 fail (in_trap 2→3):    mstatus = 0x1820  (候选 A 早 return 不动字段)
#
# 关键验证点:
#   - x11 = 2 → sret 真生效 (没 sret 跑不到 0x40)
#   - x12 = 8 → cause=8 from U (ecall_from_U 跟 a_01_7 cause=8 编码一致)
#   - mstatus.SPIE = 1 → sret 末段把 SPIE 设 1 (RV spec)
#   - mstatus.SPP = 0 → sret 末段把 SPP reset 为 PRIV_U=0

.section .text
.option norvc
.globl _start
_start:
    # ---- M-mode 起步 ----
    la   x4, m_handler      # 0x00-0x07: la (8 字节)
    csrw mtvec, x4          # 0x08

    la   x5, s_program      # 0x0c-0x13: la (8 字节)
    csrw mepc, x5           # 0x14

    addi x6, x0, 1          # 0x18: x6 = 1 (PRIV_S 编码)
    slli x6, x6, 11         # 0x1c: x6 = 0x800 (MPP_S)
    csrw mstatus, x6        # 0x20: mstatus = 0x800 (MPP=01=PRIV_S, 其他 0)

    mret                    # 0x24: M→S, pc=mepc=s_program

s_program:
    # ---- S-mode 起步标记 ----
    addi x10, x0, 1         # 0x28: x10 = 1

    # ---- 设 sret 目标 = u_program ----
    la   x7, u_program      # 0x2c-0x33: la (8 字节)
    csrw sepc, x7           # 0x34: sepc = u_program

    # ---- 设 sstatus.SPP = PRIV_U = 0 (sret 切 U 关键) ----
    # csrw sstatus, x0 写整个 sstatus = 0; SPP=0 即 PRIV_U; SIE=SPIE=SUM=MXR=0;
    # 不影响 mstatus 的 M-mode-only 字段 (MPP/MPIE/MIE 通过 SSTATUS_MASK 保护)
    csrw sstatus, x0        # 0x38

    sret                    # 0x3c: S→U, pc=sepc=u_program

u_program:
    # ---- U-mode 起步标记 (sret 真生效证据) ----
    addi x11, x0, 2         # 0x40: x11 = 2

    # ---- 触发 ecall (cause=8 from U) → trap deliver M ----
    ecall                   # 0x44

    # ---- 不到此处 ----

m_handler:
    # ---- M handler: 验 cause=8 ----
    csrr x12, mcause        # 0x48: x12 = mcause (期望 8)
    addi x13, x0, 8         # 0x4c: x13 = 8
    bne  x12, x13, halt     # 0x50: x12 != 8 → halt (防御; 期望 fall through 因 cause=8)

halt:
    # ---- 停机: csrw mtvec=0 + ecall 触发 triple fault 退出 dispatcher ----
    csrw mtvec, x0          # 0x54: mtvec = 0
    ecall                   # 0x58: ecall from M (cause=11) → 跳 0 → mmu fail → triple fault
