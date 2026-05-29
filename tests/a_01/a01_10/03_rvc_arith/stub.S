# a_01_10 / 03_rvc_arith
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验 RVC 算术 + 逻辑 + 移位 case (~16 case, a_01_10 (7) step 1)。
# (注: 原收尾靠 ecall → mtvec=0 fetch fail → triple fault (in_trap=3); a_03 Smdbltrp 后
#  首个 trap 撞 MDT=1 即 abort, 改在 RVC 序列末尾追加 test_dev 干净停机替代跑飞。
#  RVC 算出的寄存器值在停机前已落定, test_dev 停机不影响。原 in_trap=3 / mcause=1
#  "二次 fetch 0 fail" 收尾叙事见 REVIEW B.3 段。)
# 不含 sp-relative load/store: C.ADDI4SPN / C.LWSP / C.SWSP 留 fixture 06。
# 控制流 (C.JAL/J/JR/JALR/BEQZ/BNEZ/EBREAK) 留 fixture 04-05。
#
# ===== 思路 =====
# 模板风格: 纯 .S, BARE M-mode (无 SV32 / 无 PT / 无 m_handler), 套 a_01_3/02_arith_compressed。
# .option norvc 强制非 c.* 指令走 32-bit (li / ecall 等); c.* mnemonic 强制 RVC 编码。
#
# 寄存器分配 (跟 main.c dump 范围对齐):
#   - 5-bit RVC (C.LI/C.ADDI/C.LUI/C.SLLI/C.MV/C.ADD): 用 x16-x21 (a_01_10 main.c 扩 dump 范围)
#   - 3-bit RVC (C.ANDI/C.SRLI/C.SRAI/C.SUB/C.XOR/C.OR/C.AND): 用 x8-x15 (8 reg, 含 rs2 共用)
#   - sp 系列 (C.ADDI16SP): 用 x2 (sp)
#   - C.NOP 不占 reg, 仅验 fetch +=2 推进
#
# rs2 共用哲学:
#   C.SUB/XOR/OR/AND 的 rs2 都共用 x9 = 5 (固定输入), rd 各用 x12-x15 一个 final 值,
#   每条指令 rd 不同, dump 时各自验。
#
# 16 case (展开后):
#   1.  C.NOP                  → ADDI x0, x0, 0     (raw 0x0001)
#   2.  C.LI    x16, 8         → ADDI x16, x0, 8
#   3.  C.ADDI  x16, 3         → ADDI x16, x16, 3   (x16 = 8+3 = 11)
#   4.  C.LI    x17, -1        → ADDI x17, x0, -1   (sign-ext)
#   5.  C.LUI   x18, 0x10      → LUI  x18, 0x10     (x18 = 0x10 << 12 = 0x10000)
#   6.  C.LI    x19, 1         → ADDI x19, x0, 1
#   7.  C.SLLI  x19, 4         → SLLI x19, x19, 4   (x19 = 1<<4 = 16)
#   8.  C.MV    x20, x19       → ADD  x20, x0, x19  (x20 = 16)
#   9.  C.LI    x21, 16        → ADDI x21, x0, 16
#   10. C.ADD   x21, x19       → ADD  x21, x21, x19 (x21 = 16+16 = 32)
#   11. C.LI    x2, 0          → ADDI x2, x0, 0     (sp = 0)
#   12. C.ADDI16SP sp, 64      → ADDI x2, x2, 64    (sp = 64)
#   13. C.LI    x9, 5          → ADDI x9, x0, 5     (rs2 共用)
#   14. C.LI    x8, -1         → ADDI x8, x0, -1
#   15. C.ANDI  x8, 15         → ANDI x8, x8, 15    (x8 = -1 & 0xF = 0xF)
#   16. C.LI    x10, 16
#   17. C.SRLI  x10, 2         → SRLI x10, x10, 2   (x10 = 16 >> 2 = 4)
#   18. C.LI    x11, -16
#   19. C.SRAI  x11, 2         → SRAI x11, x11, 2   (x11 = -16 >> 2 = -4 sign 保留)
#   20. C.LI    x12, 12
#   21. C.SUB   x12, x9        → SUB  x12, x12, x9  (x12 = 12-5 = 7)
#   22. C.LI    x13, 31        → ADDI x13, x0, 31   (c.li imm6 max+ = 31)
#   23. C.XOR   x13, x9        → XOR  x13, x13, x9  (x13 = 31^5 = 26)
#   24. C.LI    x14, 26
#   25. C.OR    x14, x9        → OR   x14, x14, x9  (x14 = 26|5 = 31)
#   26. C.LI    x15, 12
#   27. C.AND   x15, x9        → AND  x15, x15, x9  (x15 = 12&5 = 4)
#   28. ecall                  → OP_ECALL (cause=11) → trap chain in_trap=3 halt

.section .text
                                    # 不写 .option (跟 a_01_3/02_arith_compressed 同) — 默认靠
                                    # -march=rv32imac 允许 c.* mnemonic; .option norvc 会禁 RVC
                                    # 编码导致 c.* mnemonic illegal operands; .option rvc 等价
                                    # 默认 (rvc 在 -march 已含)。li / ecall 是 pseudo / 没 RVC
                                    # 等价, GAS 自然走 32-bit, 不会被压缩。
.globl _start
_start:
    # ---- C.NOP (验 fetch loop +=2 推进, 不影响 GPR) ----
    c.nop                           # 0x0001 = ADDI x0, x0, 0

    # ---- 5-bit RVC 算术 (用 x16-x21) ----
    c.li      x16, 8                # x16 = 8 (验 C.LI 正)
    c.addi    x16, 3                # x16 = 8 + 3 = 11 (验 C.ADDI; 期望 x16 = 0x0B)

    c.li      x17, -1               # x17 = -1 (验 C.LI 负 sign-ext; 期望 x17 = 0xFFFFFFFF)

    c.lui     x18, 0x10             # x18 = 0x10 << 12 = 0x10000 (验 C.LUI; 期望 x18 = 0x10000)

    c.li      x19, 1
    c.slli    x19, 4                # x19 = 1 << 4 = 16 (验 C.SLLI; 期望 x19 = 0x10)

    c.mv      x20, x19              # x20 = 16 (验 C.MV; 期望 x20 = 0x10)

    c.li      x21, 16
    c.add     x21, x19              # x21 = 16 + 16 = 32 (验 C.ADD; 期望 x21 = 0x20)

    # ---- C.ADDI16SP (sp 系列, 用 x2) ----
    c.li      x2, 0                 # sp = 0 (清零, 避免 random)
    c.addi16sp sp, 64               # sp = 0 + 64 = 64 (验 C.ADDI16SP; 期望 x2 = 0x40 = 64)

    # ---- 3-bit RVC 算术 / 逻辑 / 移位 (用 x8-x15, rs2=x9 共用) ----
    c.li      x9, 5                 # x9 = 5 (rs2 for SUB/XOR/OR/AND; 期望 x9 = 0x05)

    c.li      x8, -1
    c.andi    x8, 15                # x8 = -1 & 0x0F = 0xF (验 C.ANDI; 期望 x8 = 0x0F)

    c.li      x10, 16
    c.srli    x10, 2                # x10 = 16 >> 2 = 4 (验 C.SRLI; 期望 x10 = 4 decimal)

    c.li      x11, -16
    c.srai    x11, 2                # x11 = -16 >> 2 = -4 (sign 保留; 期望 x11 = 4294967292)

    c.li      x12, 12
    c.sub     x12, x9               # x12 = 12 - 5 = 7 (验 C.SUB; 期望 x12 = 7)

    c.li      x13, 31               # c.li imm6 max+ = 31
    c.xor     x13, x9               # x13 = 31 ^ 5 = 26 (验 C.XOR; 期望 x13 = 26)

    c.li      x14, 26
    c.or      x14, x9               # x14 = 26 | 5 = 31 (验 C.OR; 期望 x14 = 31)

    c.li      x15, 12
    c.and     x15, x9               # x15 = 12 & 5 = 4 (验 C.AND; 期望 x15 = 4)

    # ---- 停机: test_dev FINISHER PASS (替代原跑飞进 fetch-fault) ----
    li   x30, 0x100000              # test_dev base
    li   x31, 0x5555                # FINISHER PASS magic
    sw   x31, 0(x30)                # 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望 main.c dump (假设跑通):
#   x2 = 64 (sp, 0x40)
#   x8 = 0x0F (ANDI: -1 & 0xF)
#   x9 = 0x05 (LI 5; rs2 共用 for SUB/XOR/OR/AND; 不被它们改)
#   x10 = 4 (SRLI; decimal)
#   x11 = 4294967292 (SRAI -4 unsigned; decimal)
#   x12 = 7 (SUB; decimal)
#   x13 = 26 (XOR; decimal)
#   x14 = 31 (OR; decimal)
#   x15 = 4 (AND; decimal)
#   x16 = 0x0000000B (LI 8 + ADDI 3 = 11)
#   x17 = 0xFFFFFFFF (LI -1 sign-ext)
#   x18 = 0x00010000 (LUI 0x10 << 12)
#   x19 = 0x00000010 (LI 1 + SLLI 4 = 16)
#   x20 = 0x00000010 (MV from x19)
#   x21 = 0x00000020 (LI 16 + ADD x19 = 32)
#   x30 = 0x100000 / x31 = 0x5555 (停机 scratch: test_dev base / FINISHER PASS magic)
#   停机 = test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
#
# 关键验证点:
#   - x10..x15 各 final 值符合 RVC 解码 + interpreter 执行结果
#   - x16..x21 各 final 值符合 5-bit RVC 算术
#   - x2 = 64 验 C.ADDI16SP
#   - x8 / x9 (hex dump) 符合 ANDI / rs2 共用值
# (注: 绝对 pc / total_count / in_trap / mcause 随 test_dev 收尾漂, 见 REVIEW B.3 段; 不在断言内。)
