# a_01_10 / 04_rvc_branch
#
# ===== 测试目的 =====
# 验 RVC branch case (a_01_10 (7) step 2):
#   C.BEQZ (funct3=110, CB-format) — 跳 / 不跳两路径
#   C.BNEZ (funct3=111, CB-format) — 跳 / 不跳两路径
# 跟 a_01_4/01_branch_loop 同形态用计数器 GPR 验跳数; 4 path 全覆盖 (taken/not-taken × BEQZ/BNEZ)。
#
# ===== 思路 =====
# 模板风格: 纯 .S, BARE M-mode (无 SV32 / 无 PT / 无 m_handler), 套 a_01_3/02_arith_compressed。
# 寄存器分配: rs1' 用 x8-x15 (3-bit RVC 范围); 各路径独立 GPR 验 final 值。
#
# 验证哲学:
#   - C.BEQZ/C.BNEZ 翻译为 OP_BEQ/OP_BNE rs1=rs1' rs2=x0 (decode_rvc 复用 RV32I op_kind)
#   - pc_step = PC_STEP_NONE (control flow case 自写 pc, 跟 32-bit BEQ/BNE 同)
#   - BRANCH_IF 宏 not-taken 路径根据 raw_inst[1:0] 区分 RVC pc+2 / 32-bit pc+4
#   - 4 path 全覆盖 + 两条路径都正确:
#     (a) BNEZ taken (loop 5 次, x8 倒数, x9 累加)
#     (b) BEQZ taken (跳过中间 c.li, 验 x11 = 初始值)
#     (c) BEQZ not-taken (fall-through 执行 c.addi, 验 x13 = 初始 + 1)
#     (d) BNEZ not-taken (fall-through 执行 c.addi, 验 x15 = 初始 + 1)

.section .text
.globl _start
_start:
    # ============================================================
    # (a) 验 C.BNEZ taken (loop 5 次)
    # ============================================================
    c.li      x8, 5             # x8 = 5 (loop counter)
    c.li      x9, 0             # x9 = 0 (accumulator)
loop:
    c.addi    x9, 1             # x9 += 1
    c.addi    x8, -1            # x8 -= 1
    c.bnez    x8, loop          # if x8 != 0 goto loop (loop 5 次后 x8=0 退出)
    # 此时 x8 = 0, x9 = 5

    # ============================================================
    # (b) 验 C.BEQZ taken (跳过中间)
    # ============================================================
    c.li      x10, 0            # x10 = 0
    c.li      x11, 16           # x11 = 16 (初始, BEQZ taken 则不变)
    c.beqz    x10, skip_beqz_jump
    c.li      x11, 0            # 跳过则不执行; 不跳的话才执行
skip_beqz_jump:
    # 此时 x11 = 16 (BEQZ taken 跳过 c.li x11, 0)

    # ============================================================
    # (c) 验 C.BEQZ not-taken (fall-through)
    # ============================================================
    c.li      x12, 1            # x12 != 0 (BEQZ 不跳)
    c.li      x13, 8            # x13 = 8 (初始)
    c.beqz    x12, skip_beqz_nojump
    c.addi    x13, 1            # BEQZ not-taken, fall-through 执行; x13 = 8 + 1 = 9
skip_beqz_nojump:
    # 此时 x13 = 9

    # ============================================================
    # (d) 验 C.BNEZ not-taken (fall-through)
    # ============================================================
    c.li      x14, 0            # x14 == 0 (BNEZ 不跳)
    c.li      x15, 16           # x15 = 16 (初始)
    c.bnez    x14, skip_bnez_nojump
    c.addi    x15, 1            # BNEZ not-taken, fall-through 执行; x15 = 16 + 1 = 17
skip_bnez_nojump:
    # 此时 x15 = 17

    # ---- 退出 (BARE M-mode, mtvec=0, ecall chain in_trap=3 halt) ----
    ecall

# ===== 期望结果 =====
# 期望 main.c dump:
#   x8 = 0       (BNEZ taken loop 倒数到 0; hex 0x00000000)
#   x9 = 5       (BNEZ taken loop accumulator; hex 0x00000005)
#   x10 = 0      (BEQZ taken 操作数, 不变)
#   x11 = 16     (BEQZ taken 跳过 c.li x11, 0; 保留初始 16; decimal 16)
#   x12 = 1      (BEQZ not-taken 操作数, 1!=0, BEQZ 不跳; decimal 1)
#   x13 = 9      (BEQZ not-taken fall-through 执行 c.addi 1; 8+1=9; decimal 9)
#   x14 = 0      (BNEZ not-taken 操作数, 0!=0 不成立, 不跳; decimal 0)
#   x15 = 17     (BNEZ not-taken fall-through 执行 c.addi 1; 16+1=17; decimal 17)
#
# 末态 (跟 fixture 03 同 BARE M-mode mtvec=0 chain):
#   in_trap=3 / mcause=1 (二次 fetch 0 fail) / mepc=0 / mtvec=0 / priv=3
#
# 关键验证点:
#   - x8/x9: BNEZ taken 路径完整 (loop 5 次 → counter 倒数 + accumulator 累加)
#   - x11: BEQZ taken 跳过中间, c.li x11, 0 没执行 → 保留初始 16
#   - x13: BEQZ not-taken fall-through, c.addi 执行 → 9
#   - x15: BNEZ not-taken fall-through, c.addi 执行 → 17
#   - 4 path 全 PASS → C.BEQZ/C.BNEZ decode + interpreter BRANCH_IF (RVC pc+2 自动适配) 全对
