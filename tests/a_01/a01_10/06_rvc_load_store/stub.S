# a_01_10 / 06_rvc_load_store
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验 RVC load/store case (a_01_10 (7) step 4):
# (注: 原收尾靠 ecall → mtvec=0 fetch fail → triple fault (in_trap=3); a_03 Smdbltrp 后
#  首个 trap 撞 MDT=1 即 abort, 改在末尾追加 test_dev 干净停机替代跑飞。RVC load/store 结果
#  在停机前已落定。原 in_trap=3 / mcause=1 收尾叙事见 REVIEW B.3 段。)
#   C.LW   (C0 funct3=010, CL-format) — load word, rd'/rs1' 都 3-bit (x8-x15)
#   C.SW   (C0 funct3=110, CS-format) — store word, rs1'/rs2' 都 3-bit
#   C.LWSP (C2 funct3=010, CI-format) — load word sp-relative, rd 5-bit
#   C.SWSP (C2 funct3=110, CSS-format) — store word sp-relative, rs2 5-bit
#
# ===== 思路 =====
# 模板风格: 纯 .S, BARE M-mode (无 SV32 / 无 PT), 套 a_01_3/02_arith_compressed; -march=rv32imac。
# BARE M-mode 下 lsu identity 路径: gva == pa (current_tlb=NULL), 直接 host load/store。
#
# 寄存器分配:
#   - sp (x2): 0x80100000 (RAM 内空闲, 跟 a_01_8 fixture sp 风格一致)
#   - x8: 数据基地址 = 0x80200000 (RAM 内, 不冲突代码 / sp / PT)
#   - x9 / x13: 写入值 (rs2' for C.SW, 3-bit)
#   - x10 / x14: 读回值 (rd' for C.LW, 3-bit)
#   - x11 / x15: 写入值 (rs2 for C.SWSP, 5-bit)
#   - x12: 读回值 (rd for C.LWSP, 5-bit)
#
# 4 case 全覆盖:
#   (a) C.SW + C.LW (rd-relative, offset 0): x9=31 → mem[x8] → x10
#   (b) C.SWSP + C.LWSP (sp-relative, offset 16): x11=7 → mem[sp+16] → x12
#   (c) C.SW + C.LW with offset != 0 (offset 4): x13=12 → mem[x8+4] → x14
#   (d) C.SWSP with offset != 0 (offset 20): x15=5 → mem[sp+20] (不验回读, sp+16 验过)

.section .text
.globl _start
_start:
    # ---- 设 sp + 数据基地址 ----
    lui       sp, 0x80100        # sp = 0x80100000 (32-bit lui, 不是 RVC)
    lui       x8, 0x80200        # x8 = 0x80200000 (数据基地址)

    # ============================================================
    # (a) C.SW + C.LW (rd-relative, offset 0)
    # ============================================================
    c.li      x9, 31             # x9 = 31 (c.li imm6 max+ = 31)
    c.sw      x9, 0(x8)          # mem[x8 + 0] = 31
    c.lw      x10, 0(x8)         # x10 = mem[x8 + 0] = 31 (验 C.LW)

    # ============================================================
    # (b) C.SWSP + C.LWSP (sp-relative, offset 16)
    # ============================================================
    c.li      x11, 7             # x11 = 7
    c.swsp    x11, 16(sp)        # mem[sp + 16] = 7
    c.lwsp    x12, 16(sp)        # x12 = mem[sp + 16] = 7 (验 C.LWSP)

    # ============================================================
    # (c) C.SW + C.LW with offset != 0 (offset 4)
    # ============================================================
    c.li      x13, 12            # x13 = 12
    c.sw      x13, 4(x8)         # mem[x8 + 4] = 12
    c.lw      x14, 4(x8)         # x14 = 12 (验 C.LW imm 解码正确)

    # ============================================================
    # (d) C.SWSP with offset != 0 (offset 20) — 不验回读, sp+16 验过 LWSP
    # ============================================================
    c.li      x15, 5             # x15 = 5
    c.swsp    x15, 20(sp)        # mem[sp + 20] = 5

    # ---- 停机: test_dev FINISHER PASS (替代原跑飞进 fetch-fault) ----
    li   x30, 0x100000         # test_dev base
    li   x31, 0x5555           # FINISHER PASS magic
    sw   x31, 0(x30)           # 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望 main.c dump:
#   sp (x2) = 0x80100000   (lui sp, 0x80100; decimal 2148532224)
#   x8  = 0x80200000       (lui x8, 0x80200; hex 0x80200000)
#   x9  = 0x0000001f       (= 31; rs2' for C.SW; hex)
#   x10 = 31               (C.LW 回读 mem[x8] = 31; decimal)
#   x11 = 7                (rs2 for C.SWSP; decimal)
#   x12 = 7                (C.LWSP 回读 mem[sp+16] = 7; decimal)
#   x13 = 12               (rs2' for C.SW offset 4; decimal)
#   x14 = 12               (C.LW 回读 mem[x8+4] = 12; decimal)
#   x15 = 5                (rs2 for C.SWSP offset 20; decimal)
#   x30 = 0x100000 / x31 = 0x5555 (停机 scratch: test_dev base / FINISHER PASS magic)
#
# 停机 = test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
# (注: 绝对 pc / total_count / in_trap / mcause 随 test_dev 收尾漂, 见 REVIEW B.3 段; 不在断言内。)
#
# 关键验证点:
#   - x10 == x9 → C.SW + C.LW (rd-relative, offset 0) 通过
#   - x12 == x11 → C.SWSP + C.LWSP (sp-relative, offset 16) 通过
#   - x14 == x13 → C.SW + C.LW with offset != 0 (offset 4) 通过
#   - sp / x8 都对 → 寄存器 5-bit / 3-bit 编码正确, sp-relative imm 解码正确
