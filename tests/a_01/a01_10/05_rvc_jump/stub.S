# a_01_10 / 05_rvc_jump
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验 RVC jump case (a_01_10 (7) step 3):
# (注: 原收尾靠 ecall → mtvec=0 fetch fail → triple fault (in_trap=3); a_03 Smdbltrp 后
#  首个 trap 撞 MDT=1 即 abort, 改在末尾追加 test_dev 干净停机替代跑飞。RVC jump/return 结果
#  在停机前已落定。原 in_trap=3 / mcause=1 收尾叙事见 REVIEW B.3 段。)
#   C.J     (funct3=101, CJ-format) — 无条件跳, rd=x0
#   C.JAL   (funct3=001, CJ-format, RV32-only) — call subroutine, ra=x1=pc+2
#   C.JR    (C2 funct3=100, bit12=0+rs2=0) — return, rd=x0, jump to rs1
#   C.JALR  (C2 funct3=100, bit12=1+rs1!=0+rs2=0) — indirect call, ra=x1=pc+2
#
# ===== 思路 =====
# 关键验证点 (修复 a_01_4 起步 hardcoded `pc + 4u` ra bug):
#   - C.JAL ra = pc + 2 (RVC 长度), 不是 pc + 4 (32-bit JAL 长度)
#   - C.JALR ra = pc + 2 同
#   - 修复哲学: ra = pc + d.pc_step (透传 decode 阶段指令长度), 跟 branch 修复同形态
#
# 验证间接靠 sub 调用 + return 行为:
#   - 若 ra 错 (= pc+4 而非 pc+2), c.jr x1 跳到非 ret 点, 程序乱跑 → fixture 跑不通
#   - 若 ra 对, sub 内 c.jr x1 正确返回, dump GPR 符合期望
#
# 4 path 全覆盖:
#   (a) C.J taken — 跳过中间 c.li, 验 x10 不被覆盖
#   (b) C.JAL + sub1 内 C.JR return — 验 x11 被 sub1 修改 + 返回正确
#   (c) C.JALR + sub2 内 C.JR return — 验 x12 被 sub2 修改 + 返回正确
#
# 模板风格: 纯 .S, BARE M-mode, 套 a_01_3/02_arith_compressed; -march=rv32imac, c.* 强制 RVC。

.section .text
.globl _start
_start:
    # ============================================================
    # (a) 验 C.J taken (跳过中间)
    # ============================================================
    c.li      x10, 1            # x10 = 1 (初始, 若 C.J taken 则保留)
    c.j       after_j
    c.li      x10, 0             # 跳过, 不执行 (taken 路径不到这)
after_j:
    # x10 = 1

    # ============================================================
    # (b) 验 C.JAL + sub1 内 C.JR return
    # ============================================================
    c.li      x11, 0             # x11 = 0 (累加器, sub1 += 1)
    c.jal     sub1               # ra (x1) = pc+2; jump sub1
    # 返回这里, x11 = 1

    # ============================================================
    # (c) 验 C.JALR + sub2 内 C.JR return
    # ============================================================
    la        x6, sub2           # x6 = sub2 PC (32-bit pseudo lui+addi)
    c.li      x12, 0             # 标记 sub2 是否被调用
    c.jalr    x6                 # ra (x1) = pc+2; jump x6 (= sub2)
    # 返回这里, x12 = 7

    # ---- 停机: test_dev FINISHER PASS (替代原跑飞进 fetch-fault) ----
    li   x30, 0x100000           # test_dev base
    li   x31, 0x5555             # FINISHER PASS magic
    sw   x31, 0(x30)             # 置 exit_code=0 + SRS=0, dispatcher 退主循环
1:  j       1b                      # 防 fall-through: SRS=0 下个 block 边界退

# ============================================================
# sub1: 累加 x11 + 返回 (验 C.JR rs1=x1)
# ============================================================
sub1:
    c.addi    x11, 1             # x11 += 1
    c.jr      x1                 # 返回 (jump to ra)

# ============================================================
# sub2: 设 x12 = 7 + 返回 (验 C.JR 第二次)
# ============================================================
sub2:
    c.li      x12, 7             # x12 = 7
    c.jr      x1                 # 返回

# ===== 期望结果 =====
# 期望 main.c dump:
#   x10 = 1       (C.J taken, 跳过 c.li x10, 0; 保留初始 1; decimal 1)
#   x11 = 1       (C.JAL → sub1 → c.addi x11, 1 → c.jr x1 return; decimal 1)
#   x12 = 7       (C.JALR → sub2 → c.li x12, 7 → c.jr x1 return; decimal 7)
#   x6 = sub2 PC  (la x6, sub2 设的; 0x80000xxx 范围, hex 显示 0x800000xx)
#   x1 = ret_jalr (最后 C.JALR 设的 ra; 0x80000xxx 范围)
#   x30 = 0x100000 / x31 = 0x5555 (停机 scratch: test_dev base / FINISHER PASS magic)
#
# 停机 = test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
# (注: 绝对 pc / total_count / in_trap / mcause 随 test_dev 收尾漂, 见 REVIEW B.3 段; 不在断言内。)
