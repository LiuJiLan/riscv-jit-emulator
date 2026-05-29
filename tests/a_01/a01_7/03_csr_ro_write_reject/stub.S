# a_01_7 / 03_csr_ro_write_reject
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验证 csr_op 入口 RO 写检查激活: M 模式写 RO csr (mhartid 0xF14, bits 11:10 = 11 = RO)
# → trap cause=2 (illegal instruction); 即使 priv=M 够 (mhartid 是 unprivileged csr, bits 9:8=00),
# RO 写仍然 block。
# handler 抓 mcause/mtval/mepc, 验毕 test_dev FINISHER PASS 干净停机。
# (注: 原停机靠 csrw mtvec,x0 + ecall 强制 triple fault — a_03 Smdbltrp 后 triple fault 变
#  MDT abort, 且 cpu_reset MDT=1 会让首个 trap 就 abort、handler 跑不到; 故 _start 先清 MDT,
#  收尾改 test_dev 干净停机。原 triple-fault/in_trap=3 收尾叙事见 REVIEW B.3 段。)
#
# ===== 思路 =====
# 控制流:
#   _start: 清 MDT → la+csrw mtvec=handler → csrw mhartid (RO 写) → trap cause=2 → handler
#   handler (M): csrr x1=mcause(2), x2=mtval(raw_inst), x3=mepc → test_dev FINISHER PASS → 退主循环

.section .text
.option norvc
.globl _start
_start:
    csrw 0x310, x0          # 清 mstatush.MDT (否则首个 trap 撞 MDT=1 直接 abort, handler 跑不到)
    la   x4,  handler       # la pseudo (8 字节)
    csrw mtvec, x4          # mtvec = handler

    # ---- M 模式写 RO csr mhartid (bits 11:10 = 11 = RO) → trap cause=2 ----
    # x10 当前 = 0 (memset), 但 csrrw 永远算"写" (CSR_OP_RW 不看 new_val); 即使 x10=0 也 trap。
    csrw mhartid, x10       # RO 检查 fail → trap_raise(2, raw_inst); 不到下一条

handler:
    csrr x1,  mcause        # x1 = mcause = 2 (illegal instruction)
    csrr x2,  mtval         # x2 = mtval = 0xF1451073 (raw_inst of csrw mhartid)
    csrr x3,  mepc          # x3 = mepc = csrw mhartid 的 PC

    # ---- 停机: test_dev FINISHER PASS ----
    li   x30, 0x100000      # test_dev base
    li   x31, 0x5555        # FINISHER PASS magic
    sw   x31, 0(x30)        # 干净停机: test_dev 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望:
#   x1 = 2          (mcause = illegal instruction)
#   x2 = 0xF1451073 = 4047835251
#                    (mtval = raw_inst of `csrw mhartid, x10`; 编码:
#                     csr=0xF14 << 20 = 0xF1400000
#                     rs1=10 << 15 = 0x00050000
#                     funct3=001 << 12 = 0x00001000
#                     rd=0 << 7  = 0
#                     opcode=0x73
#                     总 = 0xF1451073)
#   x3 = csrw mhartid 的 PC (绝对值随 prelude 漂, 见 REVIEW)
#   x30 = 0x100000 / x31 = 0x5555 (停机用 scratch: test_dev base / FINISHER PASS magic)
#   priv = 3        (handler 在 M 模式跑)
#   停机 = test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
# (注: 绝对 pc / total_count 见 REVIEW B.3 段; 不在断言内。)
