# a_02_2 / 06_sv32_clint_reject
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验 SV32 经 mmu 翻译后到 CLINT 内部 reject 路径 (a_02_2/03 BARE 版的 SV32 版本)。
# (注: 原 m_halt 用 csrw mtvec,x0 + ecall → triple fault 停机; a_03 Smdbltrp 后改
#  test_dev 干净停机, 旧 triple-fault / in_trap=3 叙事见 REVIEW B.3 段。)
#
# ===== 思路 =====
# 跟 a_02_2/03 比, 多走一遍 walker; clint_read/write 内部判 reject 行为应一致。
#
# 5 个 case (mret +4 chain, 跟 a_02_2/03 同):
#   case 0: sw → 0x02000004 → clint_write msip idx 越界 (idx=1 ≥ MAX_HARTS=1) → cause 7
#   case 1: sw → 0x02004008 → clint_write mtimecmp idx 越界 (idx=1)             → cause 7
#   case 2: sw → 0x0200C000 → clint_write final else (off >= MTIME_OFF+4)       → cause 7
#   case 3: sb → 0x02000000 → clint_write size != 4                             → cause 7
#   case 4: lw → 0x02000004 → clint_read  msip idx 越界 (verify cause 5 path)   → cause 5

.equ CSR_TOHOST, 0x800
.equ CSR_PRIVRD, 0xCC0

.section .text
.option norvc
.globl _start
_start:
    lui  sp, 0x80300

    la   x4, m_handler
    csrw mtvec, x4

    call setup_pt

    lui  x6, 0x80080
    addi x6, x6, 0x100
    csrw satp, x6
    sfence.vma

    la   x7, s_target
    csrw mepc, x7

    addi x4, x0, 1
    slli x4, x4, 11
    csrw mstatus, x4

    li   s0, 0
    li   s1, 0

    mret                            # M→S

s_target:
    # === case 0: msip idx 越界 ===
    lui  x4, 0x02000
    addi x4, x4, 4                  # vaddr 0x02000004 (msip[1], MAX_HARTS=1 → 越界)
    li   x5, 0
    sw   x5, 0(x4)

    # === case 1: mtimecmp idx 越界 ===
    lui  x4, 0x02004
    addi x4, x4, 8                  # vaddr 0x02004008
    sw   x5, 0(x4)

    # === case 2: final else ===
    lui  x4, 0x0200C                # vaddr 0x0200C000
    sw   x5, 0(x4)

    # === case 3: size != 4 (sb) ===
    lui  x4, 0x02000                # vaddr 0x02000000
    sb   x5, 0(x4)

    # === case 4: msip idx 越界 (read) ===
    lui  x4, 0x02000
    addi x4, x4, 4
    lw   x6, 0(x4)

    # case 4 后 handler 内 detect s0==5 → 不 mret, 在 handler 内 (M-mode) 走 test_dev
    # 干净停机 (m_halt), 不回 S phase。防御: 真跑到 = handler 没正确
    # detect halt, 走 ebreak 留 debug 信号。
    ebreak

# ----------------------------------------------------------------------------
# m_handler — switch on s0, mret +4 chain (跟 a_02_2/03 同)
# ----------------------------------------------------------------------------
m_handler:
    csrr x7,  mcause
    csrr x28, mtval
    add  s1, s1, x7                 # cause sanity sum (4*7 + 5 = 33)

    li   x29, 0
    beq  s0, x29, case0
    li   x29, 1
    beq  s0, x29, case1
    li   x29, 2
    beq  s0, x29, case2
    li   x29, 3
    beq  s0, x29, case3
    # default case 4 (read, cause + tval 都存)
case4:
    mv   a4, x7                     # a4 = case 4 cause = 5
    mv   a5, x28                    # a5 = case 4 tval
    j    epilogue
case3: mv a3, x28
    j    epilogue
case2: mv a2, x28
    j    epilogue
case1: mv a1, x28
    j    epilogue
case0: mv a0, x28
    j    epilogue

epilogue:
    addi s0, s0, 1
    li   x31, 5                     # 5 case 完成阈值
    beq  s0, x31, m_halt            # case chain 完 → M-mode 内 halt
    csrr x30, mepc
    addi x30, x30, 4
    csrw mepc, x30
    mret

# ----------------------------------------------------------------------------
# m_halt — case chain 完成后从 handler 直接走 halt 路径 (still in M-mode); 跟
# a_02_2/05 同形态。test_dev FINISHER PASS 干净停机。
# (停机 scratch 借用 x30/x31 — 此处 epilogue 已不再用它们, 复用安全。)
# ----------------------------------------------------------------------------
m_halt:
    li   x30, 0x100000              # test_dev base
    li   x31, 0x5555                # FINISHER PASS magic
    sw   x31, 0(x30)                # 置 exit_code=0 + SRS=0, dispatcher 退主循环
1:  j    1b                         # 防 fall-through 落进后续代码 (dispatcher 下个 block 末退)

# ===== 期望结果 =====
# 期望 main.c dump (跟 a_02_2/03 一致):
#   a0 (x10) = 0x02000004
#   a1 (x11) = 0x02004008
#   a2 (x12) = 0x0200C000
#   a3 (x13) = 0x02000000
#   a4 (x14) = 5                  case 4 mcause
#   a5 (x15) = 0x02000004         case 4 mtval
#   s0 (x8)  = 5
#   s1 (x9)  = 33                 (4*7 + 5)
#   x30 = 0x100000 / x31 = 0x5555  (停机 scratch: test_dev base / FINISHER PASS magic)
#   停机 = test_dev FINISHER PASS → exit code 0
# (注: 绝对 pc / total_count 见 REVIEW B.3 段; 不在断言内。)
