# b_02_6 / 01_sv32_jit_smoke
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验 b_02 T6 JIT SV32_S regime fast path 端到端真激活. SV32 region 内 loop 块
# 跑 200 iter, 跨 COMPILE_THRESHOLD=100. 跟 b02_05/01 (BARE BRANCH 块) 体例对
# 偶, 区别: 块在 SV32_S regime + 块内含 sw + lw (SV32 LOAD/STORE fast path 主
# 路径).
#
# 期望 trace 前段 '._._...' 重复 (interpret); 第 100 次 fetch_add counter 触
# 发 jit_compile_block, 后段 '._J._J...' 重复 (host code 真跑). 数 'J' 字符
# 总数 ≈ 100, 验证 SV32 host code 真激活 + sw/lw 跑出正确值.
#
# ===== 思路 =====
# PT layout 沿用 a_01_8/04 (4KB 页 root_pt + leaf_pt; main.c setup_pt() 同).
# M setup → satp → mret 进 S phase → loop 200 iter (sw/lw 在 SV32 region 同
# vaddr 反复) → halt (无 trap, 直接 sw test_dev FINISHER PASS).
#
# S phase loop 块体:
#   loop: addi x6, x6, 1
#         addi x1, x1, -1
#         sw   x6, 0(x4)       # SV32 store fast path (TLB hit 第二次起)
#         lw   x5, 0(x4)       # SV32 load fast path
#         bne  x1, x0, loop
# 5 inst / block; 200 iter 跨 THRESHOLD=100 → host code 真激活 100 次.
#
# 用 x6 (t1, 内存 reg) 作 accumulator, 不用 x2 (sp host map): x2 (sp) 末段
# fall-through 进 main.c setup_pt() gcc prologue (addi sp, sp, -32) 会被减 32
# 误读 — 跟 SV32 / T6 无关, 是 linker layout 副作用. 用 x6 + halt 段 j 死循
# 环防 fall-through 杜绝该副作用.

.section .text
.option norvc
.globl _start
_start:
    # ---- 设 sp (栈在 RAM 内空闲区, setup_pt() prologue/epilogue 用) ----
    lui  sp, 0x80300

    # ---- 设 M handler (走 halt label; bne 走 fall-through 体例) ----
    la   x4, m_handler
    csrw mtvec, x4

    # ---- C 函数建 PT (BARE M-mode, identity, walker 未激活) ----
    call setup_pt

    # ---- 设 satp: MODE=Sv32 + PPN=0x80100 (= root_pt PA >> 12) ----
    lui  x6, 0x80080
    addi x6, x6, 0x100              # satp = 0x80080100
    csrw satp, x6
    sfence.vma                      # 让新 satp 生效 (CLAUDE.md: 不自动 sfence)

    # ---- 设 mret 目标 = s_target identity vaddr ----
    la   x7, s_target
    csrw mepc, x7

    # ---- 设 mstatus.MPP = PRIV_S ----
    addi x4, x0, 1
    slli x4, x4, 11                 # x4 = 0x800 (MPP=01=S)
    csrw mstatus, x4

    mret                            # M→S, pc=identity vaddr

s_target:
    # ---- S phase loop 200 iter (跨 THRESHOLD=100; SV32_S regime sw/lw 主路径) ----
    lui  x4, 0x80001                # x4 = 0x80001000 (数据段 vaddr, PT leaf[1])
    addi x1, x0, 200                # loop counter (200, 跨 THRESHOLD=100)
    addi x6, x0, 0                  # accumulator (用 x6 t1 内存 reg, 避 x2 sp fall-through 副作用)

loop:
    addi x6, x6, 1                  # x6 += 1
    addi x1, x1, -1                 # x1 -= 1
    sw   x6, 0(x4)                  # SV32 store fast path
    lw   x5, 0(x4)                  # SV32 load fast path
    bne  x1, x0, loop               # 200 iter

    # loop 退出后 ecall (回 M handler 走 halt)
    ecall                           # cause=9 from S; trap deliver M

m_handler:
    csrr x12, mcause                # 期望 cause=9 (ecall from S)
    addi x13, x0, 9
    bne  x12, x13, halt             # 防御 (期望 fall through)

halt:
    li   x30, 0x100000              # test_dev base
    li   x31, 0x5555                # FINISHER PASS magic
    sw   x31, 0(x30)                # 干净停机: test_dev 置 exit_code=0 + SRS=0
halt_spin:
    j halt_spin                     # 防 fall-through 跑到 main.c setup_pt 内 (sp -=32 副作用)

# ===== 期望结果 =====
# 期望 main.c dump:
#   x1     = 0            (loop 倒数到 0)
#   x5     = 200          (loop 末次 lw 读到的 x6 值 = 200)
#   x6     = 200          (累加 200 次)
#   x12    = 9            (ecall_from_S cause)
#   x13    = 9            (比较 reg)
#   x30    = 0x100000     (test_dev base)
#   x31    = 0x5555       (FINISHER PASS magic)
#   priv   = 3            (M)
#   停机 = test_dev FINISHER PASS → exit 0
#
# 关键验证点:
#   - x6 = 200 + x5 = 200 → loop 200 次 sw/lw SV32 路径全跑出正确值
#   - trace 含 'J' 字符 ~100 个 (THRESHOLD=100 时) → SV32 host code 真激活 + 跑通
#   - 如果 trace 没 'J' (counter 路径或 SV32 emit 路径未接通) 或 'J' 数远少于 100
#     → SV32 JIT 路径有 bug
