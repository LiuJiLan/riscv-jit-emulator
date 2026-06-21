# b_03_1 / 04_sv32_smc
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验 b_03 T1.a SMC chain 在 **SV32_S regime** 下端到端工作 (6 axes (i)=SV32):
#   - SV32 vaddr → PA 通过 walker translate; SMC trigger 是 SV32 sw → store_helper
#     内 memcpy host RAM → host kernel 看到 mprotect R-only 触发 SIGSEGV → handler
#     标 page_dirty bitmap + smc_unprotect_page
#   - dispatcher 顶扫 page_dirty (PA 级, 不是 VA) → jit_invalidate_page → invalidate
#     SV32_S regime 的 JIT 块
#   - SV32 vaddr == PA (identity PT setup; leaf[0] PPN=0x80000 直接映射 PA 0x80000000)
#
# 跟 fixture 01 (BARE) 区别: regime = SV32_S, JIT cache key (PA, SV32_S) 不同;
# 验 SV32_S regime 下 SMC 路径都接通.
#
# 期望 x6 = 150 (phase 1 +1 × 150) + 100 (phase 3 +10 × 10) = 1150 (同 fixture 01).
# 注: phase 3 只 100 iter 不达 SV32 重 compile THRESHOLD (跟 fixture 01 一致),
# 全 interpret 跑改后代码.
#
# ===== 思路 =====
# M setup → satp 设 SV32 (跟 b02_06 同) → mret 进 S phase → hot_loop SV32_S
# regime 跑 + SMC trigger → ecall 回 M handler halt
#
# PT setup: main.c setup_pt() (跟 b02_06 同 + leaf[0] 加 PTE_W 让 code 可写)

.section .text
.option norvc
.globl _start
_start:
    # ---- M phase setup ----
    lui  sp, 0x80300                # 栈在 RAM 空闲区
    la   x4, m_handler
    csrw mtvec, x4

    call setup_pt                   # main.c 建 PT (BARE M-mode identity)

    # ---- 设 satp: MODE=Sv32 + PPN=0x80100 ----
    lui  x6, 0x80080
    addi x6, x6, 0x100              # satp = 0x80080100
    csrw satp, x6
    sfence.vma

    # ---- 设 mret 目标 = s_target identity vaddr ----
    la   x7, s_target
    csrw mepc, x7

    # ---- mstatus.MPP = PRIV_S ----
    addi x4, x0, 1
    slli x4, x4, 11                 # MPP=01=S
    csrw mstatus, x4

    mret                            # M→S

s_target:
    # ---- S phase setup (SV32_S regime) ----
    li   x1, 150                    # phase 1 iter count
    li   x6, 0                      # accumulator
    li   x14, 0                     # phase counter (用 x14, 避免跟 m_handler x12 撞)

hot_loop:
    # *** 第一条 inst (phase 2 改 imm) ***
    addi x6, x6, 1                  # 0x00130313 → 0x00a30313 (改 imm 1→10)
    addi x1, x1, -1
    bne  x1, x0, hot_loop

phase_dispatch:
    addi x14, x14, 1
    li   x15, 1
    beq  x14, x15, phase2_modify
    j    halt

phase2_modify:
    # ---- SV32 sw 改 hot_loop 第一条 inst ----
    # x7 = hot_loop vaddr (SV32 region; identity mapped to PA 0x80000xxx)
    # sw 走 SV32 store_helper → walker translate → host RAM memcpy → mprotect 触发 SIGSEGV
    la   x7, hot_loop
    li   x8, 0x00a30313             # addi x6, x6, 10
    sw   x8, 0(x7)                  # *** SMC trigger (SV32 store) ***

    li   x1, 100
    j    hot_loop                   # phase 3

halt:
    ecall                           # cause=9 from S; trap deliver M

m_handler:
    csrr x12, mcause                # 期望 cause=9 (ecall from S)
    addi x13, x0, 9
    bne  x12, x13, halt_safe        # 防御 (期望 fall through)

halt_safe:
    li   x30, 0x100000              # test_dev base (PA, M-mode BARE 直通)
    li   x31, 0x5555                # FINISHER PASS magic
    sw   x31, 0(x30)
m_halt_spin:
    j    m_halt_spin

# ===== 期望结果 =====
# phase 1 (SV32_S regime, hot_loop): x6 += 1 × 150 = 150
# phase 2: SV32 sw 改 hot_loop 第一条 inst (SMC trigger)
# phase 3 (SV32_S regime, hot_loop 改后): x6 += 10 × 100 = 1000 (全 interpret)
# 总 x6 = 1150
#
# main.c dump:
#   x1     = 0
#   x6     = 1150        ← 关键: SV32_S regime SMC 真生效
#   x12    = 9           (mcause = ecall from S)
#   x14    = 2           (phase counter; phase 1 末 + phase 3 末 都进 phase_dispatch 各 +1)
#   x15    = 1
#   x30/x31 = test_dev FINISHER PASS magic
#   priv   = 3            (M; ecall 后切回 M)
#   exit 0
#
# SV32 verify 点 (跟 fixture 01 BARE 比):
#   - regime = SV32_S 时 JIT cache key (PA, SV32_S) 跟 BARE 区分
#   - SMC SIGSEGV handler 拿到的 fault_addr 是 host_ram_base + PA offset
#     (即使是 SV32 walker translate 后的 store, host 视角仍是 host RAM addr)
#   - mprotect 单位是 PA page (跟 BARE 同), 不关 SV32 walker
#   - 整链跟 BARE 路径完全相同, 验 axes (i)=SV32_S 真接通
