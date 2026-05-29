# a_01_8 / 09_sv32_root_pt_not_in_ram
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验 SV32 walker level=1 PT (root_pt 自身) 不在 RAM 时报 access fault — 走 fetch 路径
# 触发 INST_ACCESS_FAULT (cause 1)。跟 fixture 08 (level=0 leaf_pt 不在 RAM, load 路径
# cause 5) 互补; 两个 fixture 加起来覆盖 mmu_walk RAM 检查两段 (mmu.c 内 level=1 + level=0)。
#
# ===== 思路 =====
# 跟 04-08 模板区别: 不分配 PT (root_pt 故意 satp.PPN 指 RAM 外); main.c setup_pt() 保留
# 但函数体空, 跟模板形态一致。
#
# 触发路径 (mmu_translate_pc SV32 路径, 跟 walker_helper_load 失败路径不同):
#   mret 切 S 后 dispatcher 调 mmu_translate_pc(hart, current_tlb=tlb_table[1][0]);
#   mmu_translate_pc 内 SV32 段 → mmu_walk(MMU_PERM_X) →
#     pte1_pa = satp.PPN<<12 + vpn1*4 = 0xC0000000 + 0x800 = 0xC0000800;
#     RAM 检查 (uint32_t)(0xC0000800 - 0x80000000) = 0x40000800 >= 0x08000000 → fail;
#     fault_cause = af_cause_for(MMU_PERM_X) = CAUSE_INST_ACCESS_FAULT = 1;
#   mmu_translate_pc 用 trap_set_state(hart, 1, gva) — 路径 2b, 不长跳, return 给 dispatcher,
#   dispatcher 进 trap deliver M, 重 fetch from mtvec = m_handler。
#
# 跟 fixture 08 区别:
#   1. 触发指令: lw → fetch (mret 后第一条 fetch 立即 fail)
#   2. cause:    5 (LOAD_ACCESS_FAULT) → 1 (INST_ACCESS_FAULT)
#   3. trap 路径: trap_raise_exception 长跳 (路径 2a) → trap_set_state (路径 2b) +
#                 dispatcher continue 兜底
#
# satp 设计:
#   MODE = Sv32 (bit 31 = 1)
#   ASID = 0
#   PPN  = 0xC0000 (root_pt PA = 0xC0000000, 不在 RAM 0x80000000~0x88000000)
#   satp = (1<<31) | 0xC0000 = 0x800C0000
#
# 控制流:
#   1. M setup: sp + mtvec; call setup_pt() (空); csrw satp = 0x800C0000 + sfence;
#      mepc = s_target identity vaddr + mstatus.MPP=PRIV_S; mret 切 S
#   2. mret 后: dispatcher 调 mmu_translate_pc(s_target vaddr) → walker level=1 PT 读
#      RAM 检查 fail → trap 1 (路径 2b); s_target 内任何指令都不会执行
#   3. M handler: csrr mcause/mepc/mtval → x12/x13/x14; test_dev FINISHER PASS 干净停机

.equ CSR_TOHOST, 0x800
.equ CSR_PRIVRD, 0xCC0

.section .text
.option norvc
.globl _start
_start:
    # ---- 设 sp ----
    lui  sp, 0x80300

    # ---- 设 M handler ----
    la   x4, m_handler
    csrw mtvec, x4

    # ---- C 函数建 PT (空, 跟模板一致) ----
    call setup_pt

    # ---- 设 satp: MODE=Sv32 + PPN=0xC0000 (root_pt PA 0xC0000000 不在 RAM) ----
    lui  x6, 0x800C0                # x6 = 0x800C0000
    csrw satp, x6
    sfence.vma

    # ---- 设 mret 目标 = s_target identity vaddr (但 fetch 永不到达) ----
    la   x7, s_target
    csrw mepc, x7

    # ---- 设 mstatus.MPP = PRIV_S ----
    addi x4, x0, 1
    slli x4, x4, 11
    csrw mstatus, x4

    mret                            # M→S; fetch s_target → walker fail → trap 1

s_target:
    # ---- 永未执行段 (mret 后 fetch 这条之前就 trap) ----
    ecall                           # 防御

m_handler:
    csrr x12, mcause                # x12 = 1 (INST_ACCESS_FAULT)
    csrr x13, mepc                  # x13 = s_target PA
    csrr x14, mtval                 # x14 = s_target PA (= gva, mmu_translate_pc 第 2 参)

halt:
    li   x30, 0x100000              # test_dev base
    li   x31, 0x5555                # FINISHER PASS magic
    sw   x31, 0(x30)                # 干净停机: test_dev 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望 main.c dump:
#   x10 = 0                  (s_target 永未执行)
#   x12 = 1                  (mcause = INST_ACCESS_FAULT)
#   x13 = s_target PA        (mepc = mret 后 cpu->pc; 绝对值随 codegen 漂, 不钉死; x13 == x14)
#   x14 = s_target PA        (mtval = mmu_translate_pc trap_set_state 第 2 参 = gva; 绝对值随 codegen 漂)
#   x30 = 0x100000           (停机用 scratch: test_dev base)
#   x31 = 0x5555             (停机用 scratch: FINISHER PASS magic)
#   priv = 3
#   停机 = test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
# (注: 绝对 pc / total_count 随 test_dev 收尾漂, 见 REVIEW B.3 段; 不在断言内。)
#
# 关键验证点:
#   - x12 = 1 → mmu_translate_pc 真在 PT-not-in-RAM 路径报 INST_ACCESS_FAULT (跟 lw cause 5 区分)
#   - x14 == x13 → trap_set_state 路径 mtval 协议跟 trap_raise_exception 一致 (都写 gva = 失败 fetch vaddr)
#   - x13 = mepc = 失败 fetch 的 vaddr (没真执行任何指令, mret 后 cpu->pc)
#   - 跟 fixture 08 形成 walker RAM 检查两段 (level=1 + level=0) 完整覆盖
