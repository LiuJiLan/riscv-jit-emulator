# a03_14 / 02_mmu_pte_a_atomic_smoke
#
# ===== 运行参数 =====
# RUN-SMP: 2
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 冒烟验 mmu_walk PTE.A atomic OR set + 跨 hart 可见性 (session 024 末落地 mmu PTE
# A/D atomic_fetch_or relaxed; 替换原 plain memcpy):
#   - hart 0 顺序 lw target_VA → walker_helper_load → atomic_fetch_or set PTE.A
#   - IPI hart 1 同步 (MSI trap entry 建立跨 hart happens-before)
#   - hart 1 handler 直接读 leaf PTE @ leaf_pt_pa → 验 (PTE & PTE_A) != 0
#   - 通过 = walker 改造没 break + atomic semantic 跨 hart 可见
#
# 不验 race window (race 是低概率窗口, race fixture flaky; 真 race 测试留 a_04
# mini-OS 真世界场景 + TSan 抓)。本 fixture 只是 smoke test 验改造没引入 regression。
#
# ===== 思路 =====
# M 起步, csrr mhartid 分流:
#   hart 0: 装 mtvec=m_handler_hart0 (for ecall trap from S) → setup_pt (PT 起步
#     data 页 V+R+W, **A=0** 让 walker 触发 set) → 装 satp → mret 切 S → s_main:
#     lw target_VA → walker atomic OR set PTE.A → ecall trap 回 M → IPI hart 1 →
#     poll done flag (== 1 PASS, == 2 FAIL) → 写 test_dev PASS
#   hart 1: 装 mtvec=msi_handler + 开 mie/mstatus → wfi → IPI trap → handler 清
#     msip[1] → 直接读 leaf PT data PTE (BARE 路径, hart 1 satp=0) → 验 PTE_A bit
#     → set done flag → 写 test_dev PASS/FAIL
#
# PT 设计 (跟 a02_7/08 同形态简化):
#   ROOT_PT @ 0x80100000, LEAF_PT_CODE @ 0x80101000
#   code 页 (V+R+X+A; A=1 起步, fetch walker 不卡): PA 0x80000000
#   data 页 (V+R+W; **A=0** 起步, 让 hart 0 lw 时 walker 真 set): PA 0x80010000
#   leaf_code[0]  = code PTE  @ LEAF_PT_CODE_PA + 0
#   leaf_code[16] = data PTE  @ LEAF_PT_CODE_PA + 64 (= LEAF_PTE_DATA)

.equ TEST_DEV_BASE,     0x00100000
.equ FINISHER_PASS,     0x5555
.equ FINISHER_FAIL,     0x3333

.equ CLINT_BASE,        0x02000000
.equ MSIP_HART1,        0x02000004      # CLINT_BASE + 4 (msip[1])

.equ DONE_FLAG_ADDR,    0x80200000      # 远离 PT (0x80100000-0x80102000) 和 code/data

.equ LEAF_PT_CODE_PA,   0x80101000
.equ LEAF_PTE_DATA,     (LEAF_PT_CODE_PA + 16 * 4)  # leaf PTE [16] = data 页

.equ PTE_A,             0x40            # bit 6 = A

.equ MIE_MSIE,          0x8             # MIE bit 3 = MSIE
.equ MSTATUS_MIE,       0x8             # MSTATUS bit 3 = MIE
.equ MSTATUS_MPP_S,     (1 << 11)       # MPP = S (bit 11)

.section .text
.option norvc
.globl _start

_start:
    # ---- Boot prelude (两 hart 都跑): 清 MDT/SDT 让 MIE/SIE 写得进 ----
    csrw    0x310, x0                   # mstatush 清 MDT
    csrw    sstatus, x0                 # 清 SDT (顺便清 SIE/SPIE)

    csrr    t0, mhartid
    beqz    t0, hart0_path
    li      t1, 1
    beq     t0, t1, hart1_path
    j       fail

# ---- hart 0 (init PT + S-mode access + IPI hart 1) ----
hart0_path:
    lui     sp, 0x80300                 # M-mode 栈 (setup_pt 用)

    # 装 mtvec = m_handler_hart0 (for ecall trap from S after lw target_VA)
    la      t0, m_handler_hart0
    csrw    mtvec, t0

    # call setup_pt (main.c 定义; PT 起步 data PTE V+R+W, A=0 让 walker 触发 set A)
    call    setup_pt

    # 装 satp = 0x80080100 (跟 a02_7/08 同, SV32 + PPN=0x80100 → root @ 0x80100000)
    lui     x6, 0x80080
    addi    x6, x6, 0x100
    csrw    satp, x6
    sfence.vma

    # 切 S-mode (mepc=s_main, mstatus.MPP=S, mret)
    la      x7, s_main
    csrw    mepc, x7
    li      x4, MSTATUS_MPP_S
    csrw    mstatus, x4
    mret

s_main:
    # S-mode 跑. lw data page (target_VA = 0x80010000, identity-mapped):
    # 第一次 access → TLB miss → walker_helper_load → walker walks PT → leaf PTE A=0
    # → walker 返 pte_pa = LEAF_PTE_DATA → caller atomic_fetch_or PTE_A relaxed
    lui     s2, 0x80010
    lw      t0, 0(s2)                   # walker_helper_load → atomic OR set PTE.A

    # 切回 M (ecall, cause 9 from S)
    ecall

# ---- hart 0 m_handler (after ecall from S): IPI hart 1 + poll done flag ----
m_handler_hart0:
    # 写 msip[1] = 1 (IPI hart 1)
    li      t1, MSIP_HART1
    li      t2, 1
    sw      t2, 0(t1)

    # poll DONE_FLAG_ADDR 直到 hart 1 写 1 (PASS) or 2 (FAIL)
    li      t3, DONE_FLAG_ADDR
poll_done_hart0:
    lw      t4, 0(t3)
    beqz    t4, poll_done_hart0

    # done_flag == 1 = PASS, 其它 = FAIL
    li      t5, 1
    bne     t4, t5, fail

    # PASS
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_PASS
    sw      t1, 0(t0)
1:  j       1b

# ---- hart 1 (receiver) ----
hart1_path:
    # 装 msi_handler 到 mtvec (direct mode: bits 1:0 = 0)
    la      t0, msi_handler
    csrw    mtvec, t0

    # 开 MIE.MSIE (csrw 覆盖 mie, 本 fixture 不开其它源 OK)
    li      t1, MIE_MSIE
    csrw    mie, t1

    # 开 MSTATUS.MIE (csrs 不覆盖其它 bit)
    li      t1, MSTATUS_MIE
    csrs    mstatus, t1

    # wfi 等 IPI; spurious wake 兜底重新 wfi
1:  wfi
    j       1b

# ---- hart 1 trap handler — 收 IPI 后读 leaf PTE 验 A bit + 写 done flag ----
msi_handler:
    # 清 msip[1] = 0 防 MSIP 持续 set + handler 退出后立刻重入
    li      t1, MSIP_HART1
    sw      x0, 0(t1)

    # 直接读 leaf PTE @ LEAF_PTE_DATA (hart 1 satp=0, BARE 路径直接 PA 访问)
    li      t2, LEAF_PTE_DATA
    lw      t3, 0(t2)

    # 检查 PTE_A bit
    li      t4, PTE_A
    and     t5, t3, t4

    # done_flag = 1 (PASS, A set) or 2 (FAIL, A not set)
    li      t6, DONE_FLAG_ADDR
    beqz    t5, hart1_a_not_set

    # PASS path: done_flag = 1
    li      t0, 1
    sw      t0, 0(t6)
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_PASS
    sw      t1, 0(t0)
1:  j       1b

hart1_a_not_set:
    # FAIL path: done_flag = 2 (hart 0 看到 != 1 → 也 FAIL)
    li      t0, 2
    sw      t0, 0(t6)
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_FAIL
    sw      t1, 0(t0)
2:  j       2b

# ---- 共用 fail (mhartid >= 2, 本 fixture 只 --smp 2) ----
fail:
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_FAIL
    sw      t1, 0(t0)
3:  j       3b

# ===== 期望结果 =====
#   两 hart 都 reach FINISHER_PASS sw (test_dev sifive PASS 幂等; race 无害)
#   exit_code = 0 (FINISHER_PASS)
#   main wall time 量级 < 100ms
#   无 fault / 无 ASan / 无 TSan race / 无 timeout
# 不期望:
#   - hart 0 poll_done 永不退 (hart 1 没 trap / handler 没执行)
#   - hart 1 看 PTE.A == 0 (改造没真 set A or hart 0 没 trigger walker)
#   - exit_code != 0 (FAIL 路径)
