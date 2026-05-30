# a03_14 / 01_clint_ipi
#
# ===== 运行参数 =====
# RUN-SMP: 2
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 2-hart CLINT msip IPI 端到端 — 项目第一个跨 hart fixture。验:
#   1. 多 hart spawn (--smp 2) hart 0 + hart 1 都从同 reset vector 起跑
#   2. mhartid 分流 (csrr mhartid → branch)
#   3. hart 0 写 CLINT msip[1] = 1 → 跨 hart trigger hart 1 MSIP
#   4. hart 1 trap handler 收 MSIP → 清 msip[1] → 写 done flag → PASS
#   5. hart 0 poll done flag → PASS
#   两 hart 都 reach FINISHER_PASS sw (test_dev sifive PASS 幂等, race 无害) →
#   main exit 0
#
# ===== 思路 =====
# 同 reset vector 启动 (RV spec), csrr mhartid 分流: hart 0 = sender, hart 1 =
# receiver。
#
# hart 0 (sender): 写 msip[1] (CLINT_BASE + 4) = 1 → poll DONE_FLAG_ADDR 等
# hart 1 完成 → 写 test_dev PASS。
#
# hart 1 (receiver): 装 msi_handler 到 mtvec (direct mode), 开 MIE.MSIE (bit3) +
# MSTATUS.MIE (bit3), 进 wfi 等中断。trap 后 handler 清 msip[1]=0 (防 MSIP 持续
# set 重入) → 写 DONE_FLAG_ADDR=1 → 直接写 test_dev PASS (不 mret 回主帧, 简化)。
#
# DONE_FLAG_ADDR = 0x80100000 (远离 .text, 跟 virtio_blk fixture 同段)。两 hart
# 跨 hart RAM 写共享: word 写是 atomic on RV32 (单 sw 不撕裂); hart 0 lw poll 看
# hart 1 sw 写 1 是 plain RAM 路径, 跟 host 端 _Atomic 跨线程 RAM 模型对齐 (本
# fixture 不验 memory ordering 严格性, 只验 sw 跨 hart 最终可见)。

.equ CLINT_BASE,        0x02000000
.equ MSIP_HART1,        0x02000004      # CLINT_BASE + 4 (msip[1])

.equ TEST_DEV_BASE,     0x00100000
.equ FINISHER_PASS,     0x5555
.equ FINISHER_FAIL,     0x3333

.equ DONE_FLAG_ADDR,    0x80100000

.equ MIE_MSIE,          0x8             # MIE bit 3 = MSIE
.equ MSTATUS_MIE,       0x8             # MSTATUS bit 3 = MIE

.section .text
.option norvc
.globl _start

_start:
    # ---- Boot prelude (两 hart 都跑): 清 MDT / SDT 让 MIE/SIE 写得进 ----
    # (Smdbltrp/Ssdbltrp reset 后 MDT=SDT=1, csrw mstatus 写 MIE=1 会被硬件强制
    # 0; trap 撞 MDT/SDT=1 即 Double Trap critical-error → 整机 abort。跟现有
    # a03_7/01 / a03_6/02 体例对齐。)
    csrw    0x310, x0                   # mstatush 清 MDT
    csrw    sstatus, x0                 # 清 SDT (顺便清 SIE/SPIE)

    csrr    t0, mhartid
    beqz    t0, hart0_path              # mhartid==0 → sender
    li      t1, 1
    beq     t0, t1, hart1_path          # mhartid==1 → receiver
    j       fail                        # mhartid >= 2 (本 fixture 只 --smp 2)

# ---- hart 0 (sender) ----
hart0_path:
    # 写 msip[1] = 1 触发 hart 1 MSIP
    li      t1, MSIP_HART1
    li      t2, 1
    sw      t2, 0(t1)

    # poll DONE_FLAG_ADDR 直到 hart 1 写 1
    li      t3, DONE_FLAG_ADDR
poll_done:
    lw      t4, 0(t3)
    beqz    t4, poll_done

    # PASS
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_PASS
    sw      t1, 0(t0)
1:  j       1b                          # 防 fall-through

# ---- hart 1 (receiver) ----
hart1_path:
    # 装 msi_handler 到 mtvec (direct mode: bits 1:0 = 0)
    la      t0, msi_handler
    csrw    mtvec, t0

    # 开 MIE.MSIE (csrw 覆盖 mie, 本 fixture 不开其它源, OK)
    li      t1, MIE_MSIE
    csrw    mie, t1

    # 开 MSTATUS.MIE (csrs 不覆盖其它 bit, 安全 set)
    li      t1, MSTATUS_MIE
    csrs    mstatus, t1

    # wfi 等中断; spurious wake 兜底重新 wfi
1:  wfi
    j       1b

# ---- hart 1 trap handler ----
msi_handler:
    # 清 msip[1] = 0 防 MSIP 持续 set + handler 退出后立刻重入
    li      t1, MSIP_HART1
    sw      x0, 0(t1)

    # 写 DONE_FLAG_ADDR = 1 通知 hart 0
    li      t3, DONE_FLAG_ADDR
    li      t4, 1
    sw      t4, 0(t3)

    # PASS (从 trap handler 直接 sifive PASS 简化, 不 mret 回主帧)
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_PASS
    sw      t1, 0(t0)
1:  j       1b

# ---- 共用 fail ----
fail:
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_FAIL
    sw      t1, 0(t0)
2:  j       2b

# ===== 期望结果 =====
#   两 hart 都 reach FINISHER_PASS sw (test_dev sifive PASS 幂等; race 无害)
#   exit_code = 0 (FINISHER_PASS)
#   main wall time 量级 < 100ms (poll_done busy-loop ≤ hart 1 trap dispatch 延迟)
#   无 fault / 无 ASan / 无 timeout
# 不期望:
#   - poll_done 永不退 (hart 1 没 trap / msi_handler 没执行 / done flag 没写)
#   - hart 1 trap loop 重入 (MSIP 没清 → handler 退出后立刻重 trap)
#   - mhartid >= 2 走 fail (本 fixture 只 --smp 2)
#   - exit_code != 0 (FAIL 路径)
#   - 任何 [bus] ACCESS_FAULT / ASan 报错
