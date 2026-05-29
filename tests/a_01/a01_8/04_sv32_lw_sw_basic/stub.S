# a_01_8 / 04_sv32_lw_sw_basic
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验 lsu SV32 在 4KB 页 (level=1 → level=0 leaf) 上 lw/sw 走通 + 第二次 fast path。
# 跟 fixture 02 (4MB superpage fetch) 互补 — 02 走 level=1 leaf, 04 走 level=0 leaf。
# 不混 superpage; superpage lw/sw 留给 fixture 07。
#
# ===== 思路 =====
# .S+.c 模板首次启用 (a_01_8 fixture 验证 task 起步): main.c 仅放 setup_pt() — 用
# volatile uint32_t* 直接写 PA, 比 stub.S 内 lui+addi+sw 链可读性高很多。stub.S 仍
# 写 _start + S phase + handler (主线汇编)。setup_pt 被 _start call 调用, 需要 sp.
#
# PT layout (4KB 页路径):
#   root_pt @ PA 0x80100000:
#     [0x200] = pointer-to-next-level → leaf_pt PPN | V    = 0x20040401
#   leaf_pt @ PA 0x80101000:
#     [0]     (vaddr 0x80000000) = PPN 0x80000 | V|R|X|A   = 0x2000004B  (代码段)
#     [1]     (vaddr 0x80001000) = PPN 0x80200 | V|R|W|A|D = 0x200800C7  (数据段)
#   数据 page @ PA 0x80200000 (RAM 内空闲, 不跟代码/PT 冲突)
#
# satp = (1<<31) | 0x80100 = 0x80080100
#
# 控制流:
#   1. M setup: sp + mtvec; call setup_pt() (建 PT); csrw satp + sfence.vma;
#      mepc=s_target identity vaddr + mstatus.MPP=PRIV_S → mret
#   2. S phase 走 walker leaf_pt[0]/[1] 4KB 页:
#      sw 0xCAFEBABE → vaddr 0x80001000 (TLB miss → walker fall back → fill TLB)
#      lw vaddr 0x80001000 → x10  (TLB hit fast path, 第一次 sw 已 fill)
#      sw 0x12345678 → vaddr 0x80001004 (TLB hit fast path)
#      lw vaddr 0x80001004 → x11  (TLB hit fast path)
#      ecall (cause=9 from S; trap deliver M)
#   3. M handler: csrr mcause=9; test_dev FINISHER PASS 干净停机

.equ CSR_TOHOST, 0x800
.equ CSR_PRIVRD, 0xCC0

.section .text
.option norvc
.globl _start
_start:
    # ---- 设 sp (栈在 RAM 内空闲区, setup_pt() C 函数 prologue/epilogue 用) ----
    # sp = 0x80300000 (代码 0x80000000-0x80000xxx + PT 0x80100000-0x80101FFF +
    # 数据 page 0x80200000-0x80200FFF 都在 sp 之下, 不冲突)
    lui  sp, 0x80300

    # ---- 设 M handler ----
    la   x4, m_handler
    csrw mtvec, x4

    # ---- C 函数建 PT (BARE M-mode, identity, walker 未激活) ----
    call setup_pt

    # ---- 设 satp: MODE=Sv32 + PPN=0x80100 (= root_pt PA >> 12) ----
    lui  x6, 0x80080
    addi x6, x6, 0x100              # satp = 0x80080100
    csrw satp, x6
    sfence.vma                      # 让新 satp 生效 (CLAUDE.md: 不自动 sfence)

    # ---- 设 mret 目标 = s_target identity vaddr (走 walker leaf_pt[0]) ----
    la   x7, s_target
    csrw mepc, x7

    # ---- 设 mstatus.MPP = PRIV_S ----
    addi x4, x0, 1
    slli x4, x4, 11                 # x4 = 0x800 (MPP=01=S)
    csrw mstatus, x4

    mret                            # M→S, pc=identity vaddr

s_target:
    # ---- S phase, 走 walker 经 leaf_pt[1] 4KB 页 (vaddr 0x80001000 → PA 0x80200000) ----
    lui  x4, 0x80001                # x4 = 0x80001000 (数据段 vaddr)

    # 第一次 sw —— TLB miss → store_helper fall back walker → set A+D 写回 PT (这里
    # PT init A=D=1 已 set, walker 跳过 memcpy 写回) → fill TLB → host store
    li   x5, 0xCAFEBABE
    sw   x5, 0(x4)
    lw   x10, 0(x4)                 # x10 = 0xCAFEBABE (TLB hit fast path)

    # 第二次 sw + lw 同 page —— 全部 fast path (V+tag+W+D 命中)
    li   x6, 0x12345678
    sw   x6, 4(x4)
    lw   x11, 4(x4)                 # x11 = 0x12345678 (TLB hit fast path)

    ecall                           # cause=9 from S; trap deliver M

m_handler:
    csrr x12, mcause                # 期望 cause=9 (ecall from S)
    addi x13, x0, 9
    bne  x12, x13, halt             # 防御 (期望 fall through)

halt:
    li   x30, 0x100000              # test_dev base
    li   x31, 0x5555                # FINISHER PASS magic
    sw   x31, 0(x30)                # 干净停机: test_dev 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望 main.c dump:
#   x10    = 0xCAFEBABE   (第一次 lw, 同地址刚被 sw)
#   x11    = 0x12345678   (第二次 lw, 同地址刚被 sw)
#   x12    = 9            (ecall_from_S cause)
#   x13    = 9            (比较 reg)
#   x30    = 0x100000     (停机用 scratch: test_dev base)
#   x31    = 0x5555       (停机用 scratch: FINISHER PASS magic)
#   priv   = 3            (M)
#   停机 = test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
# (注: 绝对 pc / total_count 随 test_dev 收尾漂, 见 REVIEW B.3 段; 不在断言内。)
#
# 关键验证点:
#   - x10 / x11 都对 → lsu SV32 4KB lw/sw 路径全跑通 (含 walker fall back + fast path)
#   - x12 = 9 → ecall from S; trap delivery 到 M handler 走通 (跟 fixture 02 同)
#   - 跟 fixture 02 比, root_pt[0x200] 是非叶子 (R=W=X=0), walker 真走 level=0 leaf
