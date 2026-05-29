# a03_1 / 02_plic_access_fault
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# PLIC 边界 fault 语义 — "PLIC region 内 silent (除 reserved fprintf 报告) + PLIC
# 区外 bus 自产 fault" 切分验证。
#
# RV PLIC spec / QEMU sifive_plic 实现一致: PLIC region 内的越界 (src_id / ctx_id
# 越界) + reserved (context 区 off_in_ctx 不是 0/4) 都 silent ignore 不 fault;
# 只有 PLIC region 外 (>= PLIC_BASE + PLIC_SIZE = 0x0C600000) 才走 bus 自产
# CAUSE_*_ACCESS_FAULT。详 src/platform/plic.c plic_read/write 顶段 doc。
#
# ===== 思路 =====
# 7 个 case 切两组:
#
#   PLIC 内 silent (5 case, 都不 fault 不进 handler):
#     case 0: priority src_id=96 越界 sw (off=PLIC_BASE+96*4=0x0C000180)   silent
#     case 1: pending  word_idx=3 越界 lw (off=PLIC_BASE+0x100C)            silent
#     case 2: enable   ctx_id=2  越界 sw (off=PLIC_BASE+0x2100)             silent
#     case 3: context  ctx_id=2  越界 lw (off=PLIC_BASE+0x202000)           silent
#     case 4: context  ctx 0 reserved sw (off=PLIC_BASE+0x200008)           silent + fprintf
#                                                                           "[plic] ctx=0 reserved W"
#
#   PLIC 外 bus 自产 fault (2 case, 都进 handler):
#     case 5: PLIC 区外 sw (off=0x0C600000 = PLIC_BASE + PLIC_SIZE)         cause 7
#     case 6: PLIC 区外 lw (off=0x0C600004)                                 cause 5
#
# 设计 (跟 tests/a_02/a02_2/02_bus_unmapped_reject 同模式):
#   - mtvec 指 m_handler, handler 按 mcause 分流计数 (s2 = load fault, s3 = store
#     fault); 写 mepc+=4 跳过 sw/lw 指令 (.option norvc 保证 4-byte) + mret
#   - 所有 case 跑完后 csrw mtvec, x0 + ecall → triple fault halt
#   - main dump 验 s0 / s2 / s3 寄存器

.section .text
.option norvc
.globl _start
_start:
    csrw 0x310, x0                   # 清 mstatush.MDT (Smdbltrp reset=1; case 5/6 fault 进
                                     # handler, 否则首个 fault trap 即 MDT double-trap abort)
    # set mtvec
    la   t0, m_handler
    csrw mtvec, t0

    # init counters
    li   s0, 0                       # case 进 handler 总数 (期望 = 2; 只 case 5/6 进)
    li   s2, 0                       # load  fault count (cause 5)
    li   s3, 0                       # store fault count (cause 7)

    # ========= PLIC 内 silent (5 case, 都不该进 handler) =========

    # === case 0: priority src_id=96 越界 sw → silent ===
    li   t0, 0x0C000180              # PLIC_BASE + 96*4
    li   t1, 7
    sw   t1, 0(t0)

    # === case 1: pending word_idx=3 越界 lw → silent ===
    li   t0, 0x0C00100C              # PLIC_BASE + 0x1000 + 3*4
    lw   t1, 0(t0)

    # === case 2: enable ctx_id=2 越界 sw → silent ===
    li   t0, 0x0C002100              # PLIC_BASE + 0x2000 + 2*0x80
    li   t1, 0xFF
    sw   t1, 0(t0)

    # === case 3: context ctx_id=2 越界 lw → silent ===
    li   t0, 0x0C202000              # PLIC_BASE + 0x200000 + 2*0x1000
    lw   t1, 0(t0)

    # === case 4: context ctx 0 reserved sw (off_in_ctx=8) → silent + fprintf ===
    li   t0, 0x0C200008              # PLIC_BASE + 0x200000 + 0x8
    li   t1, 0x55
    sw   t1, 0(t0)

    # ========= PLIC 外 bus 自产 fault (2 case, 都进 handler) =========

    # === case 5: PLIC 区外 sw → bus 自产 cause 7 ===
    li   t0, 0x0C600000              # PLIC_BASE + PLIC_SIZE (PLIC 区外第一个地址)
    li   t1, 0xAA
    sw   t1, 0(t0)

    # === case 6: PLIC 区外 lw → bus 自产 cause 5 ===
    li   t0, 0x0C600004              # PLIC_BASE + PLIC_SIZE + 4
    lw   t1, 0(t0)

    # done: test_dev FINISHER PASS 干净停机 (scratch x30/x31)
    li   x30, 0x100000               # test_dev base
    li   x31, 0x5555                 # FINISHER PASS magic
    sw   x31, 0(x30)
1:  j       1b                      # 防 fall-through: SRS=0 下个 block 边界退

# ----------------------------------------------------------------------------
# m_handler — 按 mcause 分流 (5=load fault s2++, 7=store fault s3++);
# 其他 cause (例如 cause 1 取指 fault, 或意外 PLIC 内访问触发 fault) 直接 halt
# 进异常路径暴露问题。mepc += 4 跳过 sw/lw (.option norvc 保证 4-byte) + mret。
# ----------------------------------------------------------------------------
m_handler:
    csrr t0, mcause

    li   t1, 5
    beq  t0, t1, load_fault
    li   t1, 7
    beq  t0, t1, store_fault

    # unexpected cause: halt error
    csrw mtvec, x0
    ecall

load_fault:
    addi s2, s2, 1
    j    epilogue

store_fault:
    addi s3, s3, 1
    j    epilogue

epilogue:
    addi s0, s0, 1
    csrr t3, mepc
    addi t3, t3, 4                   # sw / lw 都是 4 byte (.option norvc)
    csrw mepc, t3
    mret

# ===== 期望结果 =====
# 期望最终寄存器:
#   s0 (x8)  = 2            (只 case 5/6 进 handler; case 0..4 PLIC 内 silent 不进)
#   s2 (x18) = 1            (load  fault: case 6)
#   s3 (x19) = 1            (store fault: case 5)
#
# 停机:
#   test_dev FINISHER PASS → exit code 0 (SRS=0 退主循环, 非 trap / abort)
#   x30 = 0x100000 / x31 = 0x5555 (停机 scratch: test_dev base / FINISHER PASS magic)
#   (cause 5/7 通过 handler 写入 s2/s3 验证, 不存在 main dump 里)
# (注: 原停机靠 csrw mtvec=0 + ecall triple fault; a_03 Smdbltrp 后变 MDT abort,
#  改 test_dev 干净停机。unexpected-cause 错误逃逸路径仍走 ecall。
#  绝对 pc / total_count 见 REVIEW B.3 段。)
#
# 期望 stderr 只多一行 (case 4 reserved fprintf):
#   [plic] ctx=0 reserved W
#   (case 0/1/2/3 都是越界 silent 不 fprintf; case 5/6 是 PLIC 区外不进 plic_*,
#    bus 自产 fault 也不 fprintf)
#
# 不期望:
#   - s0 > 2 (说明 PLIC 内某 case 也 fault 了, 实装 silent 失败)
#   - cause 1 / 2 / 0 进 handler (会走 unexpected halt path)
#   - 程序崩溃 (ASan / UBSan 报错)
