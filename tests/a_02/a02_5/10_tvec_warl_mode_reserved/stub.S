# a02_5 / 10_tvec_warl_mode_reserved
#
# ===== 测试目的 =====
# csrw mtvec / stvec WARL mode bit 处理 (D9 解绑后):
#   mode=0 Direct       — 接受
#   mode=1 Vectored     — 接受
#   mode=2/3 Reserved   — RV Priv Spec §3.1.7 WARL 允许实现拒绝; 项目落 0 (合法值)
# 测试 mtvec + stvec 两个 csr 入口, 各 4 个 mode case (2/3/1/0 顺序). 不触发任何 trap.
#
# ===== 思路 =====
#   验证步骤: csrw 各 mode 值 → csrr 读回 → 期望 mode bit 等于实际 store 的值
#   (mode=2/3 store 后读回 mode=0; mode=0/1 store 后读回 mode=0/1).
#
# 实现验证: src/core/csr.c::csr_mtvec_write / csr_stvec_write 内 WARL 处理 mode bit
#   (汇编 preprocessor 不识别 C if-语法, 注释省去等价 C 片段; 看 csr.c 实际实现).

.section .text
.option norvc
.globl _start
_start:
    # base addr 占位 (任意值; csr_*tvec_write 不校验 base 有效性, 只看 mode bit WARL)
    li      t0, 0x80000100

    # ---- mtvec WARL ----

    # mode=2 写入 → 落 mode=0
    ori     t1, t0, 2
    csrw    mtvec, t1
    csrr    a0, mtvec            # 期望 0x80000100 (mode 落 0)

    # mode=3 写入 → 落 mode=0
    ori     t1, t0, 3
    csrw    mtvec, t1
    csrr    a1, mtvec            # 期望 0x80000100

    # mode=1 写入 → 接受
    ori     t1, t0, 1
    csrw    mtvec, t1
    csrr    a2, mtvec            # 期望 0x80000101 (mode=1 保留)

    # mode=0 写入 → 接受
    csrw    mtvec, t0            # base 自身 mode bit = 0
    csrr    a3, mtvec            # 期望 0x80000100

    # ---- stvec WARL (跟 mtvec 同形态) ----

    ori     t1, t0, 2
    csrw    stvec, t1
    csrr    a4, stvec            # 期望 0x80000100

    ori     t1, t0, 3
    csrw    stvec, t1
    csrr    a5, stvec            # 期望 0x80000100

    ori     t1, t0, 1
    csrw    stvec, t1
    csrr    a6, stvec            # 期望 0x80000101

    csrw    stvec, t0
    csrr    a7, stvec            # 期望 0x80000100

    # halt: 清 mtvec + ecall → triple fault
    csrw    mtvec, x0
    ecall

# ===== 期望结果 =====
# 期望最终寄存器:
#   a0 (x10) = 0x80000100   mtvec mode=2 → 0
#   a1 (x11) = 0x80000100   mtvec mode=3 → 0
#   a2 (x12) = 0x80000101   mtvec mode=1 接受
#   a3 (x13) = 0x80000100   mtvec mode=0 接受
#   a4 (x14) = 0x80000100   stvec mode=2 → 0
#   a5 (x15) = 0x80000100   stvec mode=3 → 0
#   a6 (x16) = 0x80000101   stvec mode=1 接受
#   a7 (x17) = 0x80000100   stvec mode=0 接受
#
# 关键验证: mode=2/3 reserved 路径 WARL 落 0 + mode=0/1 store/read 完整保留 base + mode bit.
