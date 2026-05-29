# a_01_6 / 03_store_oor_reject
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# 验证 store_helper 的 RAM 边界检查: sw 到 RAM 之外 (0x40000000) → trap_raise(7 store access fault)。
# stderr 还会有一行 "[lsu] store: PA 0x40000000 not in RAM (MMIO bus_dispatch not implemented in
# a_01_6)" 由 store_helper 的 fprintf 输出 (a_01_6 占位提示, 未来 a_01_7+ 接 bus_dispatch 时替换)。
# (注: 原收尾用 ecall + triple fault; a_03 后改 test_dev 干净停机 + 开头清 MDT prelude
#  让 store-oor trap 能投递到 handler。)
#
# ===== 思路 =====
# 触发条件 (lsu.c store_helper Step 2 BARE 路径):
#   (uint32_t)(pa - GUEST_RAM_START) >= GUEST_RAM_SIZE
# pa=0x40000000, GUEST_RAM_START=0x80000000, GUEST_RAM_SIZE=128MB:
#   (0x40000000 - 0x80000000) cast u32 = 0xC0000000 = 3 GiB > 128 MiB → access fault cause=7。
# RAM 区是 [0x80000000, 0x88000000); 0x40000000 不在 (在 RAM 起点之下)。misalign 不撞 (4-byte 对齐)。
# 控制流:
#   _start: 清 MDT → la+csrw 设 mtvec=handler → lui+addi 算地址+值 → sw (oor → trap)
#   trap → handler: csrr x1=mcause(7), x2=mtval(0x40000000), x3=mepc(sw PC) → test_dev PASS 停机

.section .text
.option norvc
.globl _start
_start:
    csrw 0x310, x0          # 清 mstatush.MDT (否则 store-oor trap 撞 MDT=1 直接 abort, handler 跑不到)
    la   x4,  handler       # auipc + addi (la pseudo)
    csrw mtvec, x4          # mtvec = handler

    # ---- 触发 store oor (PA 0x40000000 不在 RAM) ----
    lui  x6,  0x40000       # x6 = 0x40000000 (不在 RAM, 4-byte 对齐)
    addi x10, x0, 0x42      # x10 = 0x42 (要 store 的值)
    sw   x10, 0(x6)         # sw 0x40000000 → store_helper access fault cause=7
    # ↑ trap_raise_exception _Noreturn longjmp; 不到此处

handler:
    csrr x1,  mcause        # x1 = mcause = 7 (store/AMO access fault)
    csrr x2,  mtval         # x2 = mtval = 0x40000000
    csrr x3,  mepc          # x3 = mepc = sw 指令的 PC

    # ---- 停机: test_dev FINISHER PASS ----
    li   x30, 0x100000      # test_dev base
    li   x31, 0x5555        # FINISHER PASS magic
    sw   x31, 0(x30)        # 置 exit_code=0 + SRS=0, dispatcher 退主循环

# ===== 期望结果 =====
# 期望:
#   x1 = 7          (mcause = store/AMO access fault, RV spec §3.1.16)
#   x2 = 0x40000000 (mtval = 触发 store 的 GVA; 数据相关, 不随代码平移)
#   x3 = sw 指令的 PC (mepc; 绝对值随 prelude +4 漂, 见 REVIEW B.3 段)
#   x30 = 0x100000 / x31 = 0x5555  (停机 scratch)
#   停机 = test_dev FINISHER PASS → exit code 0 (store-oor→handler 投递 OK)
# (注: 绝对 pc / total_count 见 REVIEW B.3 段; 不在断言内。)
