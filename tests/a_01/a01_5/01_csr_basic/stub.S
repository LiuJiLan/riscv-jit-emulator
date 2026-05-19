# a_01_5 / 01_csr_basic
# 验证 csr decode 路径 + csr_op 大 helper 分发 + 硬边界 dispatcher 多块循环。
# 8 条指令 = 32 字节 (32-bit only, .option norvc 强制不生 RVC)。
#
# 期望执行后:
#   x10 = 0x42       ← addi 设置的 csr 写源值
#   x1..x6 = 0       ← csr stub read 都返 0, 写 rd 都拿 0
#   pc = 0x8000001c  ← ecall 地址 (OP_UNSUPPORTED 时 pc 停在触发指令本身)
#
# 期望 dispatcher 行为:
#   每条 csr 是硬边界, 各独立成块。
#     iter 1: addi(non-boundary) + csrrw(boundary), count_out=2, 退出 fetch loop
#     iter 2: csrrs(boundary), count_out=1
#     iter 3: csrrc(boundary), count_out=1
#     iter 4: csrrwi(boundary), count_out=1
#     iter 5: csrrsi(boundary), count_out=1
#     iter 6: csrrci(boundary), count_out=1
#     iter 7: ecall(OP_UNSUPPORTED), count_out=0 → dispatcher break
#   total_count = 2+1+1+1+1+1+0 = 7
#
# 期望 stderr csr_op 占位日志顺序 (Makefile 顶部已列):
#   read mtvec → write mtvec=0x42
#   read mstatus → write mstatus=0x42
#   read mcause → write mcause=0      (read_old=0 & ~0x42 = 0)
#   read mepc → write mepc=0x7        (csrrwi zimm=7)
#   read mtval                          (csrrsi zimm=0 不写)
#   read mtvec → write mtvec=0        (csrrci zimm=31 但 read_old=0 & ~0x1F = 0)
#
# 注: 本 fixture 描述写于 csr stub 阶段 (csr 无物理存储, read 永远返 0); csr 已完整
#     实装后期望寄存器值不再全 0 (例 x6 = csrrci 读 mtvec 应是上次 csrrw 写入的 0x42
#     不是 0)。fixture 期望校准留 T6.2 (fixture 注释统一); 本 fixture 验证的核心
#     ("decode + 大 switch 分发 + 硬边界 + dispatcher 多块") 仍成立。

.section .text
.option norvc                       # 强制不生 RVC; csr 指令本身就是 32-bit
.globl _start
_start:
    addi   x10, x0, 0x42        # 0x80000000: x10 = 0x42 (csr 写源值)         (0x04200513)
    csrrw  x1, mtvec, x10       # 0x80000004: csr=mtvec, RW                  (0x305510F3)
    csrrs  x2, mstatus, x10     # 0x80000008: csr=mstatus, RS                (0x30052173)
    csrrc  x3, mcause, x10      # 0x8000000c: csr=mcause, RC                 (0x342531F3)
    csrrwi x4, mepc, 7          # 0x80000010: csr=mepc, RWI zimm=7           (0x3413D273)
    csrrsi x5, mtval, 0         # 0x80000014: csr=mtval, RSI zimm=0 (不写)   (0x343062F3)
    csrrci x6, mtvec, 31        # 0x80000018: csr=mtvec, RCI zimm=31         (0x305FF373)
    ecall                       # 0x8000001c: OP_UNSUPPORTED, dispatcher halt (0x00000073)
