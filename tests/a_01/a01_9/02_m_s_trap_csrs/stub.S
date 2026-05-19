# a_01_9 / 02_m_s_trap_csrs
# 验 7 个 csr (mscratch + medeleg + mideleg + sscratch + stvec + scause + stval) csrw +
# csrr 字段值。
#
# 测试组织: 本 fixture 只在 M 态测 csrw/csrr 字段读写, 不测 trap delivery (那归
# a_01_9/03 medeleg_deliver_s)。stvec WARL "MODE 低 2 位截到 0" 描述: csrw 0x80100003
# 写入 mode = 0b11 reserved → WARL 落 0, 结果 0x80100000; 跟"接受 mode=0/1 + reserved
# 落 0"的真 WARL 行为巧合一致, 期望值仍对 (本 fixture 不测 mode=1 vectored case)。
#
# 模板: 纯 .S (跟 fixture a01_9/01 同; 没 PT 建立工作)。
#
# 测试矩阵 (7 个 csr + 期望读回值; 写入值见 stub.S 内 li):
#   mscratch (0x340)  csrw 0xCAFEBABE → csrr 期望 0xCAFEBABE   (RW 任意值)
#   medeleg  (0x302)  csrw 0xFFFFFFFF → csrr 期望 0xFFFFF7FF   (bit 11 ecall_from_M WARL hardwire 0)
#   mideleg  (0x303)  csrw 0xDEADBEEF → csrr 期望 0xDEADBEEF   (项目当前简化, 接受全 32 位)
#   sscratch (0x140)  csrw 0x12345678 → csrr 期望 0x12345678   (RW 任意值)
#   stvec    (0x105)  csrw 0x80100003 → csrr 期望 0x80100000   (MODE 低 2 位 WARL 截到 0)
#   scause   (0x142)  csrw 9          → csrr 期望 9            (RW 全 32 位接受)
#   stval    (0x143)  csrw 0x80001000 → csrr 期望 0x80001000   (RW 全 32 位接受)
#
# main dump 范围 = x4-x7 + x10-x15 共 8 GPR; 7 个 csr 读回值分配如下:
#   x10 mscratch / x11 medeleg / x12 mideleg / x13 sscratch /
#   x14 stvec    / x15 scause  / x4  stval (m_handler 地址 csrw 完不再用, x4 重用)
#
# 关键验证点:
#   - x10/x12/x13/x15/x4 = 写入值原样回读 → csr 字段读写路径走通
#   - x11 = 0xFFFFF7FF (bit 11 清 0) → medeleg WARL hardwire bit 11 真生效
#   - x14 = 0x80100000 (低 2 位清 0) → stvec WARL MODE 截 (跟 mtvec 同)
#   - mcause = 11 (handler 末 ecall, 跟 fixture a01_9/01 同, 验 csr_op trap delivery 没退化)

.section .text
.option norvc
.globl _start
_start:
    # ---- 设 M handler ----
    la   x4, m_handler
    csrw mtvec, x4

    # ---- mscratch (0x340) ----
    li   x6, 0xCAFEBABE
    csrw 0x340, x6
    csrr x10, 0x340             # 期望 0xCAFEBABE

    # ---- medeleg (0x302) — bit 11 hardwire 0 验 ----
    li   x6, -1                 # 0xFFFFFFFF
    csrw 0x302, x6
    csrr x11, 0x302             # 期望 0xFFFFF7FF (= 0xFFFFFFFF & ~(1<<11))

    # ---- mideleg (0x303) — 项目当前接受全 32 位写, 不 mask ----
    li   x6, 0xDEADBEEF
    csrw 0x303, x6
    csrr x12, 0x303             # 期望 0xDEADBEEF

    # ---- sscratch (0x140) ----
    li   x6, 0x12345678
    csrw 0x140, x6
    csrr x13, 0x140             # 期望 0x12345678

    # ---- stvec (0x105) — MODE 低 2 位 WARL 截 ----
    li   x6, 0x80100003         # MODE 故意写 11 (Reserved); WARL 应截到 00 (Direct)
    csrw 0x105, x6
    csrr x14, 0x105             # 期望 0x80100000 (低 2 位 0)

    # ---- scause (0x142) ----
    li   x6, 9
    csrw 0x142, x6
    csrr x15, 0x142             # 期望 9

    # ---- stval (0x143); x4 重用 (m_handler 地址 csrw 完不再用) ----
    li   x6, 0x80001000
    csrw 0x143, x6
    csrr x4, 0x143              # 期望 0x80001000

    ecall                       # cause=11 from M; trap deliver M handler

m_handler:
halt:
    csrw mtvec, x0
    ecall                       # cause=11 from M, mtvec=0 fetch fail → triple fault
