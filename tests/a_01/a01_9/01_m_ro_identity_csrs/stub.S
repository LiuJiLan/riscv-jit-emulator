# a_01_9 / 01_m_ro_identity_csrs
#
# ===== 测试目的 =====
# 验 5 个 M-mode RO Identity CSR (mhartid + misa + mvendorid + marchid + mimpid)。
#
# ===== 思路 =====
# 测试组织: a_01_8 fixture 文件夹只放 a_01_8 主轴 fixture (SV32 walker); a01_9
# 新增 csr 整理类 fixture 单独在 tests/a_01/a01_9/ 下。
#
# 模板: 纯 .S (没 PT 要建, 没 C 函数容器需求, 跟 fixture 02/03 同形态; 跟 a_01_8 04-09 的
# .S+.c 模板相对; .c 模板留给真要 PT 的 fixture)。
#
# 控制流:
#   M-mode 默认起步 (cpu_create priv=M, satp=0 BARE; mtvec=0 memset)
#   1. 设 mtvec → m_handler (沿用 a_01_8 模板让 mcause 最终保留 11; 不设的话 mcause 末值
#      会是 fetch from 0 fail 的 cause 1)
#   2. csrr 5 个 RO Identity CSR → x10-x14
#   3. csrw misa, 0xFFFFFFFF + csrr misa → x15 (验 RV spec WARL noop, 期望 仍 0)
#   4. ecall → m_handler → halt triple fault
#
# cpu_create 入参 (main.c): cpu_create(/*misa*/0, /*mhartid*/0); 即所有 RO Identity 默认 0。
#   - mhartid = 0 (cpu_create mhartid 入参; 写入 hart->per_hart_info.mhartid)
#   - misa    = 0 (cpu_create misa 入参; 写入 hart->per_hart_info.misa)
#   - mvendorid/marchid/mimpid = 0 (cpu.c static const cpu_info_shared_default 全 0)
#
# 关键验证点:
#   - x10-x14 全 0 → 5 个 RO Identity CSR 走 csr.c csr_op switch read path; 数据来源:
#     mhartid/misa 来自 hart->per_hart_info, mvendorid/marchid/mimpid 来自 hart->shared_info
#     解引用 (a01_9 step 3 拆 4a per-hart + 4b shared 两组)
#   - x15 = 0 而不是 0xFFFFFFFF → csr_misa_write 真的 noop (RV spec §3.1.1 WARL hardwire)
#   - 不验 U-mode trap (mhartid/mvendorid/marchid/mimpid 在 U 读应 trap cause 2 illegal;
#     单独 fixture 留待后续, 跟 priv 入口判一组测)

.section .text
.option norvc
.globl _start
_start:
    # ---- 设 M handler ----
    la   x4, m_handler
    csrw mtvec, x4

    # ---- 读 5 个 RO Identity CSR (csrr 编码: csrrs rd, csr, x0; new_val=0 不触发写) ----
    csrr x10, 0xF14         # mhartid; 期望 0
    csrr x11, 0x301         # misa;    期望 0
    csrr x12, 0xF11         # mvendorid; 期望 0
    csrr x13, 0xF12         # marchid; 期望 0
    csrr x14, 0xF13         # mimpid;  期望 0

    # ---- 验 misa csrw noop (RV spec WARL hardwire) ----
    # csrw misa, 0xFFFFFFFF; csr_op 入口判 misa addr [11:10]=00 不是 RO, csrw 通过;
    # csr_misa_write 内部 noop, hart->per_hart_info.misa 保持 0;
    # 然后 csrr misa 应仍读 0
    li   x5, -1             # x5 = 0xFFFFFFFF (li 展开为 lui+addi 或 addi -1)
    csrw 0x301, x5          # csrw misa, x5
    csrr x15, 0x301         # 期望仍 0

    ecall                   # cause=11 from M; trap deliver M handler

m_handler:
    # 不验 cause, 直接 halt (mcause 最终 = 11; trap dump 保留第 2 次, 即 handler 末 ecall)

halt:
    csrw mtvec, x0
    ecall                   # cause=11 from M → mmu fail (mtvec=0) → triple fault

# ===== 期望结果 =====
# 期望 main dump:
#   x10 = 0    (mhartid)
#   x11 = 0    (misa)
#   x12 = 0    (mvendorid)
#   x13 = 0    (marchid)
#   x14 = 0    (mimpid)
#   x15 = 0    (csrw misa 0xFFFFFFFF 后 csrr misa, 验 noop)
#   mcause = 11 (handler 末 ecall_from_M, 字段保留第 2 次)
#   priv = 3, in_trap = 3, mtvec = 0
