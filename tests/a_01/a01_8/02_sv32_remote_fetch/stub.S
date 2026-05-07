# a_01_8 / 02_sv32_remote_fetch
# 验证 SV32 walker 真激活 + mmu_translate_pc SV32 fetch 路径 (Step 6 加).
#
# 设计 (user "几个 gva 远页都指向第一个物理页"): 用 4MB superpage 把两个不同 vaddr
# 4MB-aligned 范围都映射到同一个 PA 4MB 范围 (RAM 起点)。S-mode 跑时 mret 后 pc =
# 远 vaddr (跨第一个 4MB superpage), mmu_translate_pc walker 真生效 → PA =
# 实际 fixture program 位置 → fetch 拉到 fixture inst → 跑下去。
#
# 路径:
#   1. M-mode: build PT (root_pt[0x200] + root_pt[0x210] 两个 4MB superpage 都指向 PA
#      0x80000000) + csrw satp + sfence.vma + csrw mstatus.MPP=PRIV_S + csrw mepc=远 vaddr
#      → mret 切 S
#   2. S-mode 远 vaddr (= 0x84000000 + s_target_offset): walker 跑 mmu_walk(MMU_PERM_X) →
#      4MB superpage leaf, PPN[1]=0x200 → PA = 0x80000000 + (vpn0 << 12) + offset = 实际
#      s_target 物理位置 → fetch 拉 s_target inst → 跑下去
#   3. S-mode addi x10/x11 标记 + ecall (cause=9 from S; trap deliver M)
#   4. M handler: 验 cause=9; halt triple fault
#
# 4MB superpage PTE 字段:
#   PPN[1] = (PA >> 22) = (0x80000000 >> 22) = 0x200 (12-bit 字段)
#   PPN[0] = 0 (4MB 自然对齐, walker 检查 PPN[0]==0 misalign)
#   PTE bits[31:10] PPN = (PPN[1]<<10) | PPN[0] = 0x80000
#   PTE = (0x80000 << 10) | 0xCF = 0x20000000 | 0xCF = 0x200000CF
#     0xCF = V|R|W|X|A|D = 0x01|0x02|0x04|0x08|0x40|0x80 (U=0 因为 S-mode 跑, 不要 U=1
#     否则 walker check_perm S+PTE.U=1 + SUM=0 → fail; 我们 S+PTE.U=0 PASS)
#
# satp:
#   MODE = 1 (Sv32) → bit 31
#   ASID = 0
#   PPN  = root_pt_pa >> 12 = 0x80100000 >> 12 = 0x80100
#   satp = (1 << 31) | (0 << 22) | 0x80100 = 0x80080100
#
# 远 vaddr 计算:
#   identity 映射 (root_pt[0x200]): vaddr 0x80000000-0x803FFFFF → PA 0x80000000-...
#   远映射  (root_pt[0x210]): vaddr 0x84000000-0x843FFFFF → PA 0x80000000-... (同物理)
#   即 vaddr_remote = vaddr_identity + 0x04000000 (= 偏移 64 MiB; 因为 vpn1 0x200 → 0x210
#     差 0x10, 4MB 一个槽; 0x10 * 4 MiB = 64 MiB)
#
# 期望 (main.c dump):
#   x10 = 1                  (S-mode 起步标记, 经 SV32 fetch 远 vaddr 跑过)
#   x11 = 2                  (S-mode 第二条标记)
#   x12 = 9                  (mcause = ecall from S, handler csrr 进读)
#   x13 = 9                  (比较 reg)
#   pc = 0                   (handler 末 ecall trap 跳 mtvec=0; triple fault)
#   in_trap = 3
#   mcause = 11              (handler 末 ecall = ecall from M; 字段保留第 2 次)
#   mepc = 0x80000xxx        (handler 末 ecall PC; 实际值看跑出来)
#   mtvec = 0
#   priv = 3                 (M)
#   mstatus = 0x1820 (?)     (推算同 fixture (f) 末段; SPIE 残留可能不一样, 跑出来核对)
#
# 关键验证点:
#   - x10 = 1 + x11 = 2 → SV32 fetch 真激活, 远 vaddr 通过 walker 找到 PA 跑 inst (没 SV32
#     真接的话, 远 vaddr 0x84000000+ 是非法 RAM 区 — 跑 BARE 的话会 page fault)
#   - x12 = 9 → cause=9 from S (ecall_from_S 跟 ecall_from_U cause=8 区分)
#   - 跨页 fetch 不 panic (可能此 fixture 不真跨 4K page, 因为 S 跑的 inst 都在同 4K 内;
#     真跨页要更复杂的 fixture, 留 Step 9)

.section .text
.option norvc
.globl _start
_start:
    # ---- 设 M handler ----
    la   x4, m_handler          # 0x00-0x07
    csrw mtvec, x4              # 0x08

    # ---- Build root PT @ PA 0x80100000 (1 MiB 偏移; 跟 5b 同位置) ----
    # 注: sw 的 imm 限 RV32 S-type 12-bit signed [-2048, 2047]; 直接写 0x800(x4) 超范围.
    # 解决: base x4 调到 root_pt + 0x800 (= entry 0x200 位置), 0 偏移 sw.
    lui  x4, 0x80101            # x4 = 0x80101000
    addi x4, x4, -0x800         # x4 = 0x80100800 (= root_pt + 0x200*4 = entry 0x200)

    # PTE 4MB superpage: V+R+W+X+A+D, PPN=0x80000 (= PA 0x80000000 >> 12)
    lui  x5, 0x20000            # x5 = 0x20000000
    addi x5, x5, 0xCF           # x5 = 0x200000CF (PTE)

    # root_pt[0x200] (vaddr 0x80000000) = PTE  → identity 4MB superpage
    sw   x5, 0(x4)              # x4 现在 = root_pt + 0x800

    # root_pt[0x210] (vaddr 0x84000000) = PTE  → 远 vaddr 同物理 (验证 SV32 fetch)
    addi x4, x4, 0x40           # x4 += 0x40 = root_pt + 0x840 = entry 0x210
    sw   x5, 0(x4)

    # ---- 设 satp: MODE=Sv32 + PPN=0x80100 ----
    lui  x6, 0x80080            # 0x20: x6 = 0x80080000
    addi x6, x6, 0x100          # 0x24: x6 = 0x80080100 (satp value)
    csrw satp, x6               # 0x28

    # ---- sfence.vma 让新 satp 生效 ----
    sfence.vma                  # 0x2c

    # ---- 设 mret 目标 = s_target 的远 vaddr (= s_target_PA + 0x04000000) ----
    la   x7, s_target           # 0x30-0x37: x7 = s_target PA (linker 钉 0x800000xx)
    lui  x4, 0x4000             # 0x38: x4 = 0x04000000 (复用 x4, 不再用 root_pt_pa)
    add  x7, x7, x4             # 0x3c: x7 = s_target 的远 vaddr (0x840000xx)
    csrw mepc, x7               # 0x40

    # ---- 设 mstatus.MPP = PRIV_S (mret 切 S) ----
    addi x4, x0, 1              # 0x44
    slli x4, x4, 11             # 0x48: x4 = 0x800
    csrw mstatus, x4            # 0x4c

    # ---- mret → priv=S, pc=远 vaddr → mmu_translate_pc walker 真激活 ----
    mret                        # 0x50

s_target:
    # ---- S-mode (远 vaddr fetch 经 SV32 walker 找到这里; 实际 PA 0x80000054) ----
    addi x10, x0, 1             # 0x54: SV32 fetch 真激活证据
    addi x11, x0, 2             # 0x58
    ecall                       # 0x5c: cause=9 from S; trap deliver M (没 medeleg)

m_handler:
    csrr x12, mcause            # 0x60: 期望 cause=9 (ecall from S)
    addi x13, x0, 9             # 0x64
    bne  x12, x13, halt         # 0x68: cause != 9 → halt 防御 (期望 fall through)

halt:
    csrw mtvec, x0              # 0x6c: mtvec=0
    ecall                       # 0x70: cause=11 from M → mmu fail → triple fault
