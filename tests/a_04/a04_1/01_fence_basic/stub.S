# a04_1 / 01_fence_basic
#
# ===== 运行参数 =====
# RUN-EXPECT-EXIT: 0
#
# ===== 测试目的 =====
# T1 fence / fence.i 实装 — 把"之前归 OP_UNSUPPORTED → illegal trap"这个 gap 扫掉。
# T1 之前: 0x0F opcode → decode default → OP_UNSUPPORTED → interpreter trap cause 2
# T1 之后: 0x0F + funct3=000 → OP_FENCE → fence_helper 空体 (NOP)
#          0x0F + funct3=001 → OP_FENCE_I → fence_i_helper → lrsc_clear_self
#                                (T1 空 stub; T3 真清 reservation)
# 验:
#   1. fence (含多种 pred/succ 组合 + fence.tso) 不再 illegal trap
#   2. fence.i 不再 illegal trap; 块边界后下一条正确 fetch+execute (= 块边界协议通)
#   3. 程序顺序推进到 test_dev FINISHER_PASS, main exit 0
#
# ===== 思路 =====
# 单 hart (默认无 --smp = 1)。boot prelude 清 MDT/SDT (跟 a_03 fixture 体例对齐)。
# 接连 4 条 fence (默认 / rw,rw / r,w / fence.tso) + 1 条 fence.i, 再写 test_dev
# PASS。任意一条触发 illegal trap → exit_code != 0 (fail handler 没装也会 abort);
# 全部走通即 PASS。
#
# fence pred/succ 多组合都退化为同一个空 helper (fence.h "双重 cover 论证":
# host x86 TSO + helper atomic 已兜底); 多写几条只为验"decode funct3=000 + 字段不解"
# 的容忍度, 不是验真 ordering 语义。

.equ TEST_DEV_BASE,     0x00100000
.equ FINISHER_PASS,     0x5555

.section .text
.option norvc
.globl _start

_start:
    # ---- Boot prelude: 清 MDT / SDT 让后续可正常运行 ----
    csrw    0x310, x0                   # mstatush 清 MDT
    csrw    sstatus, x0                 # 清 SDT (顺便清 SIE/SPIE)

    # ---- FENCE (RVI base "Memory Ordering"), 多种 pred/succ 组合 ----
    # 都走 0x0F + funct3=000 → OP_FENCE → fence_helper 空体. 字段 (fm/pred/succ)
    # 不解, 256 种组合都退化同一 helper.
    fence                               # 默认 pred=iorw, succ=iorw
    fence   rw, rw                      # 等价上面 (RW 只 2-bit, 不含 IO)
    fence   r, w                        # store-before-load 风格
    fence.tso                           # fm=1000 TSO 模式; gas 接受

    # ---- FENCE.I (Zifencei) ----
    # 0x0F + funct3=001 → OP_FENCE_I → fence_i_helper → lrsc_clear_self.
    # 块边界 (is_block_boundary_inst → 1): 走完 case → fetch loop 末 += PC_STEP_RV
    # (=4) → goto out → dispatcher 重派发, 下一条 (li t0,...) 从新块入口正常 fetch.
    # 验"块边界后路径通"= 下一条 sw 能 reach test_dev.
    fence.i

    # ---- PASS ----
    li      t0, TEST_DEV_BASE
    li      t1, FINISHER_PASS
    sw      t1, 0(t0)
1:  j       1b                          # 防 fall-through

# ===== 期望结果 =====
#   exit_code = 0 (FINISHER_PASS)
#   stderr 无 trap cause 2 (illegal instruction) 报错
#   main wall time 量级 < 50ms (5 条 fence + 几条 li/sw, 极轻量)
#   无 ASan / TSan 报错
# 不期望:
#   - illegal trap cause 2 (回归到 T1 之前的行为)
#   - 块边界后 fetch 失败 (fence.i 后下一条没执行 → 永远到不了 test_dev PASS)
#   - exit_code != 0
