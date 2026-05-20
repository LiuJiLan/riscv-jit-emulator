# a02_6 / 03_perf_estimate
#
# ===== 测试目的 =====
# 跑一段长工作循环, 读 mtime 前后做 delta, 配合 [dispatcher] halted total_count
# 算 effective MIPS — debug 解释器性能基线 (release + JIT 接入后跑同 fixture
# 对比, 性能数据进论文)。
#
# ===== 思路 =====
# 仅用现有指令: lw / sw / add / addi / lui / bnez / sub / mv / csrw / ecall;
# 不依赖 PLIC / UART / S-mode / Sv32 / RVC (无 .option rvc)。
#
# 工作循环 100K 轮, 每轮 6 RV inst (add + add + sw + lw + addi + bnez):
#   - bnez 是块边界 → 每轮 = 1 个 block dispatch
#   - sw 走 slow path store_helper (LR/SC reservation + 未来 SMC 副作用入口)
#   - lw 走 fast path (BARE 模式 IS_GPA_RAM 直 *hva inline)
#   - 体现解释器混合 fast/slow path 平均性能
#
# 总 RV inst ≈ 100000 * 6 + 设置/halt 约 20 = 600020; debug build O0+ASan 预期
# 0.3-0.6 MIPS, 跑 ~1-2 s wall clock; mtime delta 10-20M tick (TIMEBASE=10MHz)。
#
# 性能估算公式 (跑完看 dump 心算):
#   wall_s     = a5 (mtime delta) / 10_000_000
#   total_inst = [dispatcher] halted total_count
#   MIPS       = total_inst / wall_s / 1_000_000
# 例: a5=12_345_678, total_count=600_020 → wall_s=1.235 → MIPS≈0.486
#
# 调 ITER_COUNT (s1 初值) 改长跑量: 100K → 1-2s; 10K → 100-200ms; 1M → 10-20s。

.section .text
.option norvc
.globl _start
_start:
    # ---- 起步: 取 mtime 起点 + 堆地址 + 累加器 + 循环计数 ----
    li      t6, 0x0200BFF8          # CLINT mtime addr (低 32 位)
    lw      a0, 0(t6)               # a0 = start_mtime_lo

    li      s2, 0x80010000          # heap base (远离 .text, 安全偏移; 64KB)
    li      s0, 0                   # s0 = accumulator
    li      s1, 100000              # s1 = ITER_COUNT (调这个值改长跑量)

    # ---- 工作循环 (每轮 6 inst; bnez = 块边界 → 1 block/iter) ----
work_loop:
    add     t0, s1, s1              # t0 = s1 * 2 (用 add 取代 slli, 兼容性最稳)
    add     s0, s0, t0              # acc += t0
    sw      s0, 0(s2)               # store acc 到 heap (slow path: store_helper)
    lw      t1, 0(s2)               # load 回 (fast path: BARE *hva 直读)
    addi    s1, s1, -1              # counter--
    bnez    s1, work_loop           # 块边界 + 循环回跳

    # ---- 跑完: 取 mtime 终点 + 算 delta + 留 sanity 值 ----
    lw      a1, 0(t6)               # a1 = end_mtime_lo
    sub     a5, a1, a0              # a5 = delta_mtime (单位 100 ns/tick)
    mv      a2, s0                  # a2 = final acc (跨运行应一致, 验正确性)
    mv      a3, t1                  # a3 = last loaded value (验 lsu 路径)

    # ---- halt: 清 mtvec + ecall 触发 triple fault ----
    csrw    mtvec, x0
    ecall

# ===== 期望结果 =====
# 期望最终寄存器 (粗略):
#   a0 (x10) = start_mtime_lo  (一般几千 ~ 几万, 取决于 host 启动到 _start 间隔)
#   a1 (x11) = end_mtime_lo    (> a0; 长跑后 mtime 跨过多个 timer wake)
#   a2 (x12) = sum 32-bit truncated (跨运行一致; ITER_COUNT=100000 时
#              = 2*(1+2+...+100000) mod 2^32 = 10_000_100_000 mod 2^32 = 0x540C1E60)
#   a3 (x13) = 0x540C1E60      (= a2; 最后一次 sw/lw 配对验证 lsu)
#   a5 (x15) = a1 - a0         (mtime delta, 单位 100 ns; 预期 10M-20M debug build)
#
# 期望 trap dump:
#   in_trap = 3                (halt path triple fault, 跟 a02_5 各 + a02_6/01-02 同)
#   total_count ≈ 600_020      (100K * 6 + setup/halt ~20; 看 [dispatcher] halted 行)
#
# 期望 elapsed (main 末段打印):
#   "X min Y.YYY s (total Z.ZZZ s)" — Z 应跟 a5/10_000_000 量级匹配
#   (差异 = decode_test + 初始化 + dump 等 main 段, ~ms 级别)
#
#
# ============================================================================
# 建立这个测试当下的实验结果记录
# (未来可能会过时, 但是属于试验记录, 不用清理; 数值未来可能不同)
# ============================================================================
#
# 试验日期: AEST 2026-05-19 16:45   环境: debug build (-O0 + ASan + UBSan), 解释器, 单 hart
#
#   [dispatcher] halted: in_trap=3 total_count=600012 ...
#   [clint timer] stopped (SDS=0): mtime=3750000
#   [main] elapsed: 0 min 0.376 s (total 0.376 s)
#
# elapsed 口径: main 入口 t_start (decode_test 之前) → CLINT timer 线程看到
# SDS=0 退出 + clint_join_timer 返回之后 t_end (dump 之后 / 三 destroy 之前)。
#
# elapsed 内含 (按时序; "运行外的调试" 也都在内):
#   1. decode_test (52 case 纯 C 单测; 不进 dispatcher)
#   2. ram_init / clint_init / cpu_create (POR 初始化)
#   3. clint_start_timer (pthread_create timer thread)
#   4. while 主循环: cpu_reset + clint_reset + dispatcher (本 fixture 真正工作)
#   5. while else 分支: atomic_store SDS=0 + break
#   6. clint_join_timer: pthread_join 等 timer thread 看到 SDS=0 退出
#      (最多再过一个 TIMER_WAKE_INTERVAL_NS = 1ms 才退)
#   7. dump 段 fprintf (reg dec / reg hex / trap / state)
#
# mtime vs elapsed 互校:
#   mtime=3750000 tick × 100ns/tick = 0.3750 s; 跟 elapsed 0.376 s 差 ~1ms,
#   = 阶段 (1)(2)(3) 在 timer thread spawn 之前 + 阶段 (6) 末段 join 等下一
#   wake + 阶段 (7) dump 在 timer 已 stop 之后 — 这些都在 elapsed 内但不在
#   mtime 累加窗口内。
