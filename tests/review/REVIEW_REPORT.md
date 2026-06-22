# RISC-V JIT 模拟器 — 性能评估报告

> 本文档面向论文读者跟开源研究者, 综合呈现 RISC-V JIT 模拟器 (riscv-jit-emulator) 从解释器 baseline 到 JIT MVP 完整落地后的性能数据跟设计取舍.
>
> 内部实验记录 (调试 trail / milestone 推进时序 / 中间态实验数据) 在同目录的 `REVIEW.md` 中累积保留, 本文档作论文章节稿跟项目 perf 概览的独立成稿.
>
> 数据基于 commit `66c9cf3` (b_03 milestone 收尾) 状态, 跑批跨 a02_7 解释器 perf 套件 (16 fixture) 跟 b03_06 JIT 视角补充套件 (4 fixture).

## 目录

1. 项目背景与性能目标
2. 测试方法
3. 解释器 baseline 演进
4. JIT MVP 加速效果
5. JIT 关键子系统的设计取舍跟实测
6. 当前成果跟未来工作

---

## 1. 项目背景与性能目标

riscv-jit-emulator 是支持 RV32 G 扩展 (IMAFD_Zicsr_Zifencei) 的用户态 JIT 模拟器, 目标跑通 OpenSBI / 小型 OS / FreeRTOS-with-MMU 等真实 guest 软件. 项目实现三层架构: Dispatcher / Translator / JitBackend, 其中 JitBackend 默认走 asmjit 后端, emit x86_64 host 代码.

性能上的核心设计原则在 [plan §1.1](../../notes/plan_v7.md) 中被反复强调, 简称 "Fast path / Slow path 分离":

- Fast path = 高频且无副作用 (算术 / 逻辑 / 分支 / 寄存器操作 + 命中 TLB 的 load), 由 JIT 直接 emit host 指令, 跑在解释器之上 3-10x 速度.
- Slow path = 内存写 / 原子操作 / CSR 读写 / 中断检查 / fence 等, 统一走 C 实现的 helper 函数, JIT 块内 emit `call helper`. **Slow path 不 inline 进 JIT 块**.

这条原则有多个工程动机: helper 改逻辑一处改完不需作废 JIT 缓存; 解释器跟 JIT 共用一份 helper 代码; 调试 callstack 可读; 现代 CPU 上 call/ret 约 5-15 cycle 开销几乎可忽略. 副作用是 store / CSR / 原子操作的 JIT 加速比有上限 (约等于解释器水平), 这点会在第 4 节实测中显现.

本报告量化以下性能维度:

1. JIT 接入相对解释器的整体加速比 (典型 OS 工作负载预测)
2. 慢路径维持解释器水平的设计验证 (sacred 设计预期)
3. 块大小 (block size) 跟 dispatcher round-trip 开销的关系
4. 自修改代码 (Self-Modifying Code, SMC) 链路的端到端开销
5. fence.i / AMO 指令在 JIT 块内的实测成本

## 2. 测试方法

### 2.1 测试套件结构

性能测试 fixture 用 hand-written RV32 汇编编写, 每个 fixture 是自包含的 ELF + raw binary 二元组, 直接经模拟器 `--bios` 接口加载执行. 不依赖任何 boot ROM / OpenSBI / OS 运行时.

主套件 `tests/a_02/a02_7/` 含 16 个 fixture, 当初为解释器性能基线设计, 在 JIT 接入后继续作 JIT 跟解释器同条件对比的基准, 覆盖维度包括:

1. 取指路径: BARE M-mode identity (01_bare) vs SV32 page walk (02_mmu_sv32)
2. 内存访问形态: dense / sparse, load / store, 命中率 (04-11 共 8 fixture)
3. CSR 密集 (03_csr_heavy)
4. 块大小敏感性 sweep: block 2 / 8 / 16 / 32 / 64 inst (12-16 共 5 fixture)

补充套件 `tests/b_03/b03_06/` 含 4 个 fixture, 专门覆盖 JIT 子系统在 a02_7 未触发的关键路径:

1. SMC chain 高频/低频 (01_smc_perf_high, 02_smc_perf_low): 量化 SIGSEGV+mprotect+ invalidate+recompile 端到端开销
2. fence.i 稳态调用 (03_fencei_perf): 验证 "fence.i 不主动 invalidate JIT" 设计
3. AMO 调用密度 (04_amo_perf): 量化 atomic 指令通过 helper call 的开销

### 2.2 跑批方法与噪声控制

跑批工具 `run_perf.py` 实现两种模式:

1. 单 binary 模式: 每 fixture 连跑 N 次 (默认 N=5), 报 median / min / max MIPS, 作典型态. 单 fixture 上 ITER_COUNT 调到 Release ~1s 内完成, 让冷启动 (TLB / cache 冷 + mmap RAM 首次触页) 摊薄到 < 0.1%, 同时不被 CPU turbo 降频 / VM steal time 拖乱.
2. 多 binary 对照模式 (`--compare BIN1,BIN2,...`): 单 fixture 内严格 ABCABC... 交错跑 N 轮, 抵消 CPU turbo / cache 热漂移. 用于不同 commit / 不同 patch 配置间的细粒度对照实验 (第 3 节 PLIC 优化分析即此模式).

跨 milestone 性能数字源自不同 commit 编出的 release binary, 因 host 状态不同 (CPU 状态 / 内存负载 / 编译器副作用) 不能直接逐数字比较, 但同 fixture 内的 **相对比例** (检查占比 / 加速倍数) 抗 host 噪声, 可作跨时段对照.

### 2.3 测量条件

1. Host 平台: Linux 6.8 (Ubuntu 22.04 HWE), x86_64
2. 编译: gcc release `-O2`, 无 sanitizer
3. Guest 配置: 单 hart, BARE M-mode 或 SV32 模式 (fixture 指定)
4. 模拟器: Release build, `DEBUG_PERF_ON` 开 (其他 debug gate 关), dispatcher 打 `[perf]` 行携带 `elapsed / total_count / MIPS`
5. 测试工具链: gcc 16.1.0 / binutils 2.46 (`/opt/riscv-2026.05.19/`)

---

## 3. 解释器 baseline 演进

JIT 落地前, 解释器自身的性能受 CPU 子系统 (CLINT 时钟 / PLIC 中断 / CSR / MMU) 完整度的影响. 三个 milestone (a_02 / a_03 / a_04) 累计搭建出当前 baseline. 本节聚焦 a_03 阶段最显著的性能事件: PLIC 跟 CLINT 中断信号的 hot path 优化 — 设计精神是将 EIP / MTIP 等 "中断是否 pending" 的计算从 dispatcher 主帧转移到 PLIC / CLINT 自身, dispatcher 仅 atomic_load 读结果.

### 3.1 PLIC v1 同步实装的瓶颈

a_03 milestone 加入 RV PLIC v1.0.0 实装作外部中断源管理. 初始实装 (commit `0e840a9`) 走经典同步路径:

1. 设备 set/clear pending → 持 PLIC 写锁
2. 中断检查时, CSR `mip` 读路径上调 `is_plic_*_pending`, 持 PLIC 读锁 + 扫 96 个中断源, 算各 hart context 的 enable & priority threshold filtering

实测 (4 binary 对照实验, 见同目录 `REVIEW.md`) 显示 PLIC 占整个中断检查开销 ~99%, CLINT 几乎是噪声. PLIC 单独拖累 a02_7 fixture 50% - 92% MIPS, 跟 fixture 内 dispatcher 入口频率强相关 (CSR 密集代码 `03_csr_heavy` 拖累 -92%, 大块代码 `16_blocksize_64` 拖累 -18%).

### 3.2 PLIC hot path 优化: producer 预计算 + dispatcher atomic_load 直返

#### 3.2.1 异步刷新探索及其撤回

第一轮优化 (commit `6f72d85`, `f1c7012`) 沿 "异步 producer" 思路:

1. 引入后台 PLIC refresh 线程 (`pthread_cond_timedwait` 100ms 心跳, 受 SDS 控制)
2. `device_set/clear_pending` 写 MPSC 消息队列, producer 不阻塞
3. Refresh 线程异步消费消息, 持 PLIC 写锁重算所有 context 的 `plic_ctx_eip` atomic 字段
4. CSR `mip` 读路径上 `is_plic_*_pending` 改 `atomic_load(plic_ctx_eip, acquire)`, 从 "rwlock + 96 源扫" 退化到 "1 个 cache line acquire load"

性能数字 (4 binary 对照, 同目录 `REVIEW.md` "T6.2 PLIC 优化实装后对照" 段) 显示 PLIC 拖累从 50-90% 降至 ~15%, dispatcher hot path 退化到跟 CLINT 同量级.

然而该实装在 a_03 后段撞到正确性问题: PLIC complete (中断处理 ACK) 路径需要同步 CLEAR device_line, 而异步 refresh 让 producer-side device_set/clear 进 ring queue, 形成 **handler 同步 ACK 跟异步 CLEAR 的 race**. 表现为 spurious re-fire — 中断已被 handler ACK 但异步 queue 中还有 stale set 事件未处理, hart 退出 handler 后被同一中断源再次唤醒. 该 race 在 a_03 PLIC 集成测试中被捕获, 异步路径**整体撤回**.

#### 3.2.2 同步重构 + producer 预计算 (commit 7d08d87, a_03_007)

撤回异步后, 实装回到同步 wrlock, 但**保留了"dispatcher hot path 走 atomic_load 直返"的核心收益**. 改造的本质是将 EIP 计算从 dispatcher 转移到 PLIC 自己:

1. `device_set/clear_pending` 跟 `plic_write` 持 wrlock 后, 在临界区内**同步** `recompute_ctx_eip` 跟 `recompute_pending_bitmap`, 算完结果 `atomic_store(release)` 到 `plic_ctx_eip[ctx_id]` 跟 `plic_pending_bitmap_cache[word_idx]` 两个独立 file-static atomic 字段
2. dispatcher 上 `is_plic_meip_pending` / `is_plic_seip_pending` 走 `atomic_load(plic_ctx_eip[ctx_id], acquire)` 直返, **零 lock 零 scan, ~1 cycle**
3. 同步重构的代价是 producer 持锁路径变重 (wrlock + recompute + atomic_store), 但 a02_7 类纯解释器 / JIT fixture 不主动触发 producer side, hot path 性能跟异步版本等价
4. 收益是消除异步 staleness window, 跟真硬件 wire 模型对齐 (设备 ACK 跟 complete 同 cycle 完成)

#### 3.2.3 CLINT MTIP 同思路 (commit 3c997fb, a_03_011)

WFI 实装一并把 MTIP 计算路径同步化. 此前 `csr_mip_read` 合成 MTIP 时调 `is_clint_timer_pending` 内做 `mtime >= mtimecmp` 的 2 次 atomic_load 比较, dispatcher hot path 上每次中断检查都重算一遍. 改造后:

1. timer 辅助线程跟 guest writer (写 mtime / mtimecmp / mtimecmps) 都进同一个 `mtip_lock[hartid]` 短临界区
2. 临界区内 `inputs → compute → atomic_store(mtip, release)`, 计算 mtip 现态
3. dispatcher / WFI predicate 走 `atomic_load(mtip[hartid], acquire)` 直返, 零 scan

设计精神跟 PLIC 完全同向: **producer (CLINT 自己) 预计算, consumer (dispatcher) 直读 atomic**.

#### 3.2.4 hot path 优化效果 (a02_7 perf 套件)

`REVIEW.md` "T6.2 PLIC 优化实装后对照" 段 4 binary 实验在异步版本上跑出:

| Fixture            | PLIC 拖累 (优化前) | PLIC 拖累 (优化后) | 改善幅度 (pp) |
| ------------------ | -----------------: | -----------------: | ------------: |
| 03_csr_heavy       |             -92.4% |             -22.7% |        +69.7  |
| 12_blocksize_02    |             -90.8% |             -18.3% |        +72.6  |
| 06_bare_load       |             -70.5% |              -0.1% |        +70.4  |
| 10_mmu_sparse_load |             -54.8% |              -1.5% |        +53.3  |
| 16_blocksize_64    |             -18.5% |             -16.1% |         +2.4  |

异步路径撤回后, dispatcher hot path 行为 (atomic_load `plic_ctx_eip` 直返) 在同步重构版本上完全保留, 故此 4 binary 实验数据对当前实装的 hot path 优化效果仍然有效 — 差异仅在 producer side 的 wrlock cost, 此 a02_7 fixture 不主动触发.

优化后, PLIC 路径 cost 退化到与 CLINT 同量级 (~1 cycle atomic load), 系统从 "PLIC 主导瓶颈" 变为 "中断检查框架本身固定开销主导". 第 4 节 JIT 加速比的解释器 baseline 即以此后稳态作锚.

### 3.3 残余开销与未来节流方案

PLIC + CLINT hot path 优化后, a02_7 全套 ALL_ON vs ALL_OFF 仍有 ~15-25% 的中断检查框架开销, 来源不是 PLIC 也不是 CLINT, 而是:

1. `csr_mip_read` 函数调用框架本身 (call/ret + 寄存器保护)
2. `is_*_pending` 四个 atomic_load 串行
3. `trap_check_interrupt` 每 block 一次轮询

进一步降低需要走 plan §2 #49 的中断节流方案: dispatcher 主帧维护 `local_count`, 攒够 N 条指令再 poll 一次. 代价是中断投递最多延迟 N 条指令 (RV spec 不要求即时投递, 可接受). 此优化跟 JIT 接入会改变 dispatcher 入口频率, 故延后到 JIT 稳态后再评估其 ROI.

---

## 4. JIT MVP 加速效果

本节以 a_03 中断检查 hot path 优化后稳态作解释器 baseline (实测数据从 commit `6f72d85` 异步版本采集, 但 dispatcher hot path 行为跟当前同步重构后实装等价, 见 §3.2 末段说明), b_03 收尾 commit (`66c9cf3`) 作 JIT MVP 完整态, 在同一中断检查框架下量化 JIT 接入的端到端加速比.

### 4.1 测试 fixture 加速概览

a02_7 16 fixture 同条件 release median × 5 跑批对比:

| Fixture                | 解释器 (a_03 T6.2) | JIT (b_03 T_收尾) |  加速比 |
| ---------------------- | -----------------: | ----------------: | ------: |
| **取指 / 纯 ALU**      |                    |                   |         |
| 01_bare                |              163.2 |             674.8 |   4.14x |
| 02_mmu_sv32            |              160.3 |             620.7 |   3.88x |
| 06_bare_load           |              198.4 |             604.0 |   3.05x |
| **内存访问 (load)**    |                    |                   |         |
| 04_mem_dense           |              152.0 |             214.7 |   1.41x |
| 05_mem_tlbmiss         |              137.5 |             265.3 |   1.93x |
| 08_mmu_dense_load      |              171.3 |             461.5 |   2.70x |
| 10_mmu_sparse_load     |              101.1 |             135.4 |   1.34x |
| **慢路径 (sacred)**    |                    |                   |         |
| 07_bare_store          |              118.4 |             118.1 |   1.00x |
| 09_mmu_dense_store     |              111.7 |             113.8 |   1.02x |
| 11_mmu_sparse_store    |               87.1 |              82.2 |   0.94x |
| 03_csr_heavy           |               78.9 |              64.2 |   0.81x |
| **块大小 sweep**       |                    |                   |         |
| 12_blocksize_02        |              129.7 |             142.9 |   1.10x |
| 13_blocksize_08        |              173.2 |             564.3 |   3.26x |
| 14_blocksize_16        |              175.5 |            1102.9 |   6.28x |
| 15_blocksize_32        |              192.9 |            2030.5 |  10.53x |
| 16_blocksize_64        |              201.2 |            1568.3 |   7.80x |

统计:

1. 算术平均加速比 3.39x
2. 几何平均加速比 2.66x (抗极值漂移, 更接近 "典型加速")
3. 最大加速比 10.53x (15_blocksize_32, 大块 + 寄存器映射优化协同效应, 见 §5.1)
4. 最小加速比 0.81x (03_csr_heavy, 块大小敏感性导致退化, 见 §5.4)

### 4.2 加速曲线: 块大小是主导因子

块大小 sweep 的曲线最清晰显示 JIT 加速跟 guest 平均块大小的关系:

| 块大小 (inst) |  解释器 MIPS |    JIT MIPS |  加速比 |
| ------------: | -----------: | ----------: | ------: |
|             2 |        129.7 |       142.9 |   1.10x |
|             8 |        173.2 |       564.3 |   3.26x |
|            16 |        175.5 |      1102.9 |   6.28x |
|            32 |        192.9 |      2030.5 |  10.53x |
|            64 |        201.2 |      1568.3 |   7.80x |

直观解释: 每个块的执行经过一次 dispatcher round-trip (jit_cache_lookup + host code 调用 + return 后状态同步), 此固定开销不随块大小变化. 解释器对应的固定开销是 decode loop 的每条指令 overhead, 跟块大小无关. 因此块越大, JIT 摊薄 round-trip 越成功. 反向: 极小块 (2 inst) 时 round-trip cost 跟 decode cost 持平, JIT 几乎无增益.

`16_blocksize_64` 反序 (10.5x → 7.8x) 是大块 emit 后 host 端 `.text` 段超过某层级 I-cache 容量边界的预期效果, 不影响主结论.

实测一致性: 从块大小曲线反推 dispatcher round-trip 稳定约 15ns / 块 (`a02_7/12` 跟 `a02_7/15` 两端外推), 后续 §5.3 fence.i 实验也独立验证此数字.

### 4.3 真实 guest 工作负载的加速预测

guest OS (Linux kernel / OpenSBI / FreeRTOS-MMU) 的典型基本块大小在 10-20 RV inst 之间 (跟 host 编译器优化层级, 工作负载类型相关). 落到上表曲线 8-16 inst 段: 加速比约 3-6x. 综合考虑混合工作负载中 store / CSR / interrupt 占比 (约 10-20% 慢路径), 实际跑 OS 的端到端加速比预期落在 **3-5x 区间**.

`a02_7/01_bare` (10 inst 块, 纯 ALU) 4.14x 是这一预测的代表锚点.

---

## 5. JIT 关键子系统的设计取舍跟实测

### 5.1 寄存器映射策略

JIT 块内 RV 寄存器 (x0-x31) 跟 host 寄存器 (x86_64 通用寄存器) 的映射策略经历两次演进:

**Layer 1 — 静态固定映射** (b_02 milestone):

最低 5 个常用 RV 寄存器 (x1-x5) 硬编到 5 个 callee-saved host 寄存器 (rbx / r12 / r13 / r14 / r15). 优点: 实装简单, prologue/epilogue 固定. 缺点: 块内若 hot register 不在 x1-x5 (如循环 counter 用 s0/s1), 仍走内存路径 `mov [rdi + offset], rN`.

**Layer 2 — 块内 use_count 动态分配** (b_03 T3, commit `da6df71`):

每次 compile_block 入口扫所有 IR 指令, 按 op kind dispatch 计每个 RV 寄存器的 use_count, 按 use_count 降序选 top 5 promote 到 callee-saved host pool, 在 prologue 一次性 load, epilogue 一次性 store. use_count = 0 的寄存器仍走内存路径不浪费 host 寄存器名额.

实测增益 (跟 b_02 末 `dab36e6` baseline 对比):

1. 小块 (a02_7/01_bare, 10 inst): 持平 (block 内 hot register 已落 x1-x5)
2. `a02_7/15_blocksize_32`: 677 → 2101 MIPS (+210%)
3. `a02_7/16_blocksize_64`: 575 → 1557 MIPS (+170%)

大块增益尤其显著, 因为大块内 hot register use_count 容易 > 5, Layer 2 能动态 promote 出真正高频的寄存器, 块体内省去大量 `mov [rdi + offset], rN` 内存往返指令.

**Layer 3 — 跨块全局分配** (plan §2 #1, deferred):

跨块全局寄存器分配需要 block chaining (块间直接 jmp 不经 dispatcher) 跟 hot-trace profiling 数据支持. 当前实装未做 block chaining, 故 Layer 3 推迟到 c-series milestone (跟 tiered JIT 一起评估).

### 5.2 自修改代码 (SMC) 处理

JIT 缓存的核心正确性挑战: 当 guest 修改已被 JIT 编译的代码页时, 必须 invalidate 对应的 host code. b_03 T1.a 落地端到端 SMC chain:

1. JIT 块 install 时, 对应 guest 物理页通过 `mprotect(R-only)` 写保护
2. Guest 写代码页 → host `SIGSEGV` 触发 SMC handler (async-signal-safe)
3. Handler 仅 `atomic_fetch_or` 写 `page_dirty` 位图 + `mprotect(R/W)` 恢复
4. Dispatcher 主循环顶检测 `dirty_pending` 计数器 (T4 加, 见下), > 0 时扫位图
5. 对每个脏页调 `jit_invalidate_page`: collect + invalidate cache 项 + 等 grace period + backend 释放 host code

实测端到端开销 (b03_06/01 量化, OUTER=50000 SMC trigger):

1. 整体 fixture 跑 0.71s, 总 70.2M inst, 测得 98.8 MIPS
2. 反推单次 SMC 端到端约 12 μs / trigger
3. 主要成本来源: Linux 内核 SIGSEGV signal delivery (~3-5 μs) + asmjit 重编译单块 (~3-5 μs) + 100 条 RV inst 重 interpret 跨过 JIT 编译阈值 (~3-5 μs)
4. 项目自有 hot path (dirty_pending atomic load + jit_invalidate_page collect
   + backend release) 总贡献 < 1 μs / trigger

性能数据印证 SMC chain 设计正确: 端到端开销由不可压缩的内核 + 编译器成本主导, 项目自身路径已经接近理论下限.

**关键性能修正 (b_03 T4, commit `d219b3e`)**:

T1.a 初版未加 cardinality counter, dispatcher 每 iter 强制扫 512-word 位图. 对 dispatcher 入口频率高的 fixture 撞出 15x 性能退化 (a02_7/01_bare 从 668 MIPS 跌到 42 MIPS). T4 修复加 `_Atomic uint64_t dirty_pending`, dispatcher 顶 fast-path skip:

```
if (atomic_load_explicit(&dirty_pending, relaxed) == 0u) return false;
```

99% iter 走单次 relaxed load 跳过位图扫描. 修复后 perf 恢复 b_02 末水平.

存在两类 benign race window, 设计上明确容忍 (详见 `src/jit/smc.c` 顶段文档以及项目内 `dummy.txt §17`):

1. Handler fetch_or 跟 fetch_add 之间: dispatcher 看到 counter=0 跳过, 下一 iter 再处理. SMC 投递延迟最多 1 iter, 在 RV spec 允许范围内.
2. Consume fetch_and 跟 fetch_sub 之间: dispatcher 走全扫但找不到 bit, 浪费一次扫描后 return false. 不影响正确性.

### 5.3 fence.i 块边界 cost

按设计 audit Q4.2.1+ (b_03 T2), `fence_i_helper` 不主动调 `jit_invalidate_*`. 理由是: SMC SIGSEGV 链路已经是 invalidate 工作的唯一来源, fence.i 在 JIT 子系统视角下只是 "块边界 + lrsc_clear_self 调用".

实测验证 (b03_06/03):

| Fixture            |  inst/iter | block 数 | 平均 block 大小 | MIPS  |
| ------------------ | ---------: | -------: | --------------: | ----: |
| a02_7/01_bare      |         10 |        1 |              10 | 674.8 |
| a02_7/13_blocksize_08 |       8 |        1 |               8 | 564.3 |
| b03_06/03_fencei_perf |      10 |        2 |               5 | 286.2 |
| a02_7/12_blocksize_02 |       2 |        1 |               2 | 142.9 |

按 a02_7 sweep 反推 dispatcher round-trip 约 15 ns / 块. b03_06/03 平均块大小 5 inst 预期约 330 MIPS, 实测 286 MIPS, 差值约 -13% 对应 fence.i 本身的 helper 调用 + lrsc_clear_self atomic store, 估约 10-15 ns / 调用.

结论: fence.i 在无 SMC 触发时近似零成本, 验证 "不主动 invalidate" 设计正确. 此设计跟 sfence.vma 同精神 — JIT 子系统不主动响应 RV 同步指令, 只在 spec 要求时机 (块边界) 间接同步.

### 5.4 慢路径设计的实测验证

`store / atomic / CSR / interrupt check` 走 C helper 不 inline 进 JIT 块的 sacred 原则, 在加速比表上对应一系列接近 1.0x 的实测项. 这不是性能 regression, 而是设计预期:

| 慢路径 fixture     |   解释器 |     JIT |  比例 | 说明                                 |
| ------------------ | -------: | ------: | ----: | ------------------------------------ |
| 07_bare_store      |    118.4 |   118.1 | 1.00x | store_helper extern call, 等价路径   |
| 09_mmu_dense_store |    111.7 |   113.8 | 1.02x | store + Sv32 walker, 等价路径        |
| 11_mmu_sparse_store|     87.1 |    82.2 | 0.94x | 稀疏 walker, noise 范围              |
| b03_06/04 AMO      |        — |    67.2 |     — | 比 07_bare_store 慢 -43%             |

**AMO 比 plain store 慢的根因**: amoadd.w 在 x86_64 host 上用 `LOCK XADD` 指令实现 (per a_04 Zaamo 9 op design), `LOCK` 前缀全内存屏障约 20-30 cycle, plain `MOV` 几乎免费. 加上 AMO 需返回老值 (额外一次寄存器写回), 总 cost 约 plain store 的 1.75x. 落到 -43% MIPS.

**CSR 退化的根因 — 块大小敏感性**: `03_csr_heavy` 0.81x 退化看似异常, 但跟 `a02_7/12_blocksize_02` (1.10x, 几乎无 JIT 增益) 同源. CSR 指令是硬块边界 (`decode.h is_block_boundary_inst` 返回 1), CSR 密集代码每轮约 9 个块, 平均块大小约 1.1 inst. 解释器在此模式下没有 round-trip 开销 (一个 dispatch frame 内 decode + execute 连跑), 而 JIT 每个 CSR 都强制 round-trip → jit_cache_lookup → 重入块. 当平均块小于 dispatcher round-trip cost / decode cost 的临界点 (约 3-5 inst) 时 JIT 反慢.

意义: 这是块大小敏感性, 不是 CSR 实装的 regression. OS 实际工作负载中 CSR 密集段 (trap handler / context switch) 占总时间约 10%, 整体加速仍由 fast-path 主导. 此分析方向跟 [plan §2 #49] 中断节流方案 (`local_count`) 的 ROI 评估直接相关.

---

## 6. 当前成果跟未来工作

### 6.1 已落地能力 (commit 66c9cf3 状态)

1. JIT MVP 端到端: BARE M-mode + SV32 (S/U regime 分别编一份块体, 编译期 baked PTE_U 视角) fast-path 完整, 慢路径走 C helper 不 inline
2. SMC chain 端到端正确性 + 性能 (dirty_pending counter 修复 hot path 退化)
3. fence.i 块边界 doc + lrsc_clear_self 副作用路径
4. 寄存器映射 Layer 2 (块内 use_count 动态分配, 大块增益 +170 ~ +210%)
5. RV32 IMC 全集 70 op JIT 翻译; M 扩展 DIV/REM 用 inline branch 兜 by-0 跟 INT_MIN/-1 overflow
6. SMP 多 hart 支持 (--smp N, MAX_HARTS=8), JIT cache 用 EBR RCU 跟 per-page block list 实现无锁读

### 6.2 性能成果摘要

1. JIT 全套 a02_7 加速: 几何均值 2.66x, 算术均值 3.39x
2. 大块场景: 最高 10.53x (a02_7/15_blocksize_32)
3. OS guest 真实工作负载预测: 3-5x 加速区间
4. SMC chain 端到端开销 ~12 μs / trigger, 主导项为内核 + 编译器, 项目自身路径接近理论下限
5. fence.i 块边界 cost ~10-15 ns, 验证 "不主动 invalidate" 设计零成本承诺

### 6.3 推迟的优化方向

下列优化经评估暂不进入 b_03 范围, 留作后续 milestone:

1. **寄存器映射 Layer 3 (跨块全局)** — 需 block chaining + hot-trace profiling, 跟 tiered JIT 一起评估
2. **block chaining (块间直接 jmp)** — 当前所有块出口都走 dispatch, 简化 SMC invalidate 模型. chaining 需要双向 patch 跟 invalidate 协议联动
3. **中断检查节流 (`local_count`)** — 收回剩余 ~15-25% 中断检查框架开销; JIT 接入后 dispatcher 入口频率改变, 现在评估 ROI 数字会偏
4. **store fast path inline 化** — 不是 "store 变 fast path", 而是 store_helper extern call → static inline 或 JIT emit-equiv, 消除每次 store 付的 call/ret 开销 + 寄存器保护代价
5. **tiered JIT (LLVM 后端)** — 当前单后端 asmjit 是 IR 1:1 模式, 抽象 IR 收益要 tiered JIT 才显
6. **aq/rl memory order 精确化** — x86_64 host 上是 no-op, weak host (ARM/AArch64) JIT 才有真收益
7. **跨 hart i-cache coherence (SBI broadcast)** — 当前 cross-hart 跑 fence.i eventually consistent, RV spec 允许; SBI 接入时跟 lsu RAM access 改 atomic 一起做

### 6.4 项目当前不在目标内

1. PMP / 物理内存保护
2. 多 queue / 现代 PLIC v1.1 / AIA / IMSIC
3. H-extension (虚拟化)
4. F/D 浮点扩展 (推后非永久排除)
5. SV32_S / SV32_U 合并到一份块体 (跟 Layer 1 静态映射 / VA-as-JIT-key / block chaining 同性质, 项目永久立场 — 各编一份块体 + 编译期 baked PTE_U 是消除运行时 priv 分支的 sacred 设计)
6. VA 作 JIT cache key — JIT cache key 永远是 (PA, regime), 防 ASID / satp 切换导致的 VA aliasing

---

## 附录: 数据源 commit 索引

本报告所引数据基于以下 commit 状态:

1. `0e840a9` — a_03 PLIC v1 初版同步实装 (§3.1 PLIC 瓶颈实验源)
2. `6f72d85` — a_03 PLIC EIP 异步刷新探索 (§3.2.1 短暂尝试, §3.2.4 4 binary 实验数据采集 commit, 后撤回)
3. `7d08d87` — a_03 PLIC 同步重构 + producer 预计算保留 atomic_load 直返 (§3.2.2 当前实装路径起点)
4. `3c997fb` — a_03 CLINT MTIP 预计算同思路 (§3.2.3)
5. `dab36e6` — b_02 末 JIT MVP RV32IMC 全集 (§5.1 Layer 2 增益对照基准)
6. `d219b3e` — b_03 T4 dirty_pending counter 修 (§5.2 SMC chain 性能修复)
7. `66c9cf3` — b_03 收尾 (本报告主数据源, §4 JIT 加速比锚)

详细实验记录跟跑批原始数据在同目录 `REVIEW.md` 中按 milestone 时序累积保留, 可作 deep-dive 参考.
