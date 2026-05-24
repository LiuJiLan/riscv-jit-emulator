# tests/ 整理记录 (REVIEW.md)

每次 milestone 收尾做一次测试整理 pass,在此文件 append 一节。

记录重点是**演进 trail**(为什么老 fixture 的期望会漂)和**待校准清单**,**不是**
point-in-time 的全量"符合 / 不符"表 —— 那张表是当次产物,下个 milestone 即过时,
不进本文件。

本目录(`tests/review/`)的约定:
- `run_tests.py` / `reorg_spec.md` / `REVIEW.md` —— 跟踪进 repo(工具 + 规范 + 记录)。
- `out/` —— 每次跑批的一次性产物(冻结二进制副本 / `tests_res/` / `report.md`),
  gitignore,不进 repo。

方法(每次大致相同):
1. 从 `cmake-build-debug/` 复制一份当时构建好的二进制到 `tests/review/out/`
   (冻结,与并行的 src 改动隔离)。
2. `tests/` 下 `make` 一次,再 `python3 tests/review/run_tests.py` 跑批全部 fixture
   (带 timeout),输出落 `out/tests_res/`(镜像 tests/ 结构)。
3. 逐测试比对各 `stub.S` 底部「期望结果」段 与 实跑输出。
4. 重排 stub.S 注释为三段式(见 `reorg_spec.md`)。
5. 重建 fixture,用 `out.bin` md5 校验注释改动对代码区零影响
   (`out.elf` 构建非确定性,不用作校验基准)。
6. 按"一阶段一事":只整理注释 + 出报告,不改 fixture 的期望值 / 代码;
   过时项留作 trail,登入下方"待校准清单",后续统一校准。

---

## a_02 收尾 — 2026-05-20

范围:72 个 fixture 跑批;69 个 `stub.S` 注释重排为三段式;未改任何 fixture 的
期望值或代码(只动注释,重建后 67/67 个 `out.bin` md5 逐字节不变)。

汇总:符合 51 / 部分演进 12 / 不符 2 / 无显式期望 4 / 无 stub.S 3 = 72。

### 演进 trail(durable — 解释期望漂移的根因)

- **(A) total_count 计数语义变更** —— trap 触发块现在会计入 trap 前已执行的指令数;
  老 fixture 写期望时按"trap 触发块贡献 0"算。受影响测试的其余字段(寄存器 / trap)
  全部仍符合,只是 total_count 期望值偏小。
- **(B) csr 从 stub 变完整实装** —— csr 有物理存储后,`csrr` 不再恒返 0;老的
  csr-stub 阶段 fixture 的寄存器期望(全 0)随之过时。
- **(C) mtime 改由异步 timer 辅助线程驱动** —— mtime 不再随指令同步推进,而是 timer
  线程 ~1ms wake 一次累加;短自旋的 fixture 来不及让 mtime 越过 mtimecmp。
- **(D) 个别 stub 注释里的期望值不准** —— 算错 / 自标 "(?)" / 自标"看反汇编"待核对;
  属注释精度问题,非实装回归。

### 待校准 fixture 清单(下次校准照清,共 14 项)

因 (A) total_count(仅 total_count 偏,其余全符):
- a01_5/02_trap_arch_basic(期望 3,实跑 6)
- a01_5/04_trap_chain(期望 10,实跑 11)
- a01_6/02_load_misalign_reject(期望 7,实跑 9)
- a01_6/03_store_oor_reject(期望 7,实跑 9)
- a01_7/04_u_mode_ecall_chain(期望 19,实跑 21)
- a01_10/03_rvc_arith(期望 28,实跑 27)

因 (B) csr 实装:
- a01_5/01_csr_basic —— 寄存器期望仍按 csr-stub 阶段写(全 0),实测 x6=0x40、
  pc=0x40。fixture 自带"注:"已预告,挂 T6.2(fixture 注释统一)。

因 (C) mtime 异步化:
- a02_5/03_int_mie_mask_off —— 期望 a0=0x80,实跑 0;busy_loop 仅 100 圈,需拉长
  或改成等待式,才能让异步 timer 把 mtime 推过 mtimecmp。

因 (D) 注释期望值待核对:
- a02_6/03_perf_estimate —— a2/a3 期望写 0x540C1E60,正确值 0x540D6AA0
  (10_000_100_000 mod 2^32 算错)。
- a01_8/02_sv32_remote_fetch、a01_8/03_msu_priv_chain —— mstatus 期望值 stub 自带
  "(?)";实跑分别为 0x1800、0x1820。
- a01_8/09_sv32_root_pt_not_in_ram —— x13/x14 期望 0x80000040,stub 自标"看反汇编";
  实跑 0x8000003c。
- a01_9/03_medeleg_deliver_s —— mepc 实跑 0x80000058,与注释"deliver M 路径从未
  走过 → mepc=0"的叙事不一致。
- a02_3/01_u_priv_reject —— s0 期望 3、实跑 2;s0 实际数的是"走到 epilogue 的次数"
  (case2 提前 halt 不计),非"进 handler 次数"。

### 备注

- 三个无 stub.S 的 loader reject 测试(a01_1/05、06、07)本次未整理。
- 当次全量比对表在 `out/report.md`(gitignore,不入 repo)。
- 本次整理见 session log `a_02_session_012.md`。

---

## 中断检查开销实验(a02_7 perf 套件)— 2026-05-23

不是测试整理 pass,是一次 **perf 对照实验**(session `a_02_session_016.md`)。

把 dispatcher 主循环顶的 `if (trap_check_interrupt(hart) != 0) continue;` 临时
注释掉,跑 a02_7 全套,对照"中断检查 ON / OFF"的吞吐 —— 量化"每 block 一次
中断检查"的开销。

a02_7 **不依赖 CLINT**(11 个 fixture 的 guest 代码都不碰 mtime/mtimecmp/msip,
也从不开中断),注释掉检查后行为完全不变(照常 triple-fault 收尾),只是省了每
block 那次 poll。故此实验对 a02_7 安全。

方法:编两个 release 二进制(ON / OFF),每 fixture off/on **交错** 6 对取 median
(交错抵消热漂移,小 delta fixture 才不被两趟之间的漂移淹没)。环境同 a02_7 实验
记录(release `-O2`,解释器,单 hart)。

### 结果(median MIPS)

```
  fixture              ON(CLINT)   OFF     检查占 ON 运行
  01_bare                180.8    200.7      9.9%
  02_mmu_sv32            177.4    199.5     11.1%
  03_csr_heavy            84.9    108.7     21.9%   <- 最重
  04_mem_dense           161.4    178.7      9.7%
  05_mem_tlbmiss         148.2    166.6     11.0%
  06_bare_load           202.2    221.5      8.7%
  07_bare_store          124.9    130.0      3.9%
  08_mmu_dense_load      175.9    191.3      8.1%
  09_mmu_dense_store     121.2    124.0      2.2%
  10_mmu_sparse_load     101.4    106.0      4.4%
  11_mmu_sparse_store     89.4     90.4      1.1%
```

### 结论

- 中断检查是 **每 block 一次** 的固定开销(dispatcher 主循环顶,每次重派发都查)。
  当前 CLINT-only 状态下,它占 dispatcher 运行时间的 **1% ~ 22%**。
- **03_csr_heavy 最重(22%)**:csrr 是硬块边界(`decode.h is_block_boundary_inst`
  对 6 个 `OP_CSR*` 全 return 1),csr 密集代码每轮 9 个 block → 中断检查的 dispatch
  频率是别的 fixture(每轮 1 block)的约 9 倍。OS 内核的 trap handler / 上下文切换
  csr 密集,这个数值有现实意义。
- 其余 fixture(每轮 1 block)占比 1%~12%,规律 = **"block 越便宜,固定检查占比
  越大"**:便宜 block(纯算术 / load fast path)9~12%;贵 block(store slow path /
  稀疏 walk)1~4%(慢 block 把固定检查成本稀释了)。跟 load/store fast/slow 同一
  个道理 —— fast path 对固定叠加开销最敏感。
- 每 block 检查的绝对成本粗估 ~3-6 ns(大 delta fixture 较准;07/09/11 delta 小、
  噪声占比大,不细抠)。

### block 大小 sweep(12-16,bare 纯算术)

把上面「block 越小,固定检查占比越大」直接画成曲线。加 5 个 bare 纯算术 fixture
(`12`-`16`):循环体 = N 条 `add`(`.rept` 生成)+ addi + bnez,block 固定为
2 / 8 / 16 / 32 / 64 指令。纯算术、不碰 load/store/csr,每指令成本恒定,唯一变量
是 block 大小。同样交错 A/B(off/on 6 对取 median):

```
  block 大小   ON       OFF      检查占 ON 运行
     2        129.2    162.3       20.4%
     8        197.2    218.9        9.9%
    16        183.9    197.5        6.9%
    32        219.1    227.3        3.6%
    64        226.5    234.1        3.2%
```

- 检查占比随 block 增大单调下降:block 2 的 20.4% → block 64 的 3.2%。中断检查是
  每 block 一个固定开销,block 越大、摊到每条指令越少。
- block 2 的 20.4% 跟 `03_csr_heavy` 的 21.9% 基本一致 —— 03 每轮 9 个 block / 10
  条指令,平均 block ≈ 1.1,比 block 2 还小,落在曲线这一端。两个独立 fixture 互证。
- block 8 的 9.9% 跟 `01_bare`(10-inst block)的 ~9.9% 吻合。
- 注:ON 的绝对 MIPS 有跨 fixture 噪声(13>14 一处反序);但「检查占比」是每个
  fixture 自己 OFF/ON 的比值、热漂移已被交错 A/B 抵消,这一列单调可信。

### 4 次累积对照(perf trail)

中断检查的 pending 判断会随 milestone 变重,这里留累积对照(以最敏感的
03_csr_heavy 检查占比为风向标;每次用 `run_perf.py` + 本节交错 A/B 同法重测):

```
  # | milestone 状态       | 03_csr_heavy 检查占比 | 备注
  1 | 中断检查 OFF(基线)  | 0%                    | 本次实验对照基线
  2 | CLINT-only(当前)    | 21.9%                 | 本次实验
  3 | + PLIC               | 90.5%                 | csr_heavy 跃升 21.9% → 90.5%; PLIC rwlock+scan 主导 (详见下方 "## a_03 末 PLIC + UART 落地后" 段)
  4 | + AMO                | (待测)                | AMO milestone 后再测一次
```

注:PLIC milestone 落地后补第 3 行,AMO 后补第 4 行。a02_7 套件本身(11 个 fixture
+ `run_perf.py`)就是这个 perf trail 的固定标尺。

### 后续 TODO(PLIC milestone 后做)

PLIC 实装后,中断检查机制就齐了(CLINT + PLIC)。届时:

1. 再做一次 **中断检查 ON / OFF × a02_7 全套** 的对照(同本节交错 A/B 方法),补上
   "4 次累积对照"的第 3 行 —— 此时 check 的 pending 判断已并入 PLIC 外部中断源,
   预期比 CLINT-only 明显更重。
2. block 大小对照 —— **bare 纯算术版本本 session 已做**(见上方「block 大小
   sweep」,fixture 12-16,曲线已测出)。PLIC 后可再扩展含 load/store/mmu 的 block
   大小对照(此次 sweep 只覆盖纯算术 bare)。
3. 据结果决定:**要不要用 `local_count` 把中断检查节流** —— 现在是每 block 查一次,
   改成"累计够 N 条指令才查一次"(`local_count` 攒到阈值再 poll),让小块密集的代码
   (csr 密集 / OS trap 路径)不必每个微型 block 都付一次 check。代价是中断投递最多
   延迟 N 条指令(RV 不要求即时投递,可接受)。这个权衡等 PLIC 后 check 变重、收益
   更明显时再定。

---

## a_03 末 PLIC + UART 落地后 中断检查开销实验 — 2026-05-24

体例扩自 a_02_session_016 OFF/ON 两 binary 对照实验 (上文 "## 中断检查开销实验
(a02_7 perf 套件) — 2026-05-23" 段), 扩到 4 binary (ALL_OFF / CLINT_ONLY /
PLIC_ONLY / ALL_ON) — 量化 a_03 末 csr_mip_read 内 5 源 OR 中 CLINT 跟 PLIC 各自
单独占比, 为 PLIC ls[] 优化方案 (T6.2 待实装) 提供 ROI 直接数据. session log:
`a_03_session_006.md`.

### 4 binary patch 体例

跟 a_02_session_016 临时注释 + 编完 revert 同体例 (不引入 macro gate), 编 4 个
release binary 到 /tmp:

- **ALL_OFF**:    `src/core/dispatcher.c` L158 `if (trap_check_interrupt(hart) != 0) continue;` 整行注释 (跟 a_02 OFF 同)
- **CLINT_ONLY**: `src/core/csr.c` `csr_mip_read` 内 `mip_view = hart->trap._mip_sw;` 改 `= 0;` + 注释 PLIC 2 个 if; 留 CLINT 2 个 if
- **PLIC_ONLY**:  `src/core/csr.c` `csr_mip_read` 内 `mip_view = hart->trap._mip_sw;` 改 `= 0;` + 注释 CLINT 2 个 if; 留 PLIC 2 个 if
- **ALL_ON**:     无 patch (跟 a_02 ON 同)

每个 binary 编完 `git checkout` revert, `cp` 到 `/tmp/jit-emu-rel-{all_off,clint_only,plic_only,all_on}`;
4 binary 编出在同一 git state, 跨 binary 性能可比.

### 跑批方法

`tests/review/run_perf.py` 加 `--compare BIN1,BIN2,...` 模式 (跟原 single-binary
模式并存, a_02 测法不破坏). 单 fixture 上严格 ABCDABCD... 交错 6 轮 = 24 跑/fixture;
16 fixture × 4 binary × 6 轮 = **384 跑**. 交错抵消 turbo 降频 / cache 热漂移 / VM
steal time, 跟 a_02 OFF/ON 交错 6 对体例一致.

环境同 a_02_session_016: release `-O2`, 解释器, 单 hart.

### 结果 (median MIPS, 6 轮)

```
  fixture              ALL_OFF  CLINT_ONLY  PLIC_ONLY  ALL_ON
  01_bare                175.9      194.9       61.6     75.2
  02_mmu_sv32            176.0      190.9       61.7     74.8
  03_csr_heavy           113.6       83.5        8.7     10.8
  04_mem_dense           166.7      168.6       54.8     67.0
  05_mem_tlbmiss         152.6      154.9       40.1     49.3
  06_bare_load           213.5      211.0       62.9     75.8
  07_bare_store          125.4      128.7       53.4     62.7
  08_mmu_dense_load      185.9      183.5       60.2     71.7
  09_mmu_dense_store     117.9      123.3       51.8     59.8
  10_mmu_sparse_load     107.5      104.8       48.6     54.8
  11_mmu_sparse_store     90.1       89.4       44.7     50.2

  block 大小           ALL_OFF  CLINT_ONLY  PLIC_ONLY  ALL_ON
     2                  174.2      141.0       16.0     20.3
     8                  194.2      211.3       54.0     65.1
    16                  190.5      196.9       83.9     97.4
    32                  201.7      233.8      129.6    144.9
    64                  210.2      243.6      171.4    182.3
```

### 单源占比 (对照 ALL_OFF baseline)

CLINT 占比 = (ALL_OFF - CLINT_ONLY) / ALL_OFF × 100 (负值 = CLINT 拖累);
PLIC 占比  = (ALL_OFF - PLIC_ONLY)  / ALL_OFF × 100 (负值 = PLIC 拖累);
总开销     = (ALL_OFF - ALL_ON)     / ALL_OFF × 100.

```
  fixture              CLINT 占比%   PLIC 占比%    总开销%
  01_bare                +10.83       -65.00       -57.24
  02_mmu_sv32             +8.48       -64.92       -57.52
  03_csr_heavy           -26.51       -92.38       -90.49
  04_mem_dense            +1.15       -67.10       -59.83
  05_mem_tlbmiss          +1.47       -73.75       -67.70
  06_bare_load            -1.18       -70.54       -64.48
  07_bare_store           +2.61       -57.45       -50.03
  08_mmu_dense_load       -1.27       -67.62       -61.45
  09_mmu_dense_store      +4.57       -56.04       -49.27
  10_mmu_sparse_load      -2.53       -54.77       -49.03
  11_mmu_sparse_store     -0.76       -50.44       -44.26
  12_blocksize_02        -19.04       -90.84       -88.36
  13_blocksize_08         +8.82       -72.18       -66.47
  14_blocksize_16         +3.36       -55.98       -48.85
  15_blocksize_32        +15.89       -35.73       -28.15
  16_blocksize_64        +15.88       -18.48       -13.28
```

完整 384 跑单跑 MIPS 序列见 `tests/review/out/perf_res.txt` (out/ gitignore).

### 关键结论

1. **PLIC 占整个中断检查开销 ~99%, CLINT 几乎 noise** —
   CLINT_ONLY ≈ ALL_OFF (一般 ±5% 噪声, 仅 csr_heavy / blocksize_02 才显出 -19% ~ -26%
   拖累 — 这俩 fixture dispatcher 入口频率最高); PLIC_ONLY 跟 ALL_ON 同一数量级
   (PLIC `rwlock + scan` 双调用是绝对大头). 即: 优化 PLIC 路径可逼近 ALL_OFF
   baseline (~55% 平均 MIPS 回升).

2. **PLIC 单独拖累 ~50-90% MIPS, 跟 fixture 形态强相关**:
   - 极端高: `03_csr_heavy` -92.38% (csrr/csrw mip 让 csr_mip_read 调用频率倍增);
     `12_blocksize_02` -90.84% (小 block dispatcher 入口高频).
   - 中段:   `01_bare` / `02_mmu_sv32` / `04`-`09` mem/mmu 系列 -55% ~ -70%.
   - 极端低: `16_blocksize_64` -18.48% (大 block 摊薄 PLIC 单次调用).

3. **block 大小依赖**: PLIC 拖累跟 block 大小负相关 (block 2 -90.84%, block 64 -18.48%);
   跟 a_02_session_016 "block 越小, 固定检查占比越大" 趋势一致, 但 a_03 末 PLIC
   引入后绝对值显著更陡 — PLIC 单调用 ~200 cycle 在小 block 时占整个 dispatch 周期
   ~80%+, 大 block 摊到 ~15%.

### artifact trail (不影响主结论)

实验设计本身的两个 microbenchmark gotcha, 记下来避免归零误读:

1. **PLIC_ONLY 16/16 fixture 一致比 ALL_ON 慢 ~15-22%** — 语义上 PLIC_ONLY ⊂ ALL_ON
   应当 ≥ ALL_ON, 实测一致更慢; 不是噪声 (16/16 一致 + 排除热漂移交错). binary size
   印证: `plic_only` 274976 字节 vs `all_on` 274568 字节, +408 字节. 解释: patch
   `mip_view = 0` + 砍 CLINT 触发编译器不同 inline / register 分配, 反生成更差代码.
   不影响"PLIC 是大头"主结论 (PLIC_ONLY 跟 ALL_ON 同数量级, 远低于 CLINT_ONLY / ALL_OFF).

2. **CLINT_ONLY 在 `15_blocksize_32` / `16_blocksize_64` 反而 +15.89% / +15.88% 快于
   ALL_OFF** — 大 block 下 dispatcher 入口稀, CLINT 2 atomic_load 实际开销几乎检测
   不出; release 6 sample median 噪声 + 编译器副作用. 落在 ±15% 噪声范围内, 不视为
   真实 "CLINT 加速".

### PLIC ls[] 优化方案 (a_03_session_005 末 user 拍, 本次实验直接印证 ROI)

a_03_session_005 末 user 看完 ON 单边 baseline 数据 (该数据 post-session_006 已被
本节正经 4 binary 测试替换), 拍方案 + 决定 T6 PLIC 优化提前 (排到 T5 virtio-blk
之前). 本 session 4 binary 实验给出 ROI 的直接量化:

- PLIC 占整个中断检查 ~99% → 砍 PLIC 几乎 100% 收回中断检查开销;
- 实施后预期 MIPS 量级 = ALL_OFF baseline; 极端 fixture 如 `12_blocksize_02` 从 20.3
  → ~170 MIPS (~8x), `16_blocksize_64` 从 182.3 → ~210 MIPS (~15%).

方案 trail 留底 (实装在 T6.2, 跟 `start_plan_a_03` [4.x] §2 同步):

- PLIC 加后台仲裁线程 (受 SDS 控制, 跟 CLINT timer / UART reader 同体例; main spawn/join
  对偶; system reset 时清队列残留)
- producer = `device_set/clear_pending`; consumer = PLIC 仲裁线程 (`cond_timedwait`
  100ms)
- MPSC 消息队列: 16 slot ring + `pthread_mutex_t` + 双 cond (not_full / not_empty);
  满时 producer block 等空位 (RV PLIC level 型不允许丢)
- 仲裁线程拉消息后持 wrlock 改 device_line + 重算所有 ctx 的 `ls[ctx_id]`; release
  锁前 `atomic_store ls[ctx_id]`
- dispatcher 主帧 `is_plic_*_pending` 改 `atomic_load ls[ctx_id]` (acquire) ~1 cycle
- claim/complete 同步路径 (慢路径, hart 主帧 wrlock 内改 claimed + 重算 ls)
- MTIP / MSIP / SEIP_sw 不动 (continuous condition / 单原子 / 已最轻)

实装前后再补一次同体例 4 binary 对照测试 (本节体例即模板), 量化优化实际 ROI 跟本次
预测的差距; 同时补 a_02_session_016 段 "4 次累积对照" 表第 4 行 "+ AMO" 或下个
milestone 风向标.
