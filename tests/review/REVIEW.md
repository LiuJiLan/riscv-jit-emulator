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

source 状态: commit `0e840a9` (PLIC 同步实装 baseline; 4 binary patch 从此 commit
派生编出, 后 git checkout revert).

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


## T6.2 PLIC 优化实装后对照 — 2026-05-24

source 状态: commit `6f72d85` (PLIC EIP 异步实装; 4 binary patch 从此 commit 派生
编出, 后 git checkout revert).

T6.2 实装 (`a_03_session_007`) 后跟 T6.1 同 4 binary 体例 + 同 a02_7 16 fixture +
同 ABCDABCD 6 轮交错 (= 384 跑) 重测, 量化 PLIC refresh thread 异步化 +
`plic_ctx_eip` atomic 直返路径的实际 ROI.

跑批方法 / patch 体例完全跟上文 T6.1 段相同, src/ 是 T6.2 后状态 (新加 plic refresh
ring + consumer 线程 + atomic plic_ctx_eip); 不重述。

注: T6.2 后续 commit (`a_03_session_007` 末: 改 MMIO pending bitmap 也 atomic
cache, 跟 ctx_eip 同时间点 refresh) **不重新跑 perf** — pending bitmap 读不在
csr_mip_read hot path 上 (csr_mip_read 调 is_plic_*_pending 走 ctx_eip atomic;
pending bitmap 是 guest MMIO 读用, 调用频率低), 速度影响不显著, REVIEW 不需要补
第 3 行对照.

### 结果 (median MIPS, 6 轮)

```
  fixture              ALL_OFF  CLINT_ONLY  PLIC_ONLY  ALL_ON
  01_bare                214.2       168.2      163.8    163.2
  02_mmu_sv32            212.8       162.8      164.2    160.3
  03_csr_heavy           113.0        90.3       87.4     78.9
  04_mem_dense           181.4       158.0      155.3    152.0
  05_mem_tlbmiss         169.2       140.8      141.6    137.5
  06_bare_load           201.7       203.7      201.5    198.4
  07_bare_store          136.2       119.7      116.8    118.4
  08_mmu_dense_load      189.8       173.8      177.7    171.3
  09_mmu_dense_store     126.2       112.3      108.8    111.7
  10_mmu_sparse_load     103.4       104.1      101.8    101.1
  11_mmu_sparse_store     90.7        88.0       86.8     87.1

  block 大小           ALL_OFF  CLINT_ONLY  PLIC_ONLY  ALL_ON
     2                  172.1       144.5      140.7    129.7
     8                  232.6       181.9      182.8    173.2
    16                  203.7       180.2      177.1    175.5
    32                  236.0       192.9      193.8    192.9
    64                  244.9       205.8      205.5    201.2
```

### 单源占比 (对照 ALL_OFF baseline; 同 T6.1 段公式)

```
  fixture              CLINT 占比%   PLIC 占比%    总开销%
  01_bare                -21.47       -23.49       -23.80
  02_mmu_sv32            -23.51       -22.86       -24.67
  03_csr_heavy           -20.07       -22.67       -30.19
  04_mem_dense           -12.87       -14.39       -16.19
  05_mem_tlbmiss         -16.76       -16.31       -18.71
  06_bare_load            +0.98        -0.09        -1.65
  07_bare_store          -12.11       -14.25       -13.08
  08_mmu_dense_load       -8.44        -6.38        -9.77
  09_mmu_dense_store     -11.02       -13.80       -11.55
  10_mmu_sparse_load      +0.67        -1.51        -2.19
  11_mmu_sparse_store     -3.06        -4.34        -4.01
  12_blocksize_02        -16.06       -18.25       -24.65
  13_blocksize_08        -21.79       -21.41       -25.54
  14_blocksize_16        -11.58       -13.06       -13.86
  15_blocksize_32        -18.26       -17.85       -18.24
  16_blocksize_64        -15.98       -16.11       -17.86
```

完整 384 跑单跑 MIPS 序列见 `tests/review/out/perf_res.txt` (out/ gitignore;
本次跑后会被覆盖, 单跑序列只在跑后短暂可读).

### PLIC% T6.1 → T6.2 对照 (核心 ROI 指标)

```
  fixture              PLIC% T6.1   PLIC% T6.2    改善 pp
  01_bare                -65.00       -23.49        +41.51
  02_mmu_sv32            -64.92       -22.86        +42.06
  03_csr_heavy           -92.38       -22.67        +69.71   ★ 灾区
  04_mem_dense           -67.10       -14.39        +52.71
  05_mem_tlbmiss         -73.75       -16.31        +57.44
  06_bare_load           -70.54        -0.09        +70.45   ★ 打平 ALL_OFF
  07_bare_store          -57.45       -14.25        +43.20
  08_mmu_dense_load      -67.62        -6.38        +61.24
  09_mmu_dense_store     -56.04       -13.80        +42.24
  10_mmu_sparse_load     -54.77        -1.51        +53.26   ★ 打平 ALL_OFF
  11_mmu_sparse_store    -50.44        -4.34        +46.10
  12_blocksize_02        -90.84       -18.25        +72.59   ★ 灾区
  13_blocksize_08        -72.18       -21.41        +50.77
  14_blocksize_16        -55.98       -13.06        +42.92
  15_blocksize_32        -35.73       -17.85        +17.88
  16_blocksize_64        -18.48       -16.11         +2.37
```

灾区 (`03_csr_heavy` / `12_blocksize_02`) 跟 mem 系列从 ~50-90% 拖累降到 ~10-25%;
`06_bare_load` / `10_mmu_sparse_load` 几乎打平 ALL_OFF. T6.2 实测改善对偶 T6.1 段
"PLIC ls[] 优化方案" 预测 (~PLIC 几乎打平 ALL_OFF), 趋势一致.

### 关键结论

1. **PLIC 拖累从大头退化到跟 CLINT 同量级**:
   T6.1 PLIC_ONLY ~10x 慢于 ALL_OFF (rdlock + scan 96 sources); T6.2 PLIC_ONLY 跟
   CLINT_ONLY 几乎平 (~15-20% 拖累, 都是 atomic_load + if 框架开销). PLIC 路径已
   退化到跟 CLINT atomic_load 同形态 — 跟设计目标一致.

2. **剩余 ~15-25% 总开销不再是 PLIC 的事**:
   T6.2 后 ALL_ON vs ALL_OFF 仍有 ~15-30% gap, 但 CLINT_ONLY 跟 PLIC_ONLY 几乎平
   说明这部分是 `csr_mip_read` 内 if + 4 个 `is_*_pending` atomic_load + `trap_check_
   interrupt` 函数调用框架本身的固定开销, **跟 PLIC 实装路径无关**. 要进一步降只能
   走 plan §2 #49 中断节流 (`local_count` 攒够 N 才 poll) 或者 inline atomic_load 到
   dispatcher 主帧 (越过 csr_mip_read 函数调用).

3. **block 大小依赖大幅消失**:
   T6.1 PLIC% 跟 block 大小强相关 (block 2 -90.84% → block 64 -18.48%, 跨度 72 pp);
   T6.2 PLIC% 几乎跟 block 大小无关 (block 2 -18.25% / block 64 -16.11%, 跨度 2 pp).
   意思 "PLIC 单调用成本"从 ~200 cycle 降到 ~1 cycle, 小 block 摊薄不出来的开销
   消失了 — 进一步印证 atomic_load 直返路径生效.

### artifact trail (不影响主结论)

1. **T6.1 vs T6.2 单数字不能跨时段直接对比** — 不同时段 host CPU turbo / VM steal
   time / cache 状态都不同, T6.2 表内 CLINT_ONLY 拖累 ~15-23% 比 T6.1 表内 CLINT_ONLY
   ~±5% 看起来 "退化" 是 host 状态差异 + ABCD 交错抵消 (T6.1 跟 T6.2 内部各自一致,
   两表内 PLIC% 相对 CLINT% 才有可比性). 真正的对比指标是 PLIC% T6.1 → T6.2 (上表),
   不是单 binary 单数字.

2. **CLINT_ONLY / PLIC_ONLY 在 T6.2 表内字节大小** — clint_only 287416 / plic_only
   287264 / all_on 286856, 跟 T6.1 类似的 "patch 不规则字节增长" artifact (release
   优化器对小 patch 不同 inline 决策的副作用). 不影响 "PLIC 已退化到跟 CLINT 同量级"
   主结论.

3. **refresh thread OS 调度开销** — T6.2 引入了 single consumer thread (cond_timedwait
   100ms 心跳); 单 hart 单 dispatcher 跟 refresh thread 是 2 OS thread, 可能争抢 CPU
   核. 本次实测 a02_7 全部 fixture (主帧 spin 没主动 device_set/clear_pending) 下
   refresh thread cond_timedwait 100ms 心跳 + 几乎不消费 event, OS 调度开销可忽略
   (实测 PLIC_ONLY 跟 ALL_OFF 接近印证). 真活跃场景 (a_03 stress fixture / 真 OS 跑)
   下需重测 — 留 trail 给 a_03/a03_4 stress fixture 拿数据.

### 跟 plan §2 #49 中断节流方案的关系

T6.2 后 PLIC 不再是中断检查的瓶颈, 剩余 ~15-25% 的 ALL_ON vs ALL_OFF gap 完全来自
中断检查框架本身. plan §2 #49 中断节流 (`local_count` 攒够 N 条指令才 poll) 现在
ROI 更清晰:

- 攒够 N=16 → poll 频率 1/16, 框架开销摊薄到 ~1-2%
- 代价 = 中断延迟 + N/2 条指令 (~80ns @ 100MIPS), RV spec 不要求即时投递, 接受
- 决策: 留给后续 milestone (跟 JIT 真接入后再评估; JIT 跑期 dispatcher 入口频率会
  变, 现在节流的 ROI 数字会偏 — 等 JIT 落地后再拍).

---

## a_03 收尾测试整理 — run_tests 改造 + A.7 prelude + B.3 跑批 — 2026-05-30

范围: run_tests.py 接口改造 (必做前置) + A.7 老 fixture boot prelude 批量补 +
B.3 第一轮全量跑批分类。stub.S 三段→四段重排 (全量) **本次未做**, 留后续 session
(见 reorg_spec.md "banner 结构" 已升四段; 存量 stub.S 仅这次动到的加了第四段)。
session log: `a_03_session_018.md`。

### 工具演进 (run_tests.py + reorg_spec.md, durable)

- **接口换新**: 旧 `[EMU, artifact]` 已对不上现 CLI (报 unknown arg)。新 base =
  `[EMU, --bios, artifact]`。
- **方案 A explicit declaration**: stub.S banner 第四段「运行参数」声明 `# RUN-*`
  tag, run_tests grep 解析覆盖默认 (撤了"后缀自动检测" + "集中 JSONL", 见
  a_03_session_017/018 brainstorm trail)。schema (reorg_spec.md "运行参数" 段):
  `RUN-BLK` / `RUN-STDIN` / `RUN-EXPECT-EXIT` / `RUN-RELEASE` / `RUN-TIMEOUT`。
- **双 frozen 二进制**: `out/riscv_jit_emulator` (debug+ASan, 默认) +
  `out/riscv_jit_emulator_release` (RUN-RELEASE 用)。a02_7 perf 套件 16 fixture 打
  `# RUN-RELEASE` — debug 比 release 慢 ~30x 跑不满 timeout, 正确性批走 release
  跑到完整作冒烟, 测速仍归 run_perf.py。
- **fail loud**: tag 残缺 (未知名 / value tag 缺值 / 布尔 tag 带值 / 文件不存在 /
  数值解析失败 / RUN-RELEASE 但 release 二进制缺) → 该 fixture 标 CONFIG-ERROR 跳过。
- **ASAN_OPTIONS=detect_leaks=0**: 批跑非 gdb 环境下 LSan exit-time 扫描会把正常
  停机报泄漏 → 非 0 退出污染 PASS/FAIL, 关掉 (跟 CLion run config 体例一致)。
- **RUN-STDIN 延迟投递** (`STDIN_DELAY=0.1s`, 见下方 uart 竞态 trail)。

### B.3 第一轮跑批结果

121 fixture, **0 FAIL / 0 CONFIG-ERROR / 0 TIMEOUT** (A.7 + harness 修完后)。
中间态 trail: A.7 补之前 32 个 TIMEOUT — 16 个 A.7 缺 prelude (debug+release 都挂)
+ 16 个 a02_7 perf (debug 慢, RUN-RELEASE 后秒完) + 1 个 uart_rx_echo 启动竞态。
sifive 两个走 RUN-EXPECT-EXIT 断言 PASS (05→0 / 06→42)。

### 演进 trail 续 (durable — 接 a_02 收尾的 A/B/C/D)

- **(E) A.7 boot prelude 加一条指令 → 地址/计数漂移** —— Smdbltrp 落地后 mstatus.MDT
  reset 默认 = 1, 强制"写 mstatus.MIE"清 0; 老 fixture (写在 MDT 语义之前) 开中断前
  没清 MDT → MIE 永 0 → 中断不 fire → 等 IRQ 的 fixture 永久 spin。A.7 在每个开 MIE
  的指令 (`csrrsi x0, mstatus, 0x8` 等) 前补一条 `csrw 0x310, x0` (清 mstatush.MDT;
  数字 0x310 因 binutils <2.38 不认 mstatush 助记符)。**副作用**: 该指令其后所有地址
  +4 字节、total_count +1、且 csrw 是硬块边界故多一个 block。**功能正确** (这些
  fixture 的可观测正确性靠寄存器值 mcause/claim/data, 地址无关, 批跑全 exit=0); 漂的
  是 stub.S「期望结果」段里的绝对 PC / total_count / block 计数注解。

### 待校准 fixture 清单 (因 (E), A.7 补 prelude 后期望漂; 共 17 个 A.7 fixture)

prelude 仅清 MDT 不改可观测寄存器结果, 故只有带"绝对 PC / total_count / block 逐块
计数"注解的 fixture 的「期望结果」段需要重算。等全量三段→四段重排 pass 时随手校准
(不本次逐一改, 一阶段一事):

- **a01_7/01_trap_priv_save_restore** —— 重度 PC/count 注解 (banner 列了每条指令的
  0x00/0x08/.. 地址 + 逐 block count/total)。prelude 插在 0x0c 前 → 其后全 +4、每个
  block count / total 全漂。校准量最大的一个。
- **a02_5/06_int_priority** —— banner "xepc 落在 wait_loop 内" 的落点推导依赖 prelude
  前后指令布局, 地址平移后 xepc 具体值漂 (中断仍正确 fire, mcause 仍 0x80000003)。
- 其余 14 个 (a02_5/01,02,07; a02_6/02; a03_2/01,02,03,04,05,08; a03_3/01,02,04;
  a03_5/01,02) —— 主要核对各自 banner 有无绝对 PC / total_count 注解; 多数只列寄存器
  期望 (地址无关) 应不受影响, 重排时逐个扫一眼确认。

### uart_rx_echo 启动竞态 trail (harness 侧解决, 不改 src)

`a03_3/04_uart_rx_echo` 在瞬时 pipe stdin (`printf 'abc' | emu`) 下永久 spin; 延迟
≥10ms 再投则 echo "abc" exit=0 (实测 0.01/0.05/0.1 都过, delay=0 挂)。根因: reader
线程 vs guest 设备 init (PLIC source 10 + UART ERBFI 使能) 的启动先后, pipe 把 3 字节
瞬时塞进来时 RX 字节早于中断使能就绪 → 丢边沿。UART 代码本身在 IER 写后有重评
device_line (uart.c L296-300), 仍挂说明是更细的瞬时竞态。**决策 (user 拍)**: harness
侧处理 (模拟器 init 极快 <10ms, 给小延迟不脆) —— run_tests 对 RUN-STDIN 一律用 Popen
先 spawn → sleep `STDIN_DELAY=0.1s` → 再写 stdin, 让 guest 先跑完 init。**不改 src**;
emulator 侧那个瞬时竞态留作潜在 forward item (真要查是 src 并发排查, 当前不做)。
fixture 本就被其 Makefile 注成主要给交互/键盘用; harness 延迟让自动化批跑也能稳过。

### 工具链升级连带 — B.3 重整时必做 (session_018 末)

session_018 末把 RISC-V 工具链从 apt gcc 10.2 / binutils 2.35.1 升到 riscv-collab
预编译 (ubuntu-22.04, release 2026.05.19) **gcc 16.1.0 / binutils 2.46** (装到
/opt/riscv-2026.05.19, 前缀仍 riscv64-unknown-elf-)。两个连带影响 B.3 重整:

1. **-march 必须补 `_zicsr_zifencei`** —— binutils 2.46 把 Zicsr/Zifencei 从 base I
   拆成独立扩展, 现 fixture `-march=rv32imac` 编 csr/fence.i 报 "extension zicsr/
   zifencei required" → 新工具链编不过。**逐 fixture 按语义改, 不机械 sed** (存量:
   89 rv32imac / 35 rv32im 无 c / 1 rv64imac; 后两类可能是故意限 ISA / class64 reject
   负测, 盲 sed 会错改)。用 csr/fence.i 的补 zicsr_zifencei, 故意限制的保留。
2. **期望校准基准 = gcc16 输出** —— gcc16 codegen ≠ gcc10.2, 指令数/地址在 A.7
   prelude drift 之上再漂。B.3 重整 = 改 march → 用 gcc16 重 make → 重排注释 + 对着
   gcc16 实跑输出校准期望, 一次到位 (别对着旧 gcc10.2 的旧期望逐字抠)。

当前 out.bin 仍是旧 gcc10.2 产物 (A.7/B.3 首轮跑批基准), 在 B.3 用 gcc16 重 make 前有效。

---

## B.3 全量重整 — march 迁移 + gcc16 重编 (分多 session) — 2026-05-30 起

session_019 起做 B.3 大头, 一个 sub-milestone 一遍 (march 改 → gcc16 重编 → 注释核
四段 → 期望核对 → run_tests 跑批确认绿)。session log: `a_03_session_019.md`。

### 关键发现 1: 纯汇编 fixture 跨工具链逐字节相同 (durable, 大幅收窄重整范围)

实证: 纯汇编 stub (手写指令, 含 `.option norvc`) 在 gcc16+rv32imac_zicsr_zifencei 下
重编, out.bin 与旧 gcc10.2+rv32imac 产物 **md5 逐字节相同** —— 汇编器对手写指令不做
codegen 选择, `_zicsr_zifencei` 只是放行 csr 指令、编码不变。**只有 `.S+.c` fixture**
(main.c 经 gcc codegen) 的 out.bin 才会因 gcc16 漂。全项目 `.S+.c` fixture 仅 21 个
(见 `ls */*/*/main.c`); 其余纯汇编 fixture 期望零漂, B.3 对它们 = 仅改 Makefile march
行 (+ 注释核四段, 多数已合规)。

连带: "最重的 a01_7/01" 也是纯汇编 → gcc16 不再添新漂, 它的 drift 全来自 018 已加的
A.7 prelude (已在 on-disk out.bin、已标上文待校准), B.3 不再叠加。

### 关键发现 2: stub.S 多数已四段合规 (运行参数段可选)

四段 = 运行参数 / 测试目的 / 思路 / 期望结果。运行参数段仅 RUN-* tag fixture 才有
(018 已给 a02_7/sifive/virtio/uart 那批补上), 其余按 spec "无则整段省略"。a_01 全部
stub.S 已是 (测试目的|思路|期望结果) 三段 = 四段合规形态。所以 "三段→四段重整" 对绝大
多数 fixture 是 no-op; B.3 实质 = **march 迁移 + .S+.c 期望校准**, 不是 118 stub 大重排。

### 演进 trail 续 (durable — 接 A/B/C/D/E)

- **(F) Smdbltrp 改 halt 路径: 旧 triple-fault dump → MDT abort; in_trap 字段废除** ——
  a_03 session_011 落地 Double Trap + 废 in_trap 字段后, 早期 trap fixture (a_01/a_02)
  里 "handler 末再 ecall_from_M 第 2 次 → triple fault → in_trap=3, mcause=11" 的收尾
  路径不再成立: 第 2 次 M-trap 撞 MDT=1 → 直接 MDT abort 停机, mcause 停在首次 trap 的
  cause (非 11), in_trap 字段已不存在 (现 dump 显示 mstatus.MDT=1)。**核心逻辑断言仍
  成立** (寄存器值 / scause / sepc / mtval 都对, DEBUG_CPU_DUMP reg/trap dump 可核),
  漂的只是「期望结果」段尾部那几行 halt 状态注解 (in_trap=3 / mcause=11 / 绝对 pc /
  total_count)。这条根因独立于 gcc16 (是 emulator 侧演进), 影响所有早期"末尾再触一次
  M-trap 收尾"的 trap fixture。

### a_01 已做 (session_019)

- march: 26 个 csr-using fixture CFLAGS 补 `_zicsr_zifencei` (base 保留: rv32imac ×18 /
  rv32im ×8; 非 csr 的 a01_1/3/4 + a01_10/03-07 rvc + rv64imac class64 负测 不动)。
  a01_5/01 Makefile 旧 zicsr 体例注释加 2.46 迁移 trail (不删旧叙事)。
- gcc16 重编全 a_01: 28 纯汇编 fixture out.bin 逐字节不变; 9 个 `.S+.c` 漂
  (a01_8/04-09 + a01_9/03 + a01_10/01,02)。
- run_tests 全量批 (121 fixture, a_01 新 out.bin + a_02/a_03 仍 018 gcc10.2 基准):
  **0 FAIL / 0 TIMEOUT / 0 CONFIG-ERROR**。

### a_01 .S+.c 期望待校准 (因 (F) + gcc16, 共 9 个; 核心断言已过, 仅尾部注解漂)

下列 fixture「期望结果」段尾部 halt 注解 (in_trap=3 / mcause=11 / 绝对 pc/total_count)
描述的是旧 triple-fault 收尾, 现走 MDT abort; 核心寄存器/trap 断言实跑仍对。按
"一阶段一事" 标 trail, 不本轮逐一改 (待 user 拍校准深度):

- a01_8/04_sv32_lw_sw_basic (x10/11/12/13 ✓; mcause 期望 11 实际 9, in_trap=3 废)
- a01_8/05_sv32_hw_managed_ad (x15 A/D 写回 ✓; 同上尾部漂)
- a01_8/06_sv32_load_page_fault (cause 13 ✓; x13 lw PC 因 gcc16 漂)
- a01_8/07_sv32_superpage_lw_sw (同 04)
- a01_8/08_sv32_pt_not_in_ram (cause 5 ✓; x13 PC 漂)
- a01_8/09_sv32_root_pt_not_in_ram (cause 1 ✓; x13/x14 绝对 PC 漂)
- a01_9/03_medeleg_deliver_s (scause=9/sepc ✓; M 侧 mcause 期望 0 实际 16, in_trap=3 废)
- a01_10/01_sum_load_fast_path (x10=0xDEADBEEF/x11=0/cause 13 ✓; x14 lw PC 漂)
- a01_10/02_sum_store_fast_path (x10=0xCAFEBABE/cause 15 ✓; x14 sw PC 漂)

### 更深发现 + 策略升级: 同步 trap fixture 因 MDT=1-at-reset 失效 → test_dev 停机改造 (user 拍)

核实 pure-asm trap fixture 实跑寄存器时发现: **核心断言真的不成立了** (a01_5/04 x2=99 mret
证据 → 实跑 0; a01_6/02 x1=4 csrr 抓的 mcause → 实跑 0)。根因比 (F) 叙事漂更深: cpu_reset
mstatus.MDT=1, 这些同步 trap fixture **没 A.7 prelude 清 MDT** → 第一个 trap 就撞 MDT=1 直接
MDT abort, **handler 根本没跑** → csrr 没抓到值 / mret 没回来。A.7 那轮只补了"开中断等 IRQ"
的 fixture, 漏了同步 trap fixture (它们同样需清 MDT 才能投递首个 trap)。批跑 0 FAIL 没抓到
(这些无 exit 断言, abort cleanup 也 exit 0)。

**user 拍策略 (session_019)**: "受当时开发阶段只能用 triple-fault 停机的都改 test_dev;
纯粹测 triple-fault 行为本身的留着 (改测 MDT abort)"。组合配方 (实证可行):
1. 有 handler 收 trap 的: `_start` 加 `csrw 0x310, x0` 清 MDT prelude (让首个 trap 投递);
2. 收尾 `csrw mtvec,x0 + ecall` (借 triple-fault 当停机) → test_dev FINISHER PASS
   (`li x30,0x100000; li x31,0x5555; sw x31,0(x30)`; scratch 统一 x30/x31, 与数据/断言寄存器
   错开) → SRS=0 干净退主循环, 寄存器停在干净末态可见;
3. 加 `RUN-EXPECT-EXIT: 0` (test_dev PASS 收尾退出码 0, 白捡验证点);
4. 删 inline 绝对地址注释 (随 prelude/test_dev 平移、layout 相关); 核心断言保留, 绝对
   pc/total_count 标本节;
5. 纯粹测停机协议的 (a01_5/05_triple_fault_halt) 不转 test_dev、不加 prelude — 改测
   "ecall under MDT=1 → MDT abort" (旧 triple-fault 协议的后继), 只更新叙事。

这是 **code-region 改 (fixture 重做), 偏离"整理 pass 只动注释"** — user 授权。out.bin 随之变
(纯汇编也变, 因为加了 prelude/test_dev 指令)。

### B.3 test_dev 改造进度 (session_019)

- **a_01_5 + a_01_6 done (7 fixture)**: a01_5/02,03,04 + a01_6/01,02,03 转 test_dev (exit 0
  断言全绿, 核心断言 x1/x2/x3 干净恢复可见); a01_5/05 改 MDT-abort 断言 (留)。a01_8/04
  期望段废叙事已先压一行 (待按本体例补 RUN-EXPECT-EXIT / 重核, 它是 .S+.c handler 在 boot 已
  清 MDT、寄存器本就可见的那类)。全量批 121 fixture 0 FAIL/0 TIMEOUT, PASS 2→8。
- **剩 (后续 session)**: a01_7/01-04 + a01_8/01-03 (pure-asm trap, 同体例); a01_8/04-09 +
  a01_9/03 + a01_10/01,02 (.S+.c, handler 在 stub/main.c, 末尾 ecall_from_M → test_dev,
  需逐个定位 handler); a01_9/01,02; a01_10/03-07 (跑飞 fetch-fault 停 → 末尾补 test_dev);
  然后 a_02 (~40) / a_03 (~30) csr fixture march 迁移 + 同类 test_dev 改造。
- 上方"a_01 .S+.c 期望待校准"清单中, 转了 test_dev 的 fixture 待校准项被本改造吸收 (核心
  断言转为干净可见); 未转的绝对 pc/total_count 漂仍留本节 trail。

### B.3 test_dev 改造 — 全量完成 (session_019, 并行 6 agent + 中心 march)

整盘 test_dev 改造一 session 内完成 (并行 subagent 按 sub-milestone 分组, 每 agent 自验证
重编+跑+查 exit/寄存器)。最终全量批: **121 fixture, 0 FAIL / 0 TIMEOUT / 0 CONFIG-ERROR,
PASS 81** (转换 fixture RUN-EXPECT-EXIT:0 + sifive + virtio + uart_rx 全绿; 仅 4 个 loader
reject 负测 exit=1 符合预期)。

- **转 test_dev (~79)**: a_01 (29, 除 a01_5/05) + a02_2/03/04/05/06 (~30) + a03_1/02/03/04/05
  (~20)。配方 = 清 MDT prelude (handler 需跑且未清的) + test_dev FINISHER PASS 收尾 (scratch
  x30/x31; a03_3/04/05 agent 用 t0/t1 — 与该批断言寄存器 t6/a6/a7 错开, 功能等价) +
  RUN-EXPECT-EXIT:0 + 删 inline 绝对地址 + 核心断言保留。
- **fall-through 守卫**: test_dev 是 SRS=0 在"下个 block 边界"才退, sw 非边界 → 收尾 sw 后若
  有可执行代码会 fall-through 重跑。停机段不在文件末尾的 fixture 加 `1: j 1b` 自旋守卫
  (a01_7/01 + 多个 a02_2)。
- **M-mode handler 需补 prelude**: BARE M-mode 下首个 trap 撞 MDT=1 abort → handler 跑不到,
  这类补 `csrw 0x310,x0` (a02_2/02,03; a02_5/08; a03_1/02; a02_2/04,07 等)。S/U-mode trap
  (用 SDT 非 MDT) 不需要。
- **留 (不转)**: a01_5/05 (改 MDT-abort 断言); a03_3/05,06 (sifive 已 test_dev); a03_6/* /
  a03_7/* (wfi) / a03_8/* (M ext) — 这批 **早已用 test_dev/sifive FINISHER 停机**, 无需改。
- **a02_7 ×16 排除 test_dev 改造** (user 拍: perf 套件 release 测速用) — 仅 march 迁移 +
  保留 018 的 RUN-RELEASE tag, 不动停机。
- **顺手改正的 doc 错**: a02_3/01 s0 期望 3→2 (case2 在 epilogue 自增前停机, 旧 doc 错);
  a02_6/03 perf a2/a3 期望 0x540C1E60→0x540D6AA0 (旧 doc 算错, 接 a_02 收尾 (D) 段那条)。

### ⚠ 系统性转换 bug: test_dev halt 漏 fall-through 守卫 (已全修, session_019)

test_dev FINISHER 停机语义 = 置 SRS=0, dispatcher 在**下个 block 边界**才退主循环。停机
`sw` 不是 block 边界 → 若 halt sw 后还有可执行代码 (handler / 数据 / 子程序), 会先
**fall-through 执行**到下个 block 边界才退。批跑只查 exit code (RUN-EXPECT-EXIT) 看不出来,
但 fall-through 会**悄悄改坏注释里的核心断言**。

agent 加 `1: j 1b` 自旋守卫不一致 (A1/B1 加了, C1 + 参考件 a01_5/04 漏了)。精确扫出
**21 个 halt 后有代码却漏守卫**, 已全部补 `1: j 1b` (sw 后紧跟; j 是 block 边界, SRS=0
即退, 不落进后续 handler/数据)。全量批仍 0 FAIL。

诊断起点 = 查"a03_2/03 claim 返 0": 同步 probe claim 返 3 正常 → 中断驱动场景失败 →
定位是 fall-through 重跑 handler (真 trap claim 返 3, fall-through 再 claim 返 0 覆盖 s2)。
补守卫后 s2=3 正确。**不是 PLIC bug。**

凡是 halt 后有代码的 fixture 都要带守卫 —— 后续新写/整理 fixture 沿用 (reorg_spec 体例点)。

### 真·预存期望漂 (跟 test_dev 改造无关, 留 trail 待 user 定改期望)

补守卫后复查, 三个 agent flag 项里只剩 2 个是真 drift (a03_2/03 已证是守卫 bug、已修):
- a03_1/01_plic_mmio_skeleton — 期望段列 14 行 `[plic] ...` stderr trace, 但 PLIC 骨架期的
  逐访问 trace fprintf 在完整 atomic 实装时已删 (plic.c 现 `[plic]` = 0 个) → 实跑无 trace。
  根因 (B) 类 (stub→完整实装)。改期望注释即可 (fixture 真正测的 "MMIO 不 fault" 靠 exit 0)。
- a03_2/05_int_priority — 6 源优先级实跑序 M_EXT/M_SOFT/S_EXT/S_SOFT/S_TIMER/M_TIMER,
  M_TIMER 垫底 (期望标准序应 M_TIMER 第 3)。**编码器正确** (trap.c 标准序); 真因 = MTIP 是
  `mip.MTIP=(mtime>=mtimecmp)` 连续条件、靠 timer 线程**异步**推 mtime, 其余 5 源瞬时置位 →
  轮到 M_TIMER 时 mtime 还没被推过 mtimecmp, 编码器跳过, 等异步推过才垫底 fire。根因 (C) 类
  (mtime 异步化)。改期望注释说明即可 (s3=6 handler 进入次数正确, 非 fall-through 污染)。
