# riscv-jit-emulator

[English](./README.en.md) | 简体中文

研究生级别的 RISC-V 用户态 JIT 模拟器,目标支持 RV32 G + 标准压缩扩展 C,跑通
OpenSBI / 小型 OS / FreeRTOS-with-MMU。milestone a_02(interpreter + bus/MMIO
阶段)已收口,子任务 T1~T6 全部落地;当前进入 a_03(PLIC + UART 外部中断 +
virtio-blk + 端到端验证)。


## 设计目标

- ISA: RV32 G (IMAFD_Zicsr_Zifencei) + 标准压缩扩展 C
- 三层 JIT 架构: Dispatcher / Translator / JitBackend (asmjit 起步,留 LLVM OrcJIT 接口)
- 宿主: Linux 用户态进程
- SMP 不实现但所有设计 SMP-ready
- 64 位接口预留,不实现


## 架构概览

模拟器的运行时结构可以从四个视角理解。本节给出概览,详细机制见后续各节,
逐文件设计见「项目结构」。

### 机器模型(RAM + CPU)

经典计算机模型可抽象为 CPU + RAM(此处不涉及 MMIO / port 等 I/O 概念)。内存侧:
`ram` 用单块匿名 mmap 管理 guest 物理内存,`loader` 把 ELF / 裸二进制按物理地址
放入。CPU 侧:`cpu_t` 是纯数据(寄存器 / CSR / TLB 派发表),`dispatcher` 是行为
(取指—译码—执行主循环)。「数据 / 行为」分离让 dispatcher 不持有状态 —— SMP 时
每个 hart 一份 `cpu_t`、一个 dispatcher 线程。

### 线程模型与三层 reset lifecycle

整体线程控制由三层 reset lifecycle(POR / System reset / HART reset)加 monitor
模型组成。当前是单 hart(`dispatcher` 在 `main` 线程内直调)加一个 timer 辅助
线程异步累加 `mtime`。monitor 模型(Hoare/Hansen 范式)把共享状态的 atomic 字段
与 memory_order 封装在 consumer / producer 接口内,`dispatcher` 因此「看不到」
多线程。详见「多线程 + reset lifecycle」。

### dispatcher 主循环

每个 hart 跑一个 `dispatcher`:`sigsetjmp` 一次性永久落点 + `while (in_trap < 3)`
多块循环 + 迭代头扫尾。每轮三个 block:选叶 TLB / 取指 / 进实际运行段(当前为
解释器)。退出由 `in_trap` 位段编码控制。详见「控制流概览」「in_trap 位段编码」。

### 实际运行块的内存模型

实际运行段的访存分两条 regime:REGIME_BARE(M 态或 bare satp,bypass TLB,
identity 偏移)与 REGIME_SV32(走叶 TLB + PTE 权限)—— 运行模型的分离本质上是
内存模型导致的结果。访存内部还存在 load / store 快慢路径不对称:load 命中直接
`*hva`,store 命中因 LR/SC reservation 等副作用必经 `store_helper`。详见
「TLB 拓扑」。

> JIT 子系统(Dispatcher / Translator / JitBackend 三层、jit_cache、SMC 检测)
> 为计划设计,尚未实装(见「当前已落地」末的未实现清单),不在本 README 的
> 当前进展范围内。


## TLB 拓扑

`cpu_t.tlb_table[4]` 是 4 槽派发数组,index 按 RV privilege encoding:

```mermaid
graph TB
    subgraph cpu_t.tlb_table
        direction LR
        U["[0] U<br/>始终副本语义"]
        S["[1] S<br/>ASID 容器<br/>eager 分配"]
        VS["[2] VS<br/>当前 NULL<br/>(H 扩展接口)"]
        M["[3] M<br/>永远 NULL<br/>(Trust bypass TLB)"]
    end

    U -.->|MSU 默认 副本| S
    U -.->|"MU-only 副本<br/>(未来 misa 派发)"| M
```

- **[0] U** 始终是副本语义(副本于 [1] S 或 [3] M,取决于 misa)。即使 MU-only CPU 中
  U 副本 M、槽内 NULL(因 M 走 bare 不查 TLB),"副本"语义本身不变。副本分配两路:
    - 初始化 —— `cpu_create` 按 misa 派发(MSU 副本 [1];MU-only 副本 [3])
    - 运行时 —— H 扩展(VS / VU 切换)由对应 csr_helper 维护 mirror
- **[1] S** —— ASID 数组容器 (`tlb_t **`),容器由 `cpu_create` eager 分配,entries
  由 walker 在首次访问该 ASID 时懒分配
- **[2] VS** —— 初版 NULL(无 H 扩展),H 扩展激活时同 [1] 形态
- **[3] M** —— 永远 NULL。Trust regime(M-mode 或任何 priv 带 bare satp)直接走
  identity + IS_GPA_RAM 检查,不需要 TLB(real CPU bare 下也 bypass MMU/TLB)


## in_trap 位段编码

`hart->trap.in_trap` 是位段编码(host 端协议,多种信号可叠加):

| 位段     | 值范围   | 含义                                                          |
|----------|----------|---------------------------------------------------------------|
| bit 0-1  | 0..3     | 实际 trap 嵌套深度:0/1/2 普通 nesting; 3 = triple fault       |
| bit 2    | 4..7     | 留白,防止 trap 嵌套位将来扩展跟下面位段冲突                  |
| bit 3    | 8..15    | 内部异常 / 内部正常停机(host 端协议,不是 RV trap)           |
| bit 4    | 16..31   | 留白                                                          |
| bit 5+   | 32+      | 未来停机扩展                                                  |

设计哲学(位表示叠加):

- 解释器 / JIT 内部只看 bit 0-1(trap nesting 视角),`in_trap < 3` 即继续
- 高位(bit 3+)仅由 dispatcher 写,解释器 / JIT 不碰
- `while (in_trap < 3)` 的判断作为安全闸:高位一被设值就 ≥ 8 > 3,dispatcher 自动退出
- reset 路径只清 bit 0-1;高位由 reset 流程显式按停机类型决定

跟 RV Smdbltrp 扩展(`CAUSE_DOUBLE_TRAP=16`)无关 —— Smdbltrp 是规范层 trap 投递机制,
in_trap 是 host emulator 退出协议,两者并存但语义不重叠。


## 控制流概览

```mermaid
graph TD
    Main[main] --> Ram[ram_init]
    Ram --> Loader[loader 后缀分发]
    Loader --> Cpu["cpu_create<br/>+ 启动协议<br/>(pc / satp / priv / a0 / a1)"]
    Cpu --> Disp[dispatcher]
    Disp --> Sj[sigsetjmp 永久落点]
    Sj --> Wh{"in_trap < 3?"}
    Wh -->|true| Hd["迭代头扫尾<br/>count 累加 / trap_check_interrupt"]
    Hd --> B1["block 1<br/>regime + current_tlb"]
    B1 --> B2["block 2<br/>mmu_translate_pc<br/>→ (pa, hva)"]
    B2 --> B3["block 3<br/>interpret_one_block"]
    B3 --> Ek{退出原因}
    Ek -->|hard boundary| Wh
    Ek -->|跨页 / BLOCK_INST_LIMIT| Wh
    Ek -->|trap_raise_exception<br/>longjmp| Sj
    Wh -->|false| Halt["halt<br/>(bit 0-1 = 3 或 bit 3 设)"]
    Halt --> Dump["main: dump<br/>reg / trap / state"]
```


## 多线程 + reset lifecycle

a_02 T5 落地后, 项目从单线程 (hart 线程独占 mtime 推进) 演化为多线程
(hart 线程 + timer 辅助线程并发, atomic mtime + monitor 模型同步)。本节
集中描述 main 流程 / 三层 reset / monitor 行为 / 错误处理统一走 signal
通道的伪码。详细协议见 `src/dummy.txt §7` (monitor 模型) + `§12` (谁
spawn 谁 join), 信号语义见 `src/runtime.h` 顶段 doc。

### 三层 reset lifecycle

```mermaid
graph LR
    A["POR<br/>(Power-On Reset)"] --> B["System reset<br/>(main while 每 iter)"]
    B --> A2["POR 收尾<br/>(while 退出后)"]
    B -.->|cpu_reset / clint_reset| B
    subgraph "未来 (future)"
        Hr["HART reset<br/>(dispatcher 内部 per-hart 重启)"]
    end
    B -.->|跟 HART reset 协同| Hr
```

- **POR (Power-On Reset)** — 进程启动一次: ram_init / clint_init / cpu_create
  (内部已写硬件 reset 默认状态) / 显式 set SRS=1 SDS=1 / 起 timer 辅助线程
  (clint_start_timer_thread)。timer 辅助线程跨 system reset 一直跑 (跟真硬件
  RTC oscillator 不掉电不停一致), 随 SDS 才退
- **System reset** — main while 每 iter: cpu_reset (regs/pc/mstatus/in_trap 清,
  保留 hartid hardwired) + clint_reset (mtimecmp/msip 清回 sentinel, **mtime
  不动 timer 不动**) + dispatcher(hart) + 区分 SR-only 重 iter vs shutdown 退
  出 (当前简化恒 shutdown, if(0) 占位预留)
- **HART reset** — dispatcher 内部 per-hart 重启 (future; dispatcher.c 末段
  注释占位); SMP 多 hart 时单 hart fail 不影响其他 hart

### main 流程伪码 (当前形态, a_02 T5 落地)

```c
int main(int argc, char **argv) {
    /* === POR === */
    ram_init();                   // 失败 → main 返 1
    clint_init();                 // 失败 → main 返 1 (走 destroy chain + return)
    cpu_t *hart = cpu_create();   // 失败 → main 返 1

    /* lifecycle 信号显式 set 1 (跟 runtime.c BSS 1 init 兜底重叠, 但显式更可读) */
    atomic_store_explicit(&system_reset_signal, 1, memory_order_release);
    atomic_store_explicit(&shutdown_signal,     1, memory_order_release);

    /* 起 timer 辅助线程; 失败内部 fprintf + set SRS=0 + SDS=0;
       main 不 check, 不分 error path — 错误走 SRS/SDS signal 通道 */
    clint_start_timer_thread();

    /* === System reset === */
    while (atomic_load_explicit(&system_reset_signal, memory_order_acquire)) {
        cpu_reset(hart);
        clint_reset();

        /* 占位: 所有 SRS-controlled 线程 spawn — 每 iter spawn / join, 跟
           system reset 同步起停 (例: 未来多 hart 走 pthread_create per hart;
           其他受 SRS 控制的辅助线程也在这里) */
        dispatcher(hart);                   /* 单 hart 直调; tri-fault 内部 set SRS=0 */
        /* 占位: 所有 SRS-controlled 线程 join — 跟上面 spawn 对偶 (hart 线程 +
           其他受 SRS 控制的辅助线程都在这里 join) */

        /* SR-only vs shutdown 分支 (当前简化恒 shutdown) */
        if (0 /* SR_only 占位; 未来真做 reset 重 iter */) {
            atomic_store_explicit(&system_reset_signal, 1, memory_order_release);
            continue;
        } else {
            atomic_store_explicit(&shutdown_signal, 0, memory_order_release);
            break;
        }
    }

    /* === POR 收尾 === */
    clint_join_timer_thread();    /* timer thread 看到 SDS=0 自然退后 join */
    /* dump (reg + trap + state) */
    clint_destroy();
    cpu_destroy(hart);
    ram_destroy();
    return 0;
}
```

三种退出路径都收敛到 `while → join → cleanup → return 0`:

1. **正常退出** (dispatcher tri-fault): dispatcher 函数末 `atomic_store(&SRS,
   0, release)` → main while 退 → else 分支 set SDS=0 → break → join (正常退) →
   cleanup
2. **timer spawn 失败**: clint_start_timer_thread 内 `atomic_store(&SRS, 0) +
   atomic_store(&SDS, 0)` → main while 因 SRS=0 不进 → SDS 已 0 (else 分支不执行) →
   join (pthread_t = BSS 0, glibc 返 ESRCH fprintf 一行不 fatal) → cleanup
3. **timer routine 内部 fail** (clock_nanosleep / clock_gettime errno): timer
   routine 同样 set SRS=0 + SDS=0 + return NULL → main while 退 → else 分支
   set SDS=0 (已是 0 no-op) → break → join (timer thread 已 return, 正常 join) →
   cleanup

错误处理统一走 SRS/SDS signal 通道, **不分 error path** — destroy chain 只在
cleanup 段写一次, 不在 spawn-fail / dispatcher-fail 路径重复。

### monitor 行为 (dummy.txt §7)

来自 Hoare/Hansen 并发 monitor 范式 — 每个共享状态模块封装内部 atomic 字段 +
memory_order, 对外只暴露 consumer / producer 接口; 调用方不感知内部同步细节。

项目内两个实例:

```mermaid
graph TB
    subgraph "clint = 完整 monitor"
        CL_State["_Atomic mtime / mtimecmps[N] / msip[N]<br/>+ pthread_t timer_thread"]
        CL_Actor["file-static timer_run<br/>(异步 atomic_fetch_add mtime)"]
        CL_Cons["consumer:<br/>is_clint_msip_pending<br/>is_clint_timer_pending<br/>clint_read"]
        CL_Prod["producer:<br/>clint_write"]
        CL_Life["lifecycle:<br/>clint_init / clint_reset / clint_destroy<br/>clint_start_timer_thread / clint_join_timer_thread"]
    end

    subgraph "runtime = degenerate monitor"
        RT_State["extern _Atomic int<br/>system_reset_signal (SRS)<br/>shutdown_signal (SDS)"]
        RT_Use["caller 直接 atomic_load_explicit /<br/>atomic_store_explicit (无封装函数)"]
    end
```

- **clint** = 完整 monitor: 三函数 lifecycle (init / reset / destroy) + spawn /
  join 对偶 (clint_start_timer_thread / clint_join_timer_thread, main 端显式调,
  不埋 destroy 内); file-static timer_run routine 跑在内部 actor, 不持 cpu_t
  只动 shared 字段。memory_order: producer release (atomic_fetch_add &mtime) /
  consumer acquire (is_clint_timer_pending / clint_read), 配对建立 happens-
  before
- **runtime** = degenerate monitor (单 flag 简化): `extern _Atomic int` 直接
  读写, 无封装函数; 单字段无跨字段一致性问题, 不强制接口函数。"SDS 蕴含 SRS"
  触发关系契约 — set SDS=0 之前必须先 set SRS=0 ("通知所有辅助线程退" 蕴含
  "system 自己也得退")

外部模块 (csr.c / dispatcher.c / bus.c 等) **不直接 atomic_\* 操作 clint 内部
字段**, 一律走 consumer/producer 接口。例: csr_mip_read 合成读时走
`is_clint_msip_pending(hartid) | is_clint_timer_pending(hartid)`, 不直接
`atomic_load_explicit(&clint.mtime, ...)` (后者破坏 monitor encapsulation,
违反 dummy.txt §7)。

### 协作式停机协议 (dummy.txt §12 + runtime.h)

```
spawn 调用方 (main)      worker thread (timer_run)       lifecycle signal
─────────────────       ────────────────────────        ────────────────
SRS = 1, SDS = 1                                        runtime.c
clint_start_timer_thread() ─→  pthread_create
                                                        SRS = 1, SDS = 1
                          ─→  while(atomic_load(SDS))
                                accumulate mtime
                                ...
正常: dispatcher tri-fault                              SRS = 0 (dispatcher)
main while 退                                           SDS = 0 (main else 分支)
                          ←  timer 看到 SDS=0 退
clint_join_timer_thread() ←  pthread_join
cleanup chain
return 0
```

- **不用 pthread_cancel / pthread_kill**: deferred cancel 在 interpreter / JIT
  深栈取消 = 状态半更新风险; pthread_kill 是发 signal SIGKILL 杀整进程不是
  杀线程; 一律 cooperative (atomic flag + worker periodic check + main join)
- **不引入 SIGINT/SIGTERM signal handler**: Ctrl-C 默认杀进程足够 (atomic_fetch_add
  是 lock-prefixed 单指令, 杀进程不破坏数据); 等 reset 体系成熟 (a_03+) 再做
- **destroy 函数纯 cleanup**: 不含 pthread_join (避免控制流隐式 block); spawn /
  join 对偶函数单独暴露由 spawn 调用方显式 join



## 项目结构


### 程序入口 + 全局配置


#### main.c

程序入口,三段式 lifecycle:POR(`ram_init` / `clint_init` / `cpu_create` + 显式
set SRS/SDS + 起 timer 辅助线程)/ system reset(`while` 每轮 `cpu_reset` /
`clint_reset` + 调 `dispatcher(hart)`)/ POR 收尾(join timer 线程 + dump +
destroy chain)。详见「多线程 + reset lifecycle」。

当前单 hart 直接在 `main` 线程内调 `dispatcher(hart)`,不真起 pthread;timer
辅助线程是目前唯一真实的并发线程。末尾 dump 段是临时的 —— 跑完后印寄存器 /
trap / state 给 fixture 对照,uart + 真 unit harness 上线后逐步替代。


#### config.h

项目自身的全局编译期宏(RAM 配置 / TLB 拓扑 / IALIGN / 块软边界 / CLINT 布局 /
TIMEBASE / MAX_HARTS)。运行期变量
(`host_ram_base` 之类)放 `ram.h`。


#### riscv.h

集中收纳 RISC-V 规范定义(priv 编码 / CSR 地址 / PTE 位段 / mstatus 字段 /
Exception Code)。增量原则,真用到一个加一个。跟 `config.h` 的职能划分:
`config.h` = "我们怎么配",`riscv.h` = "规范怎么定"。


#### loader.{c,h}

guest 程序加载,三函数 `guest_load_bin / guest_load_elf / guest_is_elf`。

ELF 加载严格 6 项 sanity 检查(magic / class=ELFCLASS32 / data=ELFDATA2LSB /
machine=EM_RISCV / type=ET_EXEC / phentsize+phnum)。

三条语义边界:**不挪 ELF**(段严格按 `p_paddr` 放,越界即失败)、**不管 entry**
(由使用者保证 = `GUEST_RAM_START`)、**不清 BSS**(由 guest startup 负责) ——
模拟器只是物理地址观察者,不是 OS loader,不做 relocation。


#### dummy.txt

跨文件协议总账(`.txt` 不参与编译,但属于源码组成 —— 收纳"涉及多个文件协作的运行时
机制 / ABI 约定 / 调用顺序契约"),13 段:

- **§1 sigsetjmp / siglongjmp 协议** —— dispatcher 一次性永久落点 + helper longjmp
  跳回 + 寄存器统一保护;路径 D 中断走 dispatcher 主帧 return-based;末段
  **load/store 不对称真机理** (TLB 缓存 hva + MMIO 不进 TLB → 命中路径结构不带
  分支; load 命中 *hva / store 命中走 store_helper 强制 reservation+SMC 副作用)
- **§2 x0 寄存器全局编码** —— 读 = 立即数 0,写 = dead store 到局部垃圾桶变量
  (让 IR / backend 不感知 x0 特殊性)
- **§3 satp ASID 合法性契约** —— csr.c 是生产者(WARL 截断),dispatcher 是消费者
  (直接索引 `tlb_table[priv][asid]` 无 bounds check)
- **§4 TLB 作为 block 入口分发机制** —— bare 模式也走 TLB,让派发逻辑跨 priv 统一,
  JIT 翻译产物跨 priv 复用
- **§5 报错风格** —— 每条失败 stderr 上恰好两行(内 why + 外 where);不引 enum
  错误码,不维护 `*_strerror` 翻译表,模块不调 `exit`
- **§6 CSR 物理存储字段命名五类** —— `_` 前缀 / `x` 前缀 / 不带前缀 / `_sw`-`_hw`
  后缀四套命名分类(第 5 类 = 软件可写子集 + 异步源合成读,如 `_mip_sw`)
- **§7 多线程 vs 多 HART 术语 + monitor 模型** —— per-hart / shared / thread-local
  三态;shared 字段一律 atomic;shared 模块封装 consumer/producer 接口(Hoare/Hansen
  范式)
- **§8 PMP / MMU / 内存三层关系** —— mmu 只翻译 GVA→PA; ram+bus 才真访问; PMP 长期
  不实装
- **§9 cause 0 路径 + "0=成功" 接口约定** —— mmu_translate_pc / mmio_*_helper /
  device read/write 共用编码; 长跳 vs 返回机制本质规则
- **§10 helper 颗粒度 + may-longjmp 边界 + JIT 寄存器保存** —— mmu_walker_helper_*
  / lsu_*_helper / amo_*_helper 各自入口 (颗粒度 = 单条指令, 不合并); JIT translator
  emit 每个 may-longjmp call 前 store 映射 host 寄存器到 cpu_t
- **§11 predicate is_* 命名** —— 返回布尔语义的查询函数统一 `is_` 前缀
- **§12 线程 lifecycle —— 谁 spawn 谁 join** —— spawn / join 成对暴露,由创建方
  显式调;`*_destroy` 纯 cleanup 不含 `pthread_join`
- **§13 类型规约 typedef family** —— `uxlen_t` / `ixlen_t`(XLEN-tied)+ `u32_t` /
  `u64_t`(spec-pinned);RV64 切换 grep trail,非运行期 XLEN 抽象


### 核心 (`src/core/`)


#### cpu.{c,h}

单 hart guest CPU 状态(`cpu_t`)。`regs[32]` 单连续数组,`regs[0]` 物理位置存
pc(x0 走特殊路径不碰)。`tlb_table[4]` 4 槽派发数组(详见 [TLB 拓扑](#tlb-拓扑))。

`per_hart_info`(嵌入)/ `shared_info`(指针指向 cpu.c 内 `static const`)拆分体现
SMP-ready —— 异构 SMP(1×MU + 4×MSU)时 `misa / mhartid` per-hart 不同,
`mvendorid / marchid / mimpid` 全机器共享。


#### decode.{c,h}

RV32I + RVC 指令解码,纯函数(不读 / 不写 `cpu_t`),给 interpreter / 未来 translator
共用。49 个 op_kind(RVC 复用同源 RV32I op,`pc_step` 区分长度)。

`is_block_boundary_inst` 共享 inline,让 interpreter 跟未来 translator 对硬边界
判断 100% 一致(`-Wswitch-enum -Werror` 强制全覆盖)。


#### interpreter.{c,h}

解释器主体。`interpret_one_block` 顺序 fetch + decode + switch 执行,直到五种退出
条件:`OP_UNSUPPORTED` / 硬边界 / 异常 longjmp / `BLOCK_INST_LIMIT` 失控保护 /
跨页软边界。

PC 维护数据驱动(decode 一次决定 `pc_step`,fetch loop 末段统一推进);控制流 case
自描述 pc 走 `WRITE_PC_OR_TRAP` 宏含 IALIGN 对齐检查 + trap 占位。

count 同步契约:may-trap 路径 case 内同步在 trap_raise 之前;boundary 路径走 out
段托底同步;pure case(纯算术 / 逻辑)不需同步,跟 RV precise trap 一致(trap
触发指令本身不算入 count)。


#### dispatcher.{c,h}

hart 主循环。形态:**sigsetjmp 一次性永久落点 + while(in_trap<3) 多块循环 +
迭代头扫尾**。每轮 3 block:

- **block 1** —— 按 `priv + xatp.mode` 算派发包 `(regime, current_tlb)`
- **block 2** —— `mmu_translate_pc` 取指
- **block 3** —— `interpret_one_block`(未来 `jit_cache_hit` 优先)

helper 端 longjmp 跳回 sigsetjmp 落点会跳过迭代尾,所以扫尾(count 累加 / 未来
mtime 推进 / 中断检查 / `perf_advance`)必须搬到迭代头 —— 跟正常 continue 路径
同形态。退出条件由 `in_trap` 位段编码控制(详见 [in_trap 位段编码](#in_trap-位段编码))。


#### csr.{c,h}

CSR 大 switch + 各 r/w file-static helper。`csr_op` 是 6 csr 指令变体(CSRRW/RS/RC
+ 3 I 变体)统一入口。

入口判利用 `csr_addr` 自带的权限位段:`bits[11:10]` = RO / `bits[9:8]` = 最低 priv
要求,失败统一 trap cause 2。

5 类组织哲学:类 1 扩展 CSR(F/V/Debug)未来归 `isa/`;类 2 跨模块 CSR(`satp`)
字段在 `cpu_t`,函数留 csr.c;类 3 核心 CSR 字段在 `trap_csrs_t`;类 4 出场信息
RO CSR 拆 per-hart + shared;类 5 临时调试 CSR(`tohost / privrd`)流式输出,
uart 实装后整段删。


#### trap.{c,h}

trap 系统两层分工。架构语义层不长跳:`trap_set_exception_state`(同步路径 —— 写
xcause/xtval/xepc + 切 priv mstatus 字段 + `regs[0] = xtvec`,medeleg 分流)、
`trap_set_interrupt_state`(异步路径 —— mideleg 分流 + vectored mode base+4*cause)。
控制流层:`trap_raise_exception` `_Noreturn` 长跳(供 helper 深栈用,内部
`trap_set_exception_state + siglongjmp` 跳回 dispatcher 一次性落点);
`trap_check_interrupt` return-based(dispatcher 主帧每轮 polling,不长跳)。
exception 长跳 / interrupt 返回的形态不对偶是 by design(dummy.txt §1 路径 D)。

`deliver_priv` 按 medeleg 真生效(M-mode trap 总 M;U/S-mode + bit=1 → S)。
`in_trap >= 3` 时不 deliver,字段保留第二次状态作 root cause 给 main dump。


#### mmu.{c,h}

Sv32 MMU walker + 各 walker helper。**回归 dummy.txt §8 三层模型** —— mmu 只翻译
+ 分流,不亲自做"真访问":RAM 路径让 walker fill TLB + caller 调 RAM 直接 *hva
(load) / store_helper (store);MMIO 路径直接调 `mmio_*_helper` (不入 TLB)。

执行 regime 二分(项目内部"两套硬件逻辑"分类):**REGIME_BARE**(`priv == M` 或
任何 priv 带 bare satp,bypass TLB identity)/ **REGIME_SV32**(`tlb_table[priv][asid]`
叶 TLB + PTE 权限语义)。接口层简化,下游 `mmu_translate_pc / interpret_one_block /
lsu_*_helper` 只吃 `current_tlb`(NULL 编码 BARE)。

`check_perm`(`mmu.h` `static inline`)三处共用(取指 / load / store),跟 walker
同源不会两份维护偏离。**hw-managed A/D**(非 Svade):walker 内**只算**建议 set 后
的 new_pte 通过出参回给 caller,caller 在 RAM 路径才真 memcpy 写回 PT;MMIO 路径
不写回 (跟 Spike "fail 不 set" 一致简化版)。TLB 不存 PTE 物理地址,store 命中 D=0
时 fall back walker 重 set。

`mmu_walker_helper_*` 一族 (load/store + 未来 amo_lr/sc/amo_*) 是 **JIT 颗粒度 by
design** —— 每条访问指令对应一个 helper 入口, JIT 不合并; 详 dummy.txt §10。


#### tlb.{c,h}

TLB 数据结构(`tlb_e_t` 16 B,字段位置对齐 RV PTE)+ `tlb_alloc` / `tlb_clear`
两个函数。`tlb_table[4]` 4 槽派发(详见 [TLB 拓扑](#tlb-拓扑))。

fast path 不暴露 `tlb_lookup` 函数 —— 命中由调用方 inline(V + tag + `check_perm`),
miss 才调 walker helper。`tlb_clear(NULL)` C 标准 no-op,sfence helper 不需要
null-check。


### ISA 实现 (`src/isa/`)


#### sfence.{c,h}

`sfence_vma_helper`(extern,slow path)。RV spec 4 组合简化方案 4.a:`rs1=x0 +
rs2!=x0` 单 ASID 清,其他三组合都全清(过度刷新 RV spec 允许;sfence 不是 hot
path 不亏)。

接口分两组传 `(vaddr_val, asid_val)` + `(rs1, rs2)` —— RV spec 用 `rs1=x0` 这个
**magic 编码**标"忽略 vaddr",但 `vaddr_val=0` 也是合法实值,helper 只看寄存器值
看不出区别,必须看寄存器编号。


#### lsu.{c,h}

load / store 不对称真机理 (dummy.txt §1 末段)。当前形态:

- `lsu_load_helper` / `lsu_store_helper` (inline 顶层在 `lsu.h`, interpreter 直调):
  BARE 内联 RAM/MMIO 分流 / SV32 TLB hit 直走 (load *hva, store 调 store_helper) /
  SV32 miss fall back `mmu_walker_helper_*`
- `store_helper` (extern 在 `lsu.c`, **HVA-based**): RAM 写 + LR/SC reservation 清除
  占位 + 未来 SMC 副作用入口; 三处 caller 共用一份 (BARE/SV32 hit / walker_helper
  RAM 路径)
- MMIO 路径不经 store_helper, 直接 `mmio_*_helper` (跳过 reservation+SMC; 因 MMIO
  不参与)
- `LOAD_MISALIGN_CHECK` / `STORE_MISALIGN_CHECK` 隐式契约: caller (interpreter case
  入口) 一处做, 所有 helper 信任 caller 已查

未来改进 `store_helper` inline 化(命名澄清:**不是**"store 变 fast path",内部
所有操作仍是 slow path 性质,只是链接形态从 extern 变 inline,消除每次 store 的
函数 call/ret 开销)。


### 平台 (`src/platform/`)


#### ram.{c,h}

host mmap 管理。`host_ram_base` + `gpa_to_hva_offset = host_ram_base -
GUEST_RAM_START` 两个全局,fast path 一次加法 gpa → hva 不用 cast。

`IS_GPA_RAM(pa)` 宏 (无符号下溢 idiom) 统一封装"PA 在 RAM 区"判断, 7 处 callsite
(mmu / lsu) 共用。

`MAP_NORESERVE` 不预扣 swap commit charge;`MADV_NOHUGEPAGE` 显式排 transparent
huge page 保 4 KB 颗粒度服务未来 SMC 检测(`smc.c` 的 `page_dirty` bitmap 按
4 KB 索引)。`mmap(NULL, ...)` 内核分配地址 4 KB 对齐,interpreter 跨页保护依赖
此 invariant。


#### bus.{c,h}

MMIO 注册表 + 派发。`mmio_dev_t` 结构 (gpa_start / gpa_end / ctx / read / write / name);
`bus_register_mmio` 注册 + 半开区间重叠校验; `mmio_read_helper` / `mmio_write_helper`
线性扫表派发 (BUS_MAX_DEV=16 静态数组)。

接口形态: **_Noreturn-on-failure** —— bus 失败 (未命中 / device 拒绝) 内部 trap_raise
longjmp 不返回 caller; device read/write fn 返 cause (0=成功 / 非0=cause), bus 透传给
trap_raise (跟 mmu_translate_pc 同形态)。详 dummy.txt §8 + §9。

`mmio_dev_t` struct **不含 R/W/X/execute / tick / has_pending_irq 字段** —— 三件事
分属独立维度 (PMP / 物理可 fetch / device 副作用), 见 dummy.txt §8。

未来扩展占位 (bus.h 顶部注释): (a) 可 fetch 范围表 (ROM/flash 启用时); (b) device
unregister + 热插拔 (RCU/atomic pointer swap)。


#### clint.{c,h}

CLINT (Core-Local Interruptor) MMIO 设备 —— mtime / mtimecmp[N] / msip[N], 全部
`_Atomic` (满足 dummy.txt §7 shared 字段一律 atomic; 单 hart 编译为 plain load/store,
零开销)。布局跟 SiFive CLINT + QEMU virt 一致 (CLINT_BASE=0x02000000, mtimecmp
@+0x4000 per-hart, mtime @+0xBFF8 global)。

`mtime` 由 file-static 的 timer 辅助线程异步推进(clock_nanosleep ABSTIME 每
~1 ms 唤醒一次 atomic_fetch_add TIMEBASE_PER_WAKE;方案 C)。clint 是项目内第一个
完整 monitor 实例:三函数 lifecycle(init / reset / destroy)+ spawn/join 对偶
(clint_start_timer_thread / clint_join_timer_thread)。`size != 4` / `off & 3 != 0`
/ offset 越界 都 access fault。



## 当前已落地

- 完整 RV32 IM + RVC 指令集(49 个 op_kind,RVC 复用)
- M / S 模式 CSR(mstatus 物理 64 位拆访问 / sstatus masked view / medeleg-driven
  trap delegation;mip/mie/sip/sie 中断 CSR —— `_mip_sw` 软件可写子集 + 异步源
  合成读、`_mie` masked view;csr_op 入口判 priv + RO 写)
- Sv32 MMU walker(含 hw-managed A/D,4 KB + 4 MB superpage; **walker 不真写回 PT,
  caller 在 RAM 路径写回; MMIO 路径不写回**, 跟 Spike "fail 不 set" 一致)
- mmu / lsu / bus 三层模型 (dummy.txt §8) —— mmu 只翻译 + 分流, lsu 是 PA 后真访问,
  bus 是 MMIO 派发; **mmu_walker_helper_\*** 一族 JIT 颗粒度 by design (§10)
- TLB 4 槽 ASID 二级索引(per-priv 隔离 / U 副本 / sfence.vma 全清 / 单 ASID 清);
  **MMIO 不进 TLB** 让命中路径结构不带分支
- LSU (lsu_load/store_helper inline 顶层 + store_helper HVA-based extern; LOAD/STORE
  MISALIGN_CHECK 隐式契约; SUM/MXR perm check 三处共用)
- trap 系统 —— exception:medeleg-driven `deliver_priv` / mret/sret 真切 priv
  mstatus 字段 / **PRIV_CHECK_OR_TRAP** 在 MRET/SRET/SFENCE.VMA 入口判 priv;
  interrupt:`trap_check_interrupt` 接入 dispatcher loop 顶 + mideleg 分流 +
  vectored mode(mtvec/stvec mode=1 真接,base+4*cause)
- dispatcher(sigsetjmp 永久落点 + while(in_trap<3) + 迭代头扫尾 + count 同步契约;
  **循环顶 pc IALIGN 兜底** single source 捕所有写 pc 路径错误, dummy.txt §9)
- in_trap 位段编码(bit 0-1 trap 嵌套 / bit 3 内部停机 / 高位预留)
- 跨页保护(interpreter 推进后 hva_pc 跨 4 KB → 退出 block,dispatcher 重派发)
- **bus + CLINT MMIO** (mmio_dev_t 注册表 + 线性派发; CLINT mtime/mtimecmp/msip
  `_Atomic`, 布局跟 QEMU virt + SiFive CLINT 一致)
- timer 辅助线程异步推进 atomic `mtime`(clock_nanosleep ABSTIME,方案 C)+ 三层
  reset lifecycle + runtime degenerate monitor(SRS/SDS 两 atomic flag)+ 谁 spawn
  谁 join 协议
- 寄存器宽度 typedef family(`uxlen_t` / `ixlen_t` / `u32_t` / `u64_t`;RV64 切换
  grep trail,dummy.txt §13)
- debug 字符 trace (`_` refetch / `E` exception / `t/s/e` time/soft/ext intr;
  stderr 上, DEBUG_TICK 阈值 80 自动换行)

未实现(后续 milestone):JIT 子系统(Translator / IR / jit_cache / code_cache /
SMC,a_05+)/ PLIC / UART / virtio-blk / boot ROM(a_03+)/ A 扩展 LR/SC/AMO /
F/D 扩展浮点 / a_02_end 端到端 hello world(依赖 a_03 的 UART + PLIC)/ 长期
TODO:Ctrl+C 优雅中断、mstatus.TSR/TVM 控制位 trap、WFI 实装。


## 构建 + 运行

依赖:

- CMake ≥ 3.10
- gcc / clang(host 编译; AddressSanitizer Debug 默认开)
- riscv64-unknown-elf-gcc(跨编 fixture; `-march=rv32imac -mabi=ilp32`)

构建:

```bash
cmake -B cmake-build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build cmake-build-debug
```

跑 fixture:

```bash
make -C tests                                                # 重建所有 fixture .bin/.elf
./cmake-build-debug/jit-emu tests/a_01/a01_3/01_arith_basic/out.bin
```

CLion Debug 下需要环境变量 `ASAN_OPTIONS=abort_on_error=1:detect_leaks=0` —— LSan 撞
gdb ptrace 会 fatal exit 1,看似程序 bug 实际是工具链限制;Run 模式(无 gdb)不需要。


## 测试组织

每个 fixture 自包含一个目录,命名 `NN_descriptive_name[_reject]/`:

```
tests/a_01/a01_<N>/<NN>_<name>/
  stub.S       (RV32 汇编源,部分 fixture 配 main.c)
  Makefile     (riscv64-unknown-elf-gcc 跨编)
  link.ld      (linker script,起点 0x80000000)
```

`_reject` 后缀标负面测试(验 loader 验证分支真打到)。每个 sub-milestone 至少一个
fixture,累积成回归集。


## 许可

待定。
