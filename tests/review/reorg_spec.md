# stub.S 注释规范(四段)

每个 `tests/.../stub.S` 的注释按四段组织(运行参数 / 测试目的 / 思路 / 期望结果)。
整理 / 新写 fixture 时照此;milestone 收尾的测试整理 pass 也按此重排存量 stub.S。
「运行参数」是 a_03 收尾新加的第四段(顶端),其余三段不变。

## banner 结构(逐字照抄)
```
# <第一行原样保留, 形如  # a_01_5 / 04_trap_chain>
#
# ===== 运行参数 =====
# <RUN-* tag, 见下节。可选段: fixture 用默认跑法 (无 stdin / 无 blk / 不断言
#  exit) 就整段省略 (连 banner 一起)。>
#
# ===== 测试目的 =====
# <测试目的: 验证什么、指令数那一行。简洁。>
#
# ===== 思路 =====
# <设计 / 思路 / 伪代码 / 控制流追 / dispatcher 行为 / block-by-block 推导。
#  可选段: 没有这类内容就整段省略 (连 banner 一起)。>

.section .text
... 代码区 ...
<最后一条指令 / 标签>

# ===== 期望结果 =====
# <只放最终可观测状态清单: 寄存器值 / trap dump 字段 / pc / in_trap /
#  total_count / 关键 stderr 行 / 停机方式。要紧凑。
#  可选段: 确实没有期望状态内容就省略 (连 banner 一起)。>
```

## 运行参数段(RUN-* tag schema)

`tests/review/run_tests.py` grep stub.S 注释里形如 `# RUN-XXX: value` 的行决定跑法
(方案 A explicit declaration; 撤了"自动检测后缀" + "集中 JSONL")。解析器只认 `RUN-`
行、不依赖位置;banner 把它们收在「运行参数」段纯属人类组织约定。

- `# RUN-BLK: <file>`        —— 追加 `--blk <file>`(file 相对 fixture 目录)。
- `# RUN-STDIN: <file>`      —— 读文件字节喂子进程 stdin(延迟 `STDIN_DELAY` 再投,
  避 reader 线程 vs guest 设备 init 的启动竞态 —— pipe 瞬时投递会让 RX 字节早于
  guest 配好中断使能而丢边沿;EOF 自然触发 reader 退出)。
- `# RUN-EXPECT-EXIT: <int>` —— opt-in 断言: 比对实际 exit code, 出 PASS / FAIL。
- `# RUN-RELEASE`            —— 布尔裸 tag, 用 release 冻结二进制跑(debug 太慢/
  必须 release 的, 如 a02_7 perf 套件; 正确性批跑到完整作冒烟, 测速归 run_perf.py)。
- `# RUN-TIMEOUT: <秒>`      —— 覆盖默认 timeout(默认 3.0); 跟 RUN-RELEASE 正交可叠加。

默认(无 tag)= `[EMU(debug), --bios, <out.bin/out.elf>]`,无 stdin / 无 blk / exit
仅记录不断言 / timeout=3.0。tag 残缺(未知名 / value tag 缺值 / 布尔 tag 带值 /
文件不存在 / 数值解析失败 / RUN-RELEASE 但 release 二进制缺失)→ fail loud: 该
fixture 标 CONFIG-ERROR 跳过,不污染其他。

run_tests.py 用两个冻结二进制: `out/riscv_jit_emulator`(debug+ASan, 默认)+
`out/riscv_jit_emulator_release`(RUN-RELEASE 用); 整理前各从 cmake-build-debug /
cmake-build-release 复制一份。RUN-BLK / RUN-STDIN 指向的附属文件(disk.img /
input.txt)由 fixture 自己的 Makefile 生成 + 入库(跟 disk.img 既有体例一致),
`make clean` 清、`make` 重生。

划分原则:
- block-by-block 的「控制流追」/「dispatcher 行为」/ 逐块推导 **不是结果**,是推导
  过程 → 放「思路」段,**绝不**放「期望结果」。
- 「期望结果」段只放那张"一拉到底就能扫到的最终结果表"。

## 重排存量 stub.S 时的硬规则
- 只动注释。代码区(从 `.section` 到最后一条指令/标签,含每条指令后面的行内 `#`
  注释)必须**逐字节不变**。
- 注释散文:只在三段之间**搬移**已有注释行 + 加 banner 行。不要改写、缩写、翻译、
  精简、"优化"散文。所有技术内容逐字保留。
- 保留所有"陈旧 trail 注释"(以 `注:` 开头、或提到 未来/过时/trail/实验记录 等的
  行/块)——绝不删除、绝不"修正"。陈旧注释若关于期望放「期望结果」,关于设计放「思路」。
- 改完用 `out.bin` md5 校验代码区零改动(`out.elf` 构建非确定性,不能用作基准)。

## 期望比对(测试整理 pass 用)
对每个测试,读 `tests/review/out/tests_res/<相对 tests/ 的路径>/out.log`(模拟器实跑
输出),跟 stub.S 底部「期望结果」比对,分四类:符合 / 部分演进 / 不符·已演进 /
无显式期望。distilled 结果(演进根因 + 待校准清单)写入 `tests/review/REVIEW.md`。
