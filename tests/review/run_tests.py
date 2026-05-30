#!/usr/bin/env python3
# tests/review/run_tests.py —— 跑批所有 tests/ fixture, 结果落 out/tests_res/。
#
# 用法:  python3 tests/review/run_tests.py
# 依赖:  tests/review/out/riscv_jit_emulator         —— debug+ASan 冻结副本 (默认)
#        tests/review/out/riscv_jit_emulator_release —— release 冻结副本 (RUN-RELEASE
#                                                       fixture 用; 无 RUN-RELEASE 则可缺)
#        每次整理前自己从 cmake-build-debug / cmake-build-release 各复制一份过来
#        (与并行的 src 改动隔离)。各 fixture 的 out.bin / out.elf / disk.img /
#        input.txt 已 make 好 (先在 tests/ 跑一次 make)。
#
# 运行接口 (新, a_03 收尾): base cmd = [EMU, '--bios', artifact]。需要 stdin / 块
# 设备 / exit 断言 / release / 自定 timeout 的 fixture 在 stub.S banner 内 explicit
# declaration (方案 A), 解析 `# RUN-*` 注释行覆盖默认 (schema 见 reorg_spec.md
# "运行参数" 段):
#   # RUN-BLK: disk.img          → 追加 --blk <disk.img> (相对 fixture 目录)
#   # RUN-STDIN: input.txt       → 读文件字节喂子进程 stdin (延迟 STDIN_DELAY 投,
#                                  避 reader-vs-init 启动竞态; EOF 自然触发 reader 退)
#   # RUN-EXPECT-EXIT: 42        → opt-in 断言, 比对实际 exit code 出 PASS/FAIL
#   # RUN-RELEASE                → 用 release 冻结二进制 (debug 太慢/必须 release)
#   # RUN-TIMEOUT: 5             → 覆盖默认 timeout (秒); 不写默认 3.0
#   # RUN-SMP: 2                 → 追加 --smp N (1..8; 多 hart fixture 用)
# 没声明 tag → sensible default (debug 二进制 / 无 stdin / 无 blk / exit 仅记录
# 不断言 / timeout=3.0)。tag 残缺 (未知名 / 缺值 / 布尔带值 / 文件不存在 / 数值
# 解析失败 / RUN-RELEASE 但 release 二进制缺失) → fail loud: 该 fixture 标
# CONFIG-ERROR 跳过, 不污染其他 fixture。
#
# 每个 fixture 产出 tests/review/out/tests_res/<relpath>/out.log:
#   头部 = cmd / artifact / exit code / wall time / timed_out
#   接 ===STDOUT=== / ===STDERR=== 两段原始输出。
# 另产出 tests/review/out/tests_res/_summary.txt: 一行一个 fixture 的汇总。
#
# 本文件 + reorg_spec.md + REVIEW.md 跟踪进 repo; out/ 是一次性产物, gitignore。

import os
import re
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))   # .../tests/review
TESTS = os.path.dirname(SCRIPT_DIR)                       # .../tests
REPO = os.path.dirname(TESTS)                             # repo 根
OUT = os.path.join(SCRIPT_DIR, "out")
EMU = os.path.join(OUT, "riscv_jit_emulator")               # 默认 = debug + ASan
EMU_RELEASE = os.path.join(OUT, "riscv_jit_emulator_release")  # RUN-RELEASE 用
RESDIR = os.path.join(OUT, "tests_res")
TIMEOUT = 3.0  # 秒; 默认值, 单 fixture 可用 RUN-TIMEOUT 覆盖

# RUN-STDIN 投递前的延迟 (秒)。pipe 瞬时投递会让 RX 字节早于 guest 跑完设备 init
# (PLIC source / UART ERBFI 使能), reader 线程推字节 vs guest 配中断的先后踩到丢
# 边沿 → hart 永久 spin。模拟器 init 极快 (<10ms), 给个小延迟让 init 先完成就稳过;
# 模拟器不慢, 这点延迟不脆。用 Popen 先 spawn → sleep → 再写 stdin 实现。
STDIN_DELAY = 0.1

# frozen 二进制通常是 debug + ASan 构建。批跑非 gdb 环境下 LSan 的 exit-time 扫描
# 会把正常停机 (有意未释放的 mmap/线程资源) 报成泄漏 → 非 0 退出, 污染 PASS/FAIL
# 与 exit 断言; detect_leaks=0 关掉它 (跟 CLAUDE.md / CLion run config 体例一致)。
CHILD_ENV = dict(os.environ, ASAN_OPTIONS="detect_leaks=0:abort_on_error=1")

# stub.S banner 内的运行参数 tag (方案 A explicit declaration)。
# 两种形态: 带值 `# RUN-XXX: value` (group val) 或布尔裸 `# RUN-XXX` (val=None)。
RUN_TAG_RE = re.compile(r'^\s*#\s*(RUN-[A-Z-]+)\s*(?::\s*(\S+)\s*)?$')
VALUE_TAGS = ("RUN-BLK", "RUN-STDIN", "RUN-EXPECT-EXIT", "RUN-TIMEOUT", "RUN-SMP")
BOOL_TAGS = ("RUN-RELEASE",)
KNOWN_TAGS = VALUE_TAGS + BOOL_TAGS

class ConfigError(Exception):
    """fixture 运行参数声明残缺 —— fail loud, 跳过该 fixture 不污染其他。"""
    pass

def artifact_for(test_dir):
    """解析 fixture Makefile 的 all: 目标, 决定跑 out.bin 还是 out.elf。"""
    mk = os.path.join(test_dir, "Makefile")
    if os.path.isfile(mk):
        with open(mk, "r", errors="replace") as f:
            for line in f:
                m = re.match(r'\s*all\s*:\s*(\S+)', line)
                if m:
                    cand = os.path.join(test_dir, m.group(1))
                    if os.path.isfile(cand):
                        return cand
    # 回退: 优先 out.bin, 再 out.elf
    for name in ("out.bin", "out.elf"):
        cand = os.path.join(test_dir, name)
        if os.path.isfile(cand):
            return cand
    return None

def parse_run_config(test_dir):
    """grep stub.S 的 `# RUN-*` 注释行, 返回 cfg dict:
       {blk, stdin, expect_exit, release(bool), timeout}。
    没 stub.S / 没 tag → 全默认 (blk/stdin/expect_exit=None, release=False,
    timeout=TIMEOUT)。残缺 → raise ConfigError。路径值相对 fixture 目录解析;
    文件不存在即报错。"""
    cfg = {"blk": None, "stdin": None, "expect_exit": None,
           "release": False, "timeout": TIMEOUT, "smp": None}
    stub = os.path.join(test_dir, "stub.S")
    if not os.path.isfile(stub):
        return cfg
    with open(stub, "r", errors="replace") as f:
        for line in f:
            # 只看注释行里形如 `# RUN-XXX[: value]` 的; 其他注释/代码忽略。
            if "RUN-" not in line:
                continue
            m = RUN_TAG_RE.match(line)
            if not m:
                # 看着像 RUN- tag 但不合 schema → fail loud。
                if re.match(r'\s*#\s*RUN-', line):
                    raise ConfigError(f"malformed RUN tag: {line.strip()!r}")
                continue
            tag, val = m.group(1), m.group(2)
            if tag not in KNOWN_TAGS:
                raise ConfigError(f"unknown RUN tag: {tag}")
            if tag in VALUE_TAGS and val is None:
                raise ConfigError(f"{tag} needs a value")
            if tag in BOOL_TAGS and val is not None:
                raise ConfigError(f"{tag} takes no value (got {val!r})")
            if tag == "RUN-BLK":
                p = os.path.join(test_dir, val)
                if not os.path.isfile(p):
                    raise ConfigError(f"RUN-BLK file not found: {val}")
                cfg["blk"] = p
            elif tag == "RUN-STDIN":
                p = os.path.join(test_dir, val)
                if not os.path.isfile(p):
                    raise ConfigError(f"RUN-STDIN file not found: {val}")
                cfg["stdin"] = p
            elif tag == "RUN-EXPECT-EXIT":
                try:
                    cfg["expect_exit"] = int(val)
                except ValueError:
                    raise ConfigError(f"RUN-EXPECT-EXIT not an int: {val!r}")
            elif tag == "RUN-TIMEOUT":
                try:
                    cfg["timeout"] = float(val)
                except ValueError:
                    raise ConfigError(f"RUN-TIMEOUT not a number: {val!r}")
            elif tag == "RUN-RELEASE":
                cfg["release"] = True
            elif tag == "RUN-SMP":
                try:
                    n = int(val)
                except ValueError:
                    raise ConfigError(f"RUN-SMP not an int: {val!r}")
                if n < 1 or n > 8:
                    raise ConfigError(f"RUN-SMP out of range 1..8: {n}")
                cfg["smp"] = n
    return cfg

def discover():
    """tests/<milestone>/<sub>/<NN_name>/ —— 带 Makefile 的叶子目录。"""
    dirs = []
    for ms in sorted(os.listdir(TESTS)):
        ms_p = os.path.join(TESTS, ms)
        if not os.path.isdir(ms_p) or ms == "review":
            continue
        for sub in sorted(os.listdir(ms_p)):
            sub_p = os.path.join(ms_p, sub)
            if not os.path.isdir(sub_p):
                continue
            for nn in sorted(os.listdir(sub_p)):
                nn_p = os.path.join(sub_p, nn)
                if os.path.isdir(nn_p) and os.path.isfile(os.path.join(nn_p, "Makefile")):
                    dirs.append(nn_p)
    return dirs

def run_one(test_dir):
    rel = os.path.relpath(test_dir, TESTS)
    out_dir = os.path.join(RESDIR, rel)
    os.makedirs(out_dir, exist_ok=True)
    log = os.path.join(out_dir, "out.log")

    art = artifact_for(test_dir)
    if art is None:
        with open(log, "w") as f:
            f.write("NO ARTIFACT (no out.bin / out.elf found)\n")
        return (rel, "NO-ARTIFACT", None, 0.0, False)

    # 运行参数解析 —— fail loud: 残缺即跳过, 不带半套配置去跑。
    try:
        cfg = parse_run_config(test_dir)
    except ConfigError as e:
        with open(log, "w") as f:
            f.write(f"CONFIG ERROR: {e}\n")
        return (rel, "CONFIG-ERROR", None, 0.0, False)

    # RUN-RELEASE → 换 release frozen 二进制 (a02_7 perf 这类 debug 太慢/必须
    # release 跑的)。release 二进制缺失却被声明 → fail loud。
    emu = EMU
    if cfg["release"]:
        if not os.path.isfile(EMU_RELEASE):
            with open(log, "w") as f:
                f.write(f"CONFIG ERROR: RUN-RELEASE but {EMU_RELEASE} missing\n")
            return (rel, "CONFIG-ERROR", None, 0.0, False)
        emu = EMU_RELEASE

    art_rel = os.path.relpath(art, REPO)
    cmd = [emu, "--bios", art_rel]
    if cfg["blk"] is not None:
        cmd += ["--blk", os.path.relpath(cfg["blk"], REPO)]
    if cfg["smp"] is not None:
        cmd += ["--smp", str(cfg["smp"])]

    stdin_bytes = None
    if cfg["stdin"] is not None:
        with open(cfg["stdin"], "rb") as f:
            stdin_bytes = f.read()

    t0 = time.monotonic()
    timed_out = False
    if stdin_bytes is None:
        # 无 stdin: 直跑。
        try:
            p = subprocess.run(cmd, cwd=REPO, capture_output=True,
                               timeout=cfg["timeout"], start_new_session=True,
                               env=CHILD_ENV)
            rc, so, se = p.returncode, p.stdout, p.stderr
        except subprocess.TimeoutExpired as e:
            timed_out = True
            rc = None
            so = e.stdout or b""
            se = e.stderr or b""
    else:
        # 有 stdin: 先 spawn, 延迟 STDIN_DELAY 让 guest 跑完设备 init, 再投字节
        # (避 reader-vs-init 启动竞态; 见 STDIN_DELAY 注释)。
        proc = subprocess.Popen(cmd, cwd=REPO, stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                start_new_session=True, env=CHILD_ENV)
        time.sleep(STDIN_DELAY)
        try:
            so, se = proc.communicate(input=stdin_bytes, timeout=cfg["timeout"])
            rc = proc.returncode
        except subprocess.TimeoutExpired:
            timed_out = True
            rc = None
            proc.kill()
            so, se = proc.communicate()   # 收尾捞已产出的输出
    dt = time.monotonic() - t0

    with open(log, "wb") as f:
        head = (f"cmd       : {' '.join(cmd)}\n"
                f"artifact  : {art_rel}\n"
                f"build     : {'release' if cfg['release'] else 'debug'}\n"
                f"stdin     : {os.path.relpath(cfg['stdin'], REPO) if cfg['stdin'] else '-'}\n"
                f"expect_exit: {cfg['expect_exit'] if cfg['expect_exit'] is not None else '-'}\n"
                f"timeout   : {cfg['timeout']} s\n"
                f"exit code : {'TIMEOUT' if timed_out else rc}\n"
                f"wall time : {dt:.3f} s\n"
                f"timed_out : {timed_out}\n"
                f"{'=' * 60}\n===STDOUT===\n").encode()
        f.write(head)
        f.write(so)
        f.write(b"\n===STDERR===\n")
        f.write(se)

    if timed_out:
        status = "TIMEOUT"
    elif cfg["expect_exit"] is not None:
        status = "PASS" if rc == cfg["expect_exit"] else \
                 f"FAIL(exit={rc} want={cfg['expect_exit']})"
    else:
        status = f"exit={rc}"
    return (rel, status, rc, dt, timed_out)

def main():
    if not os.path.isfile(EMU):
        print(f"ERROR: emulator not found at {EMU}", file=sys.stderr)
        print("  先从 cmake-build-debug/ 复制一份冻结二进制过来。", file=sys.stderr)
        return 1
    dirs = discover()
    print(f"discovered {len(dirs)} fixtures; timeout={TIMEOUT}s\n")
    rows = []
    for d in dirs:
        rel, status, rc, dt, to = run_one(d)
        rows.append((rel, status, dt, to))
        print(f"  {status:24s} {dt:6.3f}s  {rel}")
    os.makedirs(RESDIR, exist_ok=True)
    with open(os.path.join(RESDIR, "_summary.txt"), "w") as f:
        f.write(f"fixtures: {len(rows)}   timeout: {TIMEOUT}s\n")
        f.write("=" * 70 + "\n")
        for rel, status, dt, to in rows:
            f.write(f"{status:24s} {dt:7.3f}s  {rel}\n")
    n_to = sum(1 for r in rows if r[3])
    n_fail = sum(1 for r in rows if r[1].startswith("FAIL"))
    n_cfg = sum(1 for r in rows if r[1] == "CONFIG-ERROR")
    print(f"\ndone: {len(rows)} fixtures, {n_to} timed out, "
          f"{n_fail} FAIL, {n_cfg} config-error")
    print(f"results -> {RESDIR}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
