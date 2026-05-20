#!/usr/bin/env python3
# tests/review/run_tests.py —— 跑批所有 tests/ fixture, 结果落 out/tests_res/。
#
# 用法:  python3 tests/review/run_tests.py
# 依赖:  tests/review/out/riscv_jit_emulator —— 冻结的二进制副本; 每次整理前自己
#        从 cmake-build-debug 复制一份过来 (与并行的 src 改动隔离)。
#        各 fixture 的 out.bin / out.elf 已 make 好 (先在 tests/ 跑一次 make)。
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
EMU = os.path.join(OUT, "riscv_jit_emulator")
RESDIR = os.path.join(OUT, "tests_res")
TIMEOUT = 3.0  # 秒; 暂定值, 长跑 fixture (perf) debug build 约 0.1-0.4s, 富余

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
    art = artifact_for(test_dir)
    out_dir = os.path.join(RESDIR, rel)
    os.makedirs(out_dir, exist_ok=True)
    log = os.path.join(out_dir, "out.log")

    if art is None:
        with open(log, "w") as f:
            f.write("NO ARTIFACT (no out.bin / out.elf found)\n")
        return (rel, "NO-ARTIFACT", None, 0.0, False)

    art_rel = os.path.relpath(art, REPO)
    cmd = [EMU, art_rel]
    t0 = time.monotonic()
    timed_out = False
    try:
        p = subprocess.run(cmd, cwd=REPO, capture_output=True,
                           timeout=TIMEOUT, start_new_session=True)
        rc = p.returncode
        so, se = p.stdout, p.stderr
    except subprocess.TimeoutExpired as e:
        timed_out = True
        rc = None
        so = e.stdout or b""
        se = e.stderr or b""
    dt = time.monotonic() - t0

    with open(log, "wb") as f:
        head = (f"cmd       : {' '.join(cmd)}\n"
                f"artifact  : {art_rel}\n"
                f"exit code : {'TIMEOUT' if timed_out else rc}\n"
                f"wall time : {dt:.3f} s\n"
                f"timed_out : {timed_out}\n"
                f"{'=' * 60}\n===STDOUT===\n").encode()
        f.write(head)
        f.write(so)
        f.write(b"\n===STDERR===\n")
        f.write(se)

    status = "TIMEOUT" if timed_out else f"exit={rc}"
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
        print(f"  {status:10s} {dt:6.3f}s  {rel}")
    os.makedirs(RESDIR, exist_ok=True)
    with open(os.path.join(RESDIR, "_summary.txt"), "w") as f:
        f.write(f"fixtures: {len(rows)}   timeout: {TIMEOUT}s\n")
        f.write("=" * 70 + "\n")
        for rel, status, dt, to in rows:
            f.write(f"{status:10s} {dt:7.3f}s  {rel}\n")
    n_to = sum(1 for r in rows if r[3])
    print(f"\ndone: {len(rows)} fixtures, {n_to} timed out")
    print(f"results -> {RESDIR}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
