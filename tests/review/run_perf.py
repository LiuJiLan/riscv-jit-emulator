#!/usr/bin/env python3
# tests/review/run_perf.py —— perf 套件跑批: 每个 fixture 连跑 N 次, 解析 dispatcher
# [perf] 行, 报 median / min / max MIPS。
#
# 用法:  python3 tests/review/run_perf.py [N] [fixture-subpath]
#          N              连跑次数 (默认 5)
#          fixture-subpath 相对 tests/ 的子路径 (默认 a_02/a02_7 全套)
# 依赖:  cmake-build-release/riscv_jit_emulator (Release 构建, DEBUG_PERF_ON 开);
#        各 fixture 的 out.bin —— 本脚本每个 fixture 先 make 一遍再跑。
#
# perf 口径: 单 fixture ITER_COUNT 调到 Release ~1s, 连跑取 median —— 冷启动
# (TLB/cache 冷 + mmap RAM 首次触页) 摊薄到 <0.1%; 单跑更久反被 CPU turbo 降频 /
# VM steal time 拖乱。median 看典型值, min 最接近纯算力, max-min 是稳定性指标。
#
# 结果打印到 stdout + 落 tests/review/out/perf_res.txt (out/ 一次性产物, gitignore)。
# 本文件跟踪进 repo (跟 run_tests.py 同)。

import os, re, statistics, subprocess, sys, time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))   # .../tests/review
TESTS = os.path.dirname(SCRIPT_DIR)                        # .../tests
REPO = os.path.dirname(TESTS)                              # repo 根
EMU = os.path.join(REPO, "cmake-build-release", "riscv_jit_emulator")
OUT = os.path.join(SCRIPT_DIR, "out")
TIMEOUT = 30.0  # 秒/单跑; perf fixture Release ~1s, 富余防挂死

PERF_RE = re.compile(
    r'\[perf\]\s+elapsed=([\d.]+)\s*s\s+total_count=(\d+)\s+MIPS=([\d.]+)')

def discover(subpath):
    """收集 subpath 下带 Makefile 的叶子 fixture 目录。"""
    base = os.path.join(TESTS, subpath)
    if os.path.isfile(os.path.join(base, "Makefile")):
        return [base]
    dirs = []
    for root, _, files in os.walk(base):
        if "Makefile" in files:
            dirs.append(root)
    return sorted(dirs)

def run_once(out_bin):
    """跑一次, 解析 stderr 的 [perf] 行; 返 (elapsed, total_count, mips) 或 None。"""
    art_rel = os.path.relpath(out_bin, REPO)
    try:
        p = subprocess.run([EMU, art_rel], cwd=REPO, capture_output=True,
                           timeout=TIMEOUT, start_new_session=True)
    except subprocess.TimeoutExpired:
        return None
    m = PERF_RE.search(p.stderr.decode(errors="replace"))
    if not m:
        return None
    return (float(m.group(1)), int(m.group(2)), float(m.group(3)))

def main():
    runs = int(sys.argv[1]) if len(sys.argv) >= 2 else 5
    subpath = sys.argv[2] if len(sys.argv) >= 3 else os.path.join("a_02", "a02_7")

    if not os.path.isfile(EMU):
        print(f"ERROR: Release 构建不存在: {EMU}", file=sys.stderr)
        print("  先 cmake 编 cmake-build-release (见 memory feedback_build_via_gui)。",
              file=sys.stderr)
        return 1
    dirs = discover(subpath)
    if not dirs:
        print(f"ERROR: tests/{subpath} 下没找到 fixture", file=sys.stderr)
        return 1

    print(f"perf 跑批: {len(dirs)} fixture x {runs} 次  (EMU = Release build)\n")
    lines = []
    for d in dirs:
        rel = os.path.relpath(d, TESTS)
        if subprocess.run(["make"], cwd=d, capture_output=True).returncode != 0:
            print(f"  {rel:30s}  MAKE FAIL")
            lines.append(f"{rel}  MAKE FAIL")
            continue
        mips, els, cnt, bad = [], [], None, False
        for _ in range(runs):
            r = run_once(os.path.join(d, "out.bin"))
            if r is None:
                bad = True
                break
            els.append(r[0]); cnt = r[1]; mips.append(r[2])
        if bad:
            print(f"  {rel:30s}  RUN FAIL (无 [perf] 行 / timeout)")
            lines.append(f"{rel}  RUN FAIL")
            continue
        row = (f"{rel:30s}  median {statistics.median(mips):7.1f} MIPS  "
               f"(min {min(mips):.1f} / max {max(mips):.1f})  "
               f"elapsed~{statistics.median(els):.3f}s  count={cnt}")
        print("  " + row)
        lines.append(row + "\n    " + str(runs) + " 跑 MIPS: "
                     + " / ".join(f"{x:.1f}" for x in sorted(mips)))

    os.makedirs(OUT, exist_ok=True)
    res = os.path.join(OUT, "perf_res.txt")
    with open(res, "w") as f:
        f.write(f"perf 跑批  {time.strftime('%Y-%m-%d %H:%M:%S')}  "
                f"{len(dirs)} fixture x {runs} 次\n" + "=" * 72 + "\n")
        for ln in lines:
            f.write(ln + "\n")
    print(f"\n-> {res}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
