#!/usr/bin/env python3
# tests/review/run_perf.py —— perf 套件跑批: 每个 fixture 连跑 N 次, 解析 dispatcher
# [perf] 行, 报 median / min / max MIPS。
#
# 用法:
#   单 binary (原默认模式):
#     python3 tests/review/run_perf.py [N] [fixture-subpath]
#       N              连跑次数 (默认 5)
#       fixture-subpath 相对 tests/ 的子路径 (默认 a_02/a02_7 全套)
#
#   多 binary 对照模式 (--compare):
#     python3 tests/review/run_perf.py --compare BIN1,BIN2,BIN3 [N] [fixture-subpath]
#       BIN1,BIN2,...  逗号分隔 binary 绝对路径 (>=2 个); 各 binary 跑 a02_7 全套对照
#       N              每 binary 在单 fixture 上跑几轮 (默认 5); 单 fixture 上严格
#                       ABCABC... 交错紧贴 (跟 a_02_session_016 OFF/ON 同 fixture 紧贴
#                       一致, 抵消 CPU turbo / cache 热漂移)
#       fixture-subpath 同上 (默认 a_02/a02_7)
#     输出: per (binary, fixture) median + 末尾横向对照表 (BIN1 median / BIN2 median /
#           ... / BIN_i-vs-BIN1 %)
#
# 依赖:  cmake-build-release/riscv_jit_emulator (Release 构建, DEBUG_PERF_ON 开);
#        各 fixture 的 out.bin —— 本脚本每个 fixture 先 make 一遍再跑 (make 继承调用者
#        env, 用 gcc16 跑需调用前 export PATH=/opt/riscv-2026.05.19/bin:$PATH)。
#        --compare 模式下 EMU 由 BIN1..N 覆盖, cmake-build-release/ 不一定要存在。
#
# 运行接口 (a_03 收尾, 跟 run_tests.py 对齐): cmd = [EMU, '--bios', out.bin]。emulator
# 新 CLI 要求 --bios (旧 [EMU, artifact] 报 unknown arg)。perf 套件 a02_7 全是无设备/
# 无 stdin fixture, 不需 run_tests 的 RUN-* tag 解析; 恒走 release binary。
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
EMU_DEFAULT = os.path.join(REPO, "cmake-build-release", "riscv_jit_emulator")
OUT = os.path.join(SCRIPT_DIR, "out")
TIMEOUT = 30.0  # 秒/单跑; perf fixture Release ~1s, 富余防挂死

PERF_RE = re.compile(
    r'\[hart\d+\s+perf\]\s+elapsed=([\d.]+)\s*s\s+total_count=(\d+)\s+MIPS=([\d.]+)')

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

def run_once(emu, out_bin):
    """跑一次, 解析 stderr 的 [perf] 行; 返 (elapsed, total_count, mips) 或 None。"""
    art_rel = os.path.relpath(out_bin, REPO)
    # 接口跟 run_tests.py 对齐 (a_03 收尾): emulator 现要求 `--bios <artifact>` base,
    # 旧 `[emu, artifact]` 报 unknown arg。perf 套件 (a02_7) 全是 bare/mmu/mem fixture,
    # 无 stdin / 块设备 / exit 断言, 故不需 run_tests 的 RUN-* tag 解析; 恒用 release
    # binary (RUN-RELEASE 隐含)。ASAN_OPTIONS detect_leaks=0 跟 run_tests 同 (release
    # 一般无 ASan, 但带上无害, 防 frozen binary 偶含 sanitizer 时污染)。
    env = dict(os.environ, ASAN_OPTIONS="detect_leaks=0:abort_on_error=1")
    try:
        p = subprocess.run([emu, "--bios", art_rel], cwd=REPO, capture_output=True,
                           timeout=TIMEOUT, start_new_session=True, env=env)
    except subprocess.TimeoutExpired:
        return None
    m = PERF_RE.search(p.stderr.decode(errors="replace"))
    if not m:
        return None
    return (float(m.group(1)), int(m.group(2)), float(m.group(3)))

def run_single(runs, subpath):
    """原 single-binary 模式: 每 fixture 连跑 N 次, 取 median。"""
    if not os.path.isfile(EMU_DEFAULT):
        print(f"ERROR: Release 构建不存在: {EMU_DEFAULT}", file=sys.stderr)
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
            r = run_once(EMU_DEFAULT, os.path.join(d, "out.bin"))
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

def run_compare(emus, runs, subpath):
    """多 binary 对照模式: 单 fixture 上 ABCABC... 严格交错 N 轮, 每 (binary, fixture)
    取 median, 末尾汇横向对照表。emus = list of (label, abs_path)."""
    for label, emu in emus:
        if not os.path.isfile(emu):
            print(f"ERROR: binary 不存在: {label} = {emu}", file=sys.stderr)
            return 1
    dirs = discover(subpath)
    if not dirs:
        print(f"ERROR: tests/{subpath} 下没找到 fixture", file=sys.stderr)
        return 1

    labels = [e[0] for e in emus]
    print(f"perf 对照: {len(dirs)} fixture x {len(emus)} binary x {runs} 轮 "
          f"= {len(dirs) * len(emus) * runs} 跑")
    print(f"  binaries: " + " / ".join(f"{l}={p}" for l, p in emus))
    print(f"  交错形态: 单 fixture 内 {''.join(labels)}-{''.join(labels)}-... × {runs} 轮\n")

    # results[fixture_rel][label] = [mips, ...]
    results = {}
    per_fixture_lines = []
    for d in dirs:
        rel = os.path.relpath(d, TESTS)
        if subprocess.run(["make"], cwd=d, capture_output=True).returncode != 0:
            print(f"  {rel:30s}  MAKE FAIL")
            results[rel] = None
            continue
        out_bin = os.path.join(d, "out.bin")
        per_label = {l: [] for l in labels}
        bad = False
        for _round in range(runs):
            for label, emu in emus:
                r = run_once(emu, out_bin)
                if r is None:
                    bad = True
                    break
                per_label[label].append(r[2])
            if bad:
                break
        if bad:
            print(f"  {rel:30s}  RUN FAIL")
            results[rel] = None
            continue
        results[rel] = per_label
        # 单 fixture 行: 每 binary median + 单跑序列
        seg = "  ".join(
            f"{l} med {statistics.median(per_label[l]):7.1f}" for l in labels)
        print(f"  {rel:30s}  {seg}")
        per_fixture_lines.append(
            f"{rel}\n    " + "\n    ".join(
                f"{l:12s} " + " / ".join(f"{x:7.1f}" for x in per_label[l])
                for l in labels))

    # 横向对照表: fixture / med_A / med_B / med_C / B-vs-A% / C-vs-A%
    base_label = labels[0]
    header = f"  {'fixture':30s}  " + "  ".join(f"{l:>10s}" for l in labels) \
             + "  " + "  ".join(f"{l}-vs-{base_label}%".rjust(14) for l in labels[1:])
    print("\n横向对照表 (median MIPS; 占比 = (M_i - M_base) / M_base * 100, "
          f"负值 = 比 {base_label} 慢):")
    print(header)
    print("  " + "-" * (len(header) - 2))
    cmp_lines = [header]
    cmp_lines.append("-" * len(header))
    for rel, per_label in results.items():
        if per_label is None:
            row = f"  {rel:30s}  RUN FAIL"
        else:
            meds = {l: statistics.median(per_label[l]) for l in labels}
            base = meds[base_label]
            cols = "  ".join(f"{meds[l]:10.1f}" for l in labels)
            ratios = "  ".join(
                f"{(meds[l] - base) / base * 100:+13.2f}%" for l in labels[1:])
            row = f"  {rel:30s}  {cols}  {ratios}"
        print(row)
        cmp_lines.append(row)

    os.makedirs(OUT, exist_ok=True)
    res = os.path.join(OUT, "perf_res.txt")
    with open(res, "w") as f:
        f.write(f"perf 对照  {time.strftime('%Y-%m-%d %H:%M:%S')}  "
                f"{len(dirs)} fixture x {len(emus)} binary x {runs} 轮  "
                f"= {len(dirs) * len(emus) * runs} 跑\n")
        f.write("binaries:\n")
        for l, p in emus:
            f.write(f"  {l} = {p}\n")
        f.write("=" * 72 + "\n\n")
        f.write("单 fixture 详:\n")
        for ln in per_fixture_lines:
            f.write(ln + "\n")
        f.write("\n" + "=" * 72 + "\n")
        f.write("横向对照表 (median MIPS):\n")
        for ln in cmp_lines:
            f.write(ln + "\n")
    print(f"\n-> {res}")
    return 0

def main():
    argv = sys.argv[1:]
    # --compare 模式 sniff
    if argv and argv[0] == "--compare":
        if len(argv) < 2:
            print("ERROR: --compare 需要 BIN1,BIN2,... 参数", file=sys.stderr)
            return 1
        bins = argv[1].split(",")
        if len(bins) < 2:
            print("ERROR: --compare 至少 2 个 binary", file=sys.stderr)
            return 1
        # label 取 basename 去掉 jit-emu-rel- 前缀 (若有), 否则用 basename
        emus = []
        for b in bins:
            b_abs = os.path.abspath(b)
            base = os.path.basename(b_abs)
            label = base[len("jit-emu-rel-"):] if base.startswith("jit-emu-rel-") else base
            emus.append((label, b_abs))
        runs = int(argv[2]) if len(argv) >= 3 else 5
        subpath = argv[3] if len(argv) >= 4 else os.path.join("a_02", "a02_7")
        return run_compare(emus, runs, subpath)

    # 原 single-binary 模式
    runs = int(argv[0]) if len(argv) >= 1 else 5
    subpath = argv[1] if len(argv) >= 2 else os.path.join("a_02", "a02_7")
    return run_single(runs, subpath)

if __name__ == "__main__":
    sys.exit(main())
