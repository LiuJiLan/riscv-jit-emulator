#!/usr/bin/env python3
"""把 CLion workspace.xml 里 fixture run config 的 PROGRAM_PARAMS 加 `--bios `
前缀.

跟 notes/context/cmdline_decision.md (a_03_session_008 中段命令行参数重做)
配对. 现有 fixture run config 全部是 `PROGRAM_PARAMS="tests/<path>"` 单文件
形态, main.c 改造后 argv 解析要求 `--bios FILE` 显式, 一刀切批量改.

用法 (项目根跑):
    python3 scripts/fix_workspace_cmdline.py [--dry-run] [PATH]

参数:
    --dry-run   只报告会改多少, 不写文件
    PATH        默认 .idea/workspace.xml; 也可指 org.xml / mod.xml 跑 sample

判断条件 (满足才改):
    PROGRAM_PARAMS 值非空 + 以 "tests/" 开头 + 不含 --bios/--load/--blk
    满足 → 加 "--bios " 前缀

不满足跳过 (幂等 — 再跑一次不会双重前缀):
    - 已是新形态 (含 --bios/--load/--blk)
    - 非 tests/ 路径 (其他 target 可能存在, 不动)
    - 空 PROGRAM_PARAMS

副作用:
    - 默认原地修改, 同目录写 <PATH>.bak 备份
    - --dry-run 时不写任何文件
"""
import re
import shutil
import sys
from pathlib import Path

PATTERN = re.compile(r'PROGRAM_PARAMS="([^"]*)"')


def main() -> None:
    args = sys.argv[1:]
    dry_run = False
    if "--dry-run" in args:
        args.remove("--dry-run")
        dry_run = True
    path = Path(args[0]) if args else Path(".idea/workspace.xml")

    if not path.is_file():
        sys.exit(f"file not found: {path}")

    content = path.read_text(encoding="utf-8")
    changed = 0

    def transform(match: re.Match) -> str:
        nonlocal changed
        val = match.group(1)
        if not val:
            return match.group(0)
        if "--bios" in val or "--load" in val or "--blk" in val:
            return match.group(0)
        if not val.startswith("tests/"):
            return match.group(0)
        changed += 1
        return f'PROGRAM_PARAMS="--bios {val}"'

    new_content = PATTERN.sub(transform, content)

    if changed == 0:
        print(f"nothing to change in {path}")
        return

    if dry_run:
        print(f"[dry-run] would change {changed} configurations in {path}")
        return

    backup = path.with_suffix(path.suffix + ".bak")
    shutil.copy(path, backup)
    path.write_text(new_content, encoding="utf-8")
    print(f"backed up: {backup}")
    print(f"updated:   {path}")
    print(f"changed:   {changed} configurations")


if __name__ == "__main__":
    main()
