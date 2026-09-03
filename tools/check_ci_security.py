#!/usr/bin/env python3
"""Fail closed on GitHub Actions/self-hosted trust-boundary regressions."""
from __future__ import annotations

import re
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
WORKFLOWS = ROOT / ".github" / "workflows"


def trigger_header(text: str) -> str:
    match = re.search(r"(?ms)^on:\s*\n(.*?)(?=^[A-Za-z_][A-Za-z0-9_-]*:\s*(?:\n|$))", text)
    return match.group(1) if match else ""


def check(path: Path) -> list[str]:
    text = path.read_text()
    triggers = trigger_header(text)
    problems: list[str] = []
    has_pr = bool(re.search(r"(?m)^\s{2}(pull_request|pull_request_target):", triggers))
    self_hosted = bool(re.search(r"(?m)^\s*runs-on:\s*(?:\[?[^\n]*\bself-hosted\b)", text))

    if "pull_request_target:" in triggers:
        problems.append("pull_request_target is forbidden")
    if has_pr and self_hosted:
        problems.append("a pull-request-triggered workflow contains a self-hosted job")
    if self_hosted:
        if "pull_request" in triggers:
            problems.append("self-hosted workflow trigger text mentions pull_request")
        if not re.search(r"(?ms)^\s{2}push:\s*\n\s{4}branches:\s*\[main\]", triggers):
            problems.append("self-hosted workflow push trigger is not restricted to main")
        if "persist-credentials: false" not in text:
            problems.append("self-hosted checkout does not disable persisted credentials")
        if "ref: ${{ github.event_name == 'push' && github.sha || 'refs/heads/main' }}" not in text:
            problems.append("self-hosted checkout is not pinned to triggering protected SHA/main")
        if "LMCAD_REQUIRE_REPRODUCIBLE_ANALYSIS: '1'" not in text:
            problems.append("self-hosted physics does not enforce reproducible analysis")
    return [f"{path.relative_to(ROOT)}: {p}" for p in problems]


def main() -> int:
    if "--self-test" in sys.argv:
        unsafe = """on:\n  pull_request:\njobs:\n  pwn:\n    runs-on: self-hosted\n"""
        with tempfile.NamedTemporaryFile("w", suffix=".yml", dir=WORKFLOWS, delete=False) as handle:
            handle.write(unsafe)
            probe = Path(handle.name)
        try:
            found = check(probe)
            assert any("pull-request-triggered" in item for item in found), found
        finally:
            probe.unlink(missing_ok=True)
        print("CI security negative control: PASS (unsafe PR/self-hosted fixture fired)")
        return 0
    problems: list[str] = []
    paths = sorted(WORKFLOWS.glob("*.yml")) + sorted(WORKFLOWS.glob("*.yaml"))
    for path in paths:
        problems.extend(check(path))
    if problems:
        print("CI SECURITY GATE FAILED", file=sys.stderr)
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        return 1
    print(f"CI security gate: PASS ({len(paths)} workflows; no PR/self-hosted crossing)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
