#!/usr/bin/env python3
"""Build the checksum manifest for the frozen VBD W1/W2/W3 bundle."""

from __future__ import annotations

import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "reports" / "MANIFEST.json"
NEWTON_COMMIT = "b74df534"


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def included(path: str) -> bool:
    return (
        path.startswith("reports/logs/vbd/final/")
        or path.startswith("reports/logs/vbd/w1_screen/")
        or (path.startswith("reports/logs/vbd/") and "/" not in path.removeprefix("reports/logs/vbd/") and path.endswith(".json"))
        or path.startswith("reports/logs/vbd/e2v2_")
        or (path.startswith("reports/vbd/") and "/" not in path.removeprefix("reports/vbd/") and path.endswith(".md"))
        or path.startswith("reports/vbd/clips/")
    )


def main() -> None:
    paths = sorted(path for path in git("ls-files").splitlines() if included(path))
    files = []
    for path in paths:
        data = (ROOT / path).read_bytes()
        files.append({"path": path, "sha256": hashlib.sha256(data).hexdigest(), "bytes": len(data)})

    manifest = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "git_commit": git("rev-parse", "HEAD"),
        "newton_commit": NEWTON_COMMIT,
        "file_count": len(files),
        "files": files,
    }
    OUTPUT.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
