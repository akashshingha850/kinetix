"""Run-directory writer (data-format.md §3): run.yaml, frames.jsonl, steps.jsonl, frames/, log.txt."""
from __future__ import annotations

import json
import platform
import subprocess
import time
from pathlib import Path

import yaml

from kinetix.core.types import Frame

FORMAT = 1


def _git(root: Path) -> dict:
    try:
        sha = subprocess.run(["git", "-C", str(root), "rev-parse", "--short", "HEAD"], capture_output=True, text=True,
                             timeout=5).stdout.strip()
        dirty = bool(subprocess.run(["git", "-C", str(root), "status", "--porcelain"], capture_output=True, text=True,
                                    timeout=5).stdout.strip())
        return {"git": sha or None, "dirty": dirty}
    except (OSError, subprocess.SubprocessError):
        return {"git": None, "dirty": None}


class RunLog:
    def __init__(self, out_root: Path | str, scene: str, method: str, rig: str, seed: int, cfg: dict, uses_gt: bool,
                 repo_root: Path | None = None):
        self.run_id = f"{time.strftime('%Y%m%d-%H%M%S')}_{scene}_{method}_{rig}_s{seed}"
        self.dir = Path(out_root) / self.run_id
        (self.dir / "frames").mkdir(parents=True, exist_ok=True)
        self.t0 = time.time()
        self.frames = 0
        self.travel_m = 0.0
        self.head = {"format": FORMAT, "kinetix": {"version": "0.1.0", **(_git(repo_root) if repo_root else {})},
                     "env": {"host": platform.node()}, "scene": scene, "rig": {"name": rig}, "method": method,
                     "sensing": {"preset": "mono", "plan_depth": "none", "recon_input": "left"},
                     "uses_gt": uses_gt, "seed": seed, "config": cfg}
        self._write_run_yaml()
        self._log = open(self.dir / "log.txt", "a", buffering=1)

    def _write_run_yaml(self, summary: dict | None = None):
        doc = dict(self.head)
        if summary is not None:
            doc["summary"] = summary
        (self.dir / "run.yaml").write_text(yaml.safe_dump(doc, sort_keys=False))

    def info(self, msg: str):
        line = f"[{time.time() - self.t0:8.1f}s] {msg}"
        print(line, flush=True)
        self._log.write(line + "\n")

    def frame(self, f: Frame):
        self.frames += 1
        self.travel_m += f.travel_m
        with open(self.dir / "frames.jsonl", "a") as fh:
            fh.write(json.dumps(f.to_dict()) + "\n")

    def step(self, rec: dict):
        with open(self.dir / "steps.jsonl", "a") as fh:
            fh.write(json.dumps(rec) + "\n")

    def close(self, stop_reason: str, extra: dict | None = None):
        summary = {"frames": self.frames, "registered": None, "travel_m": round(self.travel_m, 2),
                   "wall_s": round(time.time() - self.t0, 1), "sim_s": None, "stop_reason": stop_reason, **(extra or {})}
        self._write_run_yaml(summary)
        self.info(f"run closed: {summary}")
        self._log.close()
        return summary
