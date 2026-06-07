from __future__ import annotations

import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Tuple

GridPos = Tuple[int, int]


@dataclass
class RunMetrics:
    scenario: str
    algorithm: str
    path_length: float
    runtime_sec: float
    replans: int
    reached_goal: bool


def compute_path_length(path: List[GridPos]) -> float:
    if len(path) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(path)):
        x0, y0 = path[i - 1]
        x1, y1 = path[i]
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def timed_run(fn: Callable[[], Dict]) -> Tuple[Dict, float]:
    start = time.perf_counter()
    result = fn()
    runtime = time.perf_counter() - start
    return result, runtime


def save_metrics_csv(rows: List[RunMetrics], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["scenario", "algorithm", "path_length", "runtime_sec", "replans", "reached_goal"])
        for r in rows:
            writer.writerow(
                [
                    r.scenario,
                    r.algorithm,
                    round(r.path_length, 3),
                    round(r.runtime_sec, 6),
                    r.replans,
                    int(r.reached_goal),
                ]
            )
