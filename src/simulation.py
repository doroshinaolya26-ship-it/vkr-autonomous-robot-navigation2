from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Set, Tuple

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib.pyplot as plt

from src.astar import astar_path
from src.hybrid_planner import hybrid_plan
from src.metrics import RunMetrics, compute_path_length, save_metrics_csv, timed_run

GridPos = Tuple[int, int]


@dataclass
class Scenario:
    name: str
    start: GridPos
    goal: GridPos
    static_obstacles: Set[GridPos]
    dynamic_timeline: List[Set[GridPos]]
    image_name: str


def execute_path(path: List[GridPos], dynamic_timeline: List[Set[GridPos]]) -> Tuple[List[GridPos], bool]:
    if not path:
        return [], False
    actual = [path[0]]
    for i in range(1, len(path)):
        appearing = dynamic_timeline[i - 1] if i - 1 < len(dynamic_timeline) else set()
        if path[i] in appearing:
            return actual, False
        actual.append(path[i])
    return actual, True


def plot_scenario(
    scenario: Scenario,
    astar_route: List[GridPos],
    hybrid_route: List[GridPos],
    output_path: Path,
    grid_size: int = 20,
) -> None:
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_title(scenario.name)
    ax.set_xlim(-0.5, grid_size - 0.5)
    ax.set_ylim(-0.5, grid_size - 0.5)
    ax.set_xticks(range(grid_size))
    ax.set_yticks(range(grid_size))
    ax.grid(True, linestyle="--", alpha=0.3)

    all_obstacles = set(scenario.static_obstacles)
    for t in scenario.dynamic_timeline:
        all_obstacles |= t

    if all_obstacles:
        ox, oy = zip(*all_obstacles)
        ax.scatter(ox, oy, c="black", marker="s", s=110, label="Препятствия")

    if astar_route:
        x_a, y_a = zip(*astar_route)
        ax.plot(x_a, y_a, color="tab:blue", linewidth=2, label="A*")

    if hybrid_route:
        x_h, y_h = zip(*hybrid_route)
        ax.plot(x_h, y_h, color="tab:orange", linewidth=2, linestyle="-.", label="Hybrid")

    ax.scatter([scenario.start[0]], [scenario.start[1]], c="green", s=150, marker="o", label="Старт")
    ax.scatter([scenario.goal[0]], [scenario.goal[1]], c="red", s=150, marker="*", label="Цель")
    ax.legend(loc="upper right")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def build_scenarios() -> List[Scenario]:
    wall = {(6, y) for y in range(2, 18)} - {(6, 10)}
    blocks = {(12, y) for y in range(0, 12)} - {(12, 5)}

    return [
        Scenario(
            name="Scenario 1: static empty",
            start=(1, 1),
            goal=(18, 18),
            static_obstacles=set(),
            dynamic_timeline=[],
            image_name="scenario_1_static.png",
        ),
        Scenario(
            name="Scenario 2: static obstacles",
            start=(1, 1),
            goal=(18, 18),
            static_obstacles=wall | blocks | {(x, 14) for x in range(8, 16)},
            dynamic_timeline=[],
            image_name="scenario_2_obstacles.png",
        ),
        Scenario(
            name="Scenario 3: dynamic obstacle",
            start=(1, 1),
            goal=(18, 1),
            static_obstacles=set(),
            dynamic_timeline=[
                set(),
                {(4, 0), (4, 1), (4, 2), (5, 0), (5, 1), (5, 2), (6, 0), (6, 1), (6, 2)},
                {(4, 0), (4, 1), (4, 2), (5, 0), (5, 1), (5, 2), (6, 0), (6, 1), (6, 2)},
                {(4, 0), (4, 1), (4, 2), (5, 0), (5, 1), (5, 2), (6, 0), (6, 1), (6, 2)},
            ],
            image_name="scenario_3_dynamic.png",
        ),
    ]


def run_simulation() -> None:
    scenarios = build_scenarios()
    metrics_rows: List[RunMetrics] = []

    for scenario in scenarios:
        astar_result, astar_runtime = timed_run(
            lambda: {"path": astar_path(scenario.start, scenario.goal, scenario.static_obstacles, 20)}
        )
        astar_path_raw = astar_result["path"] or [scenario.start]
        astar_exec_path, astar_reached = execute_path(astar_path_raw, scenario.dynamic_timeline)

        hybrid_result, hybrid_runtime = timed_run(
            lambda: {
                "result": hybrid_plan(
                    scenario.start,
                    scenario.goal,
                    scenario.static_obstacles,
                    scenario.dynamic_timeline,
                    20,
                )
            }
        )
        h = hybrid_result["result"]

        metrics_rows.extend(
            [
                RunMetrics(
                    scenario=scenario.name,
                    algorithm="A*",
                    path_length=compute_path_length(astar_exec_path),
                    runtime_sec=astar_runtime,
                    replans=0,
                    reached_goal=astar_reached and astar_exec_path[-1] == scenario.goal,
                ),
                RunMetrics(
                    scenario=scenario.name,
                    algorithm="hybrid",
                    path_length=compute_path_length(h.path),
                    runtime_sec=hybrid_runtime,
                    replans=h.replans,
                    reached_goal=h.reached_goal,
                ),
            ]
        )

        plot_scenario(scenario, astar_exec_path, h.path, Path("results") / scenario.image_name, 20)

    save_metrics_csv(metrics_rows, Path("results/metrics.csv"))


if __name__ == "__main__":
    run_simulation()
