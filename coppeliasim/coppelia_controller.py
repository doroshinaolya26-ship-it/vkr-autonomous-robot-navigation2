#!/usr/bin/env python3
"""Simple CoppeliaSim route follower for A* path demonstration.

The script tries to connect to CoppeliaSim via the ZeroMQ Remote API,
gets an object named ``robot``, and moves it through 2D waypoints.
"""

from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple


def _try_import_remote_api():
    """Import CoppeliaSim Remote API client with clear diagnostics."""
    try:
        from zmqRemoteApi import RemoteAPIClient  # type: ignore

        return RemoteAPIClient
    except ImportError:
        coppelia_root = os.environ.get("COPPELIASIM_ROOT")
        if coppelia_root:
            candidate = Path(coppelia_root) / "programming" / "zmqRemoteApi" / "clients" / "python"
            if candidate.exists():
                sys.path.append(str(candidate))
                try:
                    from zmqRemoteApi import RemoteAPIClient  # type: ignore

                    return RemoteAPIClient
                except ImportError:
                    pass

    return None


def load_route_points() -> List[Tuple[float, float]]:
    """Load route produced by A*.

    Expected optional source: ``results/astar_path.txt`` where each line
    has ``x y`` or ``x,y``. If the file is missing, a fallback demo route
    is returned.
    """
    path_file = Path("results/astar_path.txt")
    if path_file.exists():
        points: List[Tuple[float, float]] = []
        for raw_line in path_file.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip().replace(",", " ")
            if not line:
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            points.append((float(parts[0]), float(parts[1])))
        if points:
            return points

    return [(0.0, 0.0), (0.5, 0.0), (1.0, 0.3), (1.2, 0.7), (1.5, 1.0)]


def _distance(p1: Sequence[float], p2: Sequence[float]) -> float:
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def move_robot_along_route(sim, robot_handle: int, route: Iterable[Tuple[float, float]], step_delay: float = 0.1):
    """Move robot along route and return metrics dict."""
    total_distance = 0.0
    start_time = time.time()
    reached_points = 0

    current_pos = sim.getObjectPosition(robot_handle, -1)
    for idx, waypoint in enumerate(route, start=1):
        target = [float(waypoint[0]), float(waypoint[1]), float(current_pos[2])]
        segment = _distance(current_pos, target)
        total_distance += segment

        sim.setObjectPosition(robot_handle, -1, target)
        time.sleep(step_delay)
        current_pos = sim.getObjectPosition(robot_handle, -1)
        reached_points += 1
        print(f"[CoppeliaSim] waypoint {idx}: position=({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")

    elapsed = time.time() - start_time
    return {
        "reached_points": reached_points,
        "total_distance": total_distance,
        "elapsed_time_sec": elapsed,
        "avg_speed": (total_distance / elapsed) if elapsed > 0 else 0.0,
        "final_position": current_pos,
    }


def save_metrics(metrics: dict, output_file: Path) -> None:
    output_file.parent.mkdir(parents=True, exist_ok=True)
    content = [
        "CoppeliaSim navigation metrics",
        f"reached_points={metrics['reached_points']}",
        f"total_distance={metrics['total_distance']:.4f}",
        f"elapsed_time_sec={metrics['elapsed_time_sec']:.4f}",
        f"avg_speed={metrics['avg_speed']:.4f}",
        "final_position={:.4f},{:.4f},{:.4f}".format(*metrics["final_position"]),
    ]
    output_file.write_text("\n".join(content) + "\n", encoding="utf-8")


def main() -> int:
    RemoteAPIClient = _try_import_remote_api()
    if RemoteAPIClient is None:
        print(
            "Не удалось импортировать Python API CoppeliaSim (zmqRemoteApi).\n"
            "Установите CoppeliaSim и добавьте путь к programming/zmqRemoteApi/clients/python в PYTHONPATH\n"
            "или задайте переменную COPPELIASIM_ROOT.",
            file=sys.stderr,
        )
        return 1

    try:
        client = RemoteAPIClient()
        sim = client.getObject("sim")
    except Exception as exc:  # pragma: no cover
        print(
            f"Не удалось подключиться к CoppeliaSim: {exc}.\n"
            "Убедитесь, что CoppeliaSim запущен и активирован ZeroMQ Remote API Add-on.",
            file=sys.stderr,
        )
        return 1

    try:
        robot_handle = sim.getObject("/robot")
    except Exception as exc:
        print(
            f"Объект robot не найден в сцене: {exc}.\n"
            "Проверьте, что объект существует и назван ровно 'robot'.",
            file=sys.stderr,
        )
        return 1

    route = load_route_points()
    metrics = move_robot_along_route(sim, robot_handle, route)
    output = Path("results/coppelia_metrics.txt")
    save_metrics(metrics, output)
    print(f"[CoppeliaSim] Метрики сохранены: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
