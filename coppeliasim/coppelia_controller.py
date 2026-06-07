#!/usr/bin/env python3
"""Контроллер демонстрационной сцены CoppeliaSim для маршрута A*.

Скрипт предназначен для защиты ВКР: он подключается к CoppeliaSim через
ZeroMQ Remote API, находит объект ``robot`` и пошагово перемещает мобильный
робот по маршруту. Если CoppeliaSim или Python API недоступны, запускается
fallback-режим: программа не падает, а сохраняет демонстрационные метрики и
выводит понятное сообщение о дальнейших действиях.
"""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import math
import os
import sys
import time
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

ROBOT_OBJECT_NAME = "/robot"
DEFAULT_ROUTE: List[Tuple[float, float]] = [
    (0.0, 0.0),
    (0.5, 0.0),
    (1.0, 0.3),
    (1.2, 0.7),
    (1.5, 1.0),
]
SENSOR_COMPONENTS = [
    "колесная база мобильного робота",
    "левое и правое приводные колеса",
    "корпус робота",
    "лидар или имитация дальномерного сенсора",
    "ультразвуковые/инфракрасные датчики расстояния",
    "датчик положения и ориентации IMU",
    "энкодеры колес",
    "датчик столкновения или контактный сенсор",
    "локальная карта / occupancy grid",
]


def _append_coppeliasim_api_path() -> None:
    """Добавить путь к ZeroMQ Remote API из COPPELIASIM_ROOT, если он задан."""
    coppelia_root = os.environ.get("COPPELIASIM_ROOT")
    if not coppelia_root:
        return

    candidate = Path(coppelia_root) / "programming" / "zmqRemoteApi" / "clients" / "python"
    if candidate.exists() and str(candidate) not in sys.path:
        sys.path.append(str(candidate))


def get_remote_api_client_class():
    """Вернуть класс RemoteAPIClient или None, если Python API недоступен."""
    _append_coppeliasim_api_path()
    if importlib.util.find_spec("zmqRemoteApi") is None:
        return None

    module = importlib.import_module("zmqRemoteApi")
    return getattr(module, "RemoteAPIClient", None)


def load_route_points(route_file: Path = Path("results/astar_path.txt")) -> List[Tuple[float, float]]:
    """Загрузить маршрут A* из файла или вернуть демонстрационный маршрут.

    Формат файла маршрута: одна точка на строку, координаты ``x y`` или
    ``x,y``. Такой формат позволяет использовать результаты существующей
    сеточной симуляции без добавления новых зависимостей.
    """
    if route_file.exists():
        points: List[Tuple[float, float]] = []
        for raw_line in route_file.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip().replace(",", " ")
            if not line:
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            points.append((float(parts[0]), float(parts[1])))
        if points:
            return points

    return list(DEFAULT_ROUTE)


def _distance_xy(p1: Sequence[float], p2: Sequence[float]) -> float:
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def move_robot_along_route(
    sim,
    robot_handle: int,
    route: Iterable[Tuple[float, float]],
    step_delay: float = 0.1,
) -> dict:
    """Переместить объект ``robot`` по маршруту и вернуть метрики движения."""
    total_distance = 0.0
    start_time = time.time()
    reached_points = 0

    current_pos = sim.getObjectPosition(robot_handle, -1)
    for idx, waypoint in enumerate(route, start=1):
        target = [float(waypoint[0]), float(waypoint[1]), float(current_pos[2])]
        segment = _distance_xy(current_pos, target)
        total_distance += segment

        sim.setObjectPosition(robot_handle, -1, target)
        time.sleep(step_delay)
        current_pos = sim.getObjectPosition(robot_handle, -1)
        reached_points += 1
        print(
            "[CoppeliaSim] waypoint "
            f"{idx}: position=({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})"
        )

    elapsed = time.time() - start_time
    return {
        "mode": "coppeliasim",
        "reached_points": reached_points,
        "total_distance": total_distance,
        "elapsed_time_sec": elapsed,
        "avg_speed": (total_distance / elapsed) if elapsed > 0 else 0.0,
        "final_position": current_pos,
        "sensor_components": SENSOR_COMPONENTS,
    }


def build_fallback_metrics(route: Sequence[Tuple[float, float]], reason: str) -> dict:
    """Сформировать метрики fallback-режима без подключения к CoppeliaSim."""
    total_distance = 0.0
    previous = [route[0][0], route[0][1], 0.0] if route else [0.0, 0.0, 0.0]
    for point in route[1:]:
        current = [point[0], point[1], 0.0]
        total_distance += _distance_xy(previous, current)
        previous = current

    return {
        "mode": "fallback",
        "fallback_reason": reason,
        "reached_points": len(route),
        "total_distance": total_distance,
        "elapsed_time_sec": 0.0,
        "avg_speed": 0.0,
        "final_position": previous,
        "sensor_components": SENSOR_COMPONENTS,
    }


def save_metrics(metrics: dict, output_file: Path) -> None:
    """Сохранить метрики движения и перечень сенсорных компонентов."""
    output_file.parent.mkdir(parents=True, exist_ok=True)
    content = [
        "CoppeliaSim navigation metrics",
        f"mode={metrics['mode']}",
        f"reached_points={metrics['reached_points']}",
        f"total_distance={metrics['total_distance']:.4f}",
        f"elapsed_time_sec={metrics['elapsed_time_sec']:.4f}",
        f"avg_speed={metrics['avg_speed']:.4f}",
        "final_position={:.4f},{:.4f},{:.4f}".format(*metrics["final_position"]),
    ]
    if metrics.get("fallback_reason"):
        content.append(f"fallback_reason={metrics['fallback_reason']}")
    content.append("sensor_components=" + "; ".join(metrics["sensor_components"]))
    output_file.write_text("\n".join(content) + "\n", encoding="utf-8")


def run_fallback(route: Sequence[Tuple[float, float]], output_file: Path, reason: str) -> int:
    """Запустить безопасный fallback-режим и завершиться без ошибки."""
    print(
        "[Fallback] CoppeliaSim недоступен или отключен. "
        "Будут сохранены демонстрационные метрики без управления 3D-сценой."
    )
    print(f"[Fallback] Причина: {reason}")
    print(
        "[Fallback] Для полноценной демонстрации установите CoppeliaSim, "
        "запустите сцену и проверьте наличие объекта robot."
    )
    metrics = build_fallback_metrics(route, reason)
    save_metrics(metrics, output_file)
    print(f"[Fallback] Метрики сохранены: {output_file}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="CoppeliaSim controller for A* robot navigation demo")
    parser.add_argument(
        "--route-file",
        default="results/astar_path.txt",
        help="Файл с точками маршрута A* в формате 'x y' или 'x,y'.",
    )
    parser.add_argument(
        "--metrics-file",
        default="results/coppelia_metrics.txt",
        help="Файл для сохранения метрик движения.",
    )
    parser.add_argument(
        "--fallback",
        action="store_true",
        help="Принудительно запустить режим без подключения к CoppeliaSim.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    route = load_route_points(Path(args.route_file))
    output_file = Path(args.metrics_file)

    if args.fallback or os.environ.get("COPPELIASIM_FORCE_FALLBACK") == "1":
        return run_fallback(route, output_file, "fallback mode requested")

    RemoteAPIClient = get_remote_api_client_class()
    if RemoteAPIClient is None:
        return run_fallback(
            route,
            output_file,
            "Python API CoppeliaSim (zmqRemoteApi) не найден",
        )

    try:
        client = RemoteAPIClient()
        sim = client.getObject("sim")
    except Exception as exc:  # pragma: no cover - требует запущенный CoppeliaSim
        return run_fallback(
            route,
            output_file,
            f"не удалось подключиться к CoppeliaSim: {exc}",
        )

    try:
        robot_handle = sim.getObject(ROBOT_OBJECT_NAME)
    except Exception as exc:  # pragma: no cover - зависит от содержимого сцены
        return run_fallback(
            route,
            output_file,
            f"объект {ROBOT_OBJECT_NAME} не найден: {exc}",
        )

    metrics = move_robot_along_route(sim, robot_handle, route)
    save_metrics(metrics, output_file)
    print(f"[CoppeliaSim] Метрики сохранены: {output_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
from __future__ import annotations

import heapq
import time
from pathlib import Path
from typing import Dict, List, Tuple

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

GridPoint = Tuple[int, int]

SCALE = 0.22
X_OFFSET = -2.1
Y_OFFSET = 2.1
ROBOT_Z = 0.16


def create_grid() -> List[List[int]]:
    grid = [[0 for _ in range(20)] for _ in range(20)]
    for r in range(4, 16):
        grid[r][6] = 1
    grid[10][6] = 0
    for c in range(7, 16):
        grid[6][c] = 1
    grid[6][10] = 0
    for r in range(9, 17):
        grid[r][12] = 1
    grid[13][12] = 0
    for c in range(2, 6):
        grid[14][c] = 1
    return grid


def manhattan(a: GridPoint, b: GridPoint) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def neighbors_4(grid: List[List[int]], node: GridPoint) -> List[GridPoint]:
    rows = len(grid)
    cols = len(grid[0])
    r, c = node
    pts = [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]
    return [(nr, nc) for nr, nc in pts if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0]


def reconstruct_path(came_from: Dict[GridPoint, GridPoint], current: GridPoint) -> List[GridPoint]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return list(reversed(path))


def astar(grid: List[List[int]], start: GridPoint, goal: GridPoint) -> List[GridPoint]:
    if grid[start[0]][start[1]] != 0 or grid[goal[0]][goal[1]] != 0:
        return []
    open_heap: List[Tuple[int, GridPoint]] = []
    heapq.heappush(open_heap, (0, start))
    came_from: Dict[GridPoint, GridPoint] = {}
    g_cost: Dict[GridPoint, float] = {start: 0.0}
    f_cost: Dict[GridPoint, float] = {start: float(manhattan(start, goal))}
    in_open = {start}

    while open_heap:
        _, current = heapq.heappop(open_heap)
        in_open.discard(current)
        if current == goal:
            return reconstruct_path(came_from, current)
        for nxt in neighbors_4(grid, current):
            cand = g_cost[current] + 1.0
            if cand < g_cost.get(nxt, float("inf")):
                came_from[nxt] = current
                g_cost[nxt] = cand
                f_cost[nxt] = cand + manhattan(nxt, goal)
                if nxt not in in_open:
                    heapq.heappush(open_heap, (int(f_cost[nxt]), nxt))
                    in_open.add(nxt)
    return []


def filter_state(previous_state: Tuple[float, float], measured_state: Tuple[float, float], alpha: float = 0.6) -> Tuple[float, float]:
    x = alpha * measured_state[0] + (1 - alpha) * previous_state[0]
    y = alpha * measured_state[1] + (1 - alpha) * previous_state[1]
    return x, y


def grid_to_scene(cell: GridPoint, z: float = 0.15) -> List[float]:
    row, col = cell
    return [col * SCALE + X_OFFSET, -row * SCALE + Y_OFFSET, z]


def clear_demo_objects(sim) -> None:
    for prefix in ("/obstacle_", "/point_", "/dynamic_obstacle_"):
        idx = 0
        while True:
            try:
                handle = sim.getObject(f"{prefix}{idx}")
            except Exception:
                break
            sim.removeObject(handle)
            idx += 1

    for single in ("/start_marker", "/goal_marker"):
        try:
            sim.removeObject(sim.getObject(single))
        except Exception:
            pass


def scene_center_for_rect(r0: int, c0: int, h: int, w: int, z: float) -> List[float]:
    center_row = r0 + (h - 1) / 2.0
    center_col = c0 + (w - 1) / 2.0
    return [center_col * SCALE + X_OFFSET, -center_row * SCALE + Y_OFFSET, z]


def build_obstacle_rectangles(grid: List[List[int]]) -> List[Tuple[int, int, int, int]]:
    rows = len(grid)
    cols = len(grid[0])
    used = [[False] * cols for _ in range(rows)]
    rects: List[Tuple[int, int, int, int]] = []

    for r in range(rows):
        c = 0
        while c < cols:
            if grid[r][c] != 1 or used[r][c]:
                c += 1
                continue
            w = 1
            while c + w < cols and grid[r][c + w] == 1 and not used[r][c + w]:
                w += 1
            h = 1
            grow = True
            while r + h < rows and grow:
                for cc in range(c, c + w):
                    if grid[r + h][cc] != 1 or used[r + h][cc]:
                        grow = False
                        break
                if grow:
                    h += 1
            for rr in range(r, r + h):
                for cc in range(c, c + w):
                    used[rr][cc] = True
            rects.append((r, c, h, w))
            c += w
    return rects


def create_obstacle_block(sim, idx: int, rect: Tuple[int, int, int, int]) -> None:
    r0, c0, h, w = rect
    sx = max(0.18, w * SCALE * 0.92)
    sy = max(0.18, h * SCALE * 0.92)
    sz = 0.20
    cube = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [sx, sy, sz], 0)
    sim.setObjectAlias(cube, f"obstacle_{idx}")
    sim.setShapeColor(cube, None, sim.colorcomponent_ambient_diffuse, [0.7, 0.12, 0.12])
    sim.setObjectPosition(cube, -1, scene_center_for_rect(r0, c0, h, w, z=0.10))
    print(f"[coppeliasim] создано препятствие obstacle_{idx} блоком ({r0},{c0}) {h}x{w}")


def create_dynamic_obstacle(sim, idx: int, cell: GridPoint) -> None:
    block = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [0.20, 0.20, 0.20], 0)
    sim.setObjectAlias(block, f"dynamic_obstacle_{idx}")
    sim.setShapeColor(block, None, sim.colorcomponent_ambient_diffuse, [0.2, 0.3, 0.9])
    sim.setObjectPosition(block, -1, grid_to_scene(cell, z=0.10))
    print(f"[coppeliasim] создано препятствие dynamic_obstacle_{idx} в клетке {cell}")


def create_point_marker(sim, idx: int, cell: GridPoint) -> None:
    marker = sim.createDummy(0.05)
    sim.setObjectAlias(marker, f"point_{idx}")
    sim.setObjectColor(marker, 0, sim.colorcomponent_ambient_diffuse, [0.2, 0.8, 0.2])
    sim.setObjectPosition(marker, -1, grid_to_scene(cell, z=0.04))


def create_start_goal_markers(sim, start: GridPoint, goal: GridPoint) -> None:
    start_obj = sim.createDummy(0.08)
    sim.setObjectAlias(start_obj, "start_marker")
    sim.setObjectColor(start_obj, 0, sim.colorcomponent_ambient_diffuse, [0.15, 0.85, 0.2])
    sim.setObjectPosition(start_obj, -1, grid_to_scene(start, z=0.05))

    goal_obj = sim.createDummy(0.08)
    sim.setObjectAlias(goal_obj, "goal_marker")
    sim.setObjectColor(goal_obj, 0, sim.colorcomponent_ambient_diffuse, [0.95, 0.85, 0.1])
    sim.setObjectPosition(goal_obj, -1, grid_to_scene(goal, z=0.05))


def render_route_points(sim, route: List[GridPoint]) -> None:
    idx = 0
    while True:
        try:
            sim.removeObject(sim.getObject(f"/point_{idx}"))
            idx += 1
        except Exception:
            break
    for i, wp in enumerate(route):
        create_point_marker(sim, i, wp)


def move_robot_smoothly(sim, robot, from_cell: GridPoint, to_cell: GridPoint, steps: int = 35, delay: float = 0.015) -> None:
    p0 = grid_to_scene(from_cell, z=ROBOT_Z)
    p1 = grid_to_scene(to_cell, z=ROBOT_Z)
    prev_xy = (p0[0], p0[1])
    for s in range(1, steps + 1):
        t = s / steps
        measured = (p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t)
        smooth = filter_state(prev_xy, measured, alpha=0.6)
        sim.setObjectPosition(robot, -1, [smooth[0], smooth[1], ROBOT_Z])
        try:
            sim.resetDynamicObject(robot)
        except Exception:
            pass
        prev_xy = smooth
        time.sleep(delay)


def write_metrics(
    scenario_name: str,
    grid: List[List[int]],
    start: GridPoint,
    goal: GridPoint,
    obstacle_count: int,
    route_length: int,
    replans: int,
    planning_time: float,
    execution_time: float,
    reached_goal: bool,
) -> None:
    path = Path("results") / "coppelia_metrics.txt"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write(f"scenario: {scenario_name}\n")
        f.write(f"grid_size: {len(grid)}x{len(grid[0])}\n")
        f.write(f"start: {start}\n")
        f.write(f"goal: {goal}\n")
        f.write(f"obstacle_count: {obstacle_count}\n")
        f.write(f"route_length: {route_length}\n")
        f.write(f"replans: {replans}\n")
        f.write(f"planning_time_sec: {planning_time:.6f}\n")
        f.write(f"execution_time_sec: {execution_time:.6f}\n")
        f.write(f"reached_goal: {int(reached_goal)}\n")
    print("[metrics] сохранено: results/coppelia_metrics.txt")


def main() -> None:
    scenario_name = "CoppeliaSim Hybrid Scenario"
    grid = create_grid()
    start = (2, 2)
    goal = (16, 16)

    print("[planner] построение глобального маршрута A*")
    t0 = time.perf_counter()
    route = astar(grid, start, goal)
    planning_time = time.perf_counter() - t0
    print(f"[planner] длина маршрута A*: {len(route)}")

    if not route:
        write_metrics(scenario_name, grid, start, goal, sum(cell for row in grid for cell in row), 0, 0, planning_time, 0.0, False)
        return

    try:
        client = RemoteAPIClient()
        sim = client.require("sim")
    except Exception as exc:
        print("[error] Не удалось подключиться к CoppeliaSim.")
        print("[error] Проверь, что CoppeliaSim запущен;")
        print("[error] Проверь, что ZeroMQ Remote API server running;")
        print("[error] Проверь, что модель робота переименована в robot.")
        print(f"[debug] {exc}")
        return

    try:
        robot = sim.getObject("/robot")
    except Exception as exc:
        print("[error] Не найден объект /robot.")
        print("[error] Проверь, что CoppeliaSim запущен;")
        print("[error] Проверь, что ZeroMQ Remote API server running;")
        print("[error] Проверь, что модель робота переименована в robot.")
        print(f"[debug] {exc}")
        return

    clear_demo_objects(sim)
    create_start_goal_markers(sim, start, goal)
    sim.setObjectPosition(robot, -1, grid_to_scene(start, z=ROBOT_Z))
    try:
        sim.resetDynamicObject(robot)
    except Exception:
        pass

    rects = build_obstacle_rectangles(grid)
    for idx, rect in enumerate(rects):
        create_obstacle_block(sim, idx, rect)

    render_route_points(sim, route)

    replans = 0
    current = start
    dynamic_inserted = False
    exec_start = time.perf_counter()

    i = 1
    while i < len(route):
        next_wp = route[i]

        if not dynamic_inserted and i + 2 < len(route):
            dynamic_cell = route[i + 1]
            grid[dynamic_cell[0]][dynamic_cell[1]] = 1
            create_dynamic_obstacle(sim, replans, dynamic_cell)
            dynamic_inserted = True

        if grid[next_wp[0]][next_wp[1]] == 1:
            print("[local] обнаружено препятствие, выполняется перестроение маршрута")
            replans += 1
            new_route = astar(grid, current, goal)
            if not new_route:
                break
            route = route[:i] + new_route[1:]
            render_route_points(sim, route)
            print(f"[local] новый маршрут найден, длина: {len(new_route)}")
            next_wp = route[i]

        print(f"[coppeliasim] движение к waypoint {next_wp}")
        move_robot_smoothly(sim, robot, current, next_wp)
        current = next_wp
        i += 1

    execution_time = time.perf_counter() - exec_start
    reached_goal = current == goal

    write_metrics(
        scenario_name=scenario_name,
        grid=grid,
        start=start,
        goal=goal,
        obstacle_count=sum(cell for row in grid for cell in row),
        route_length=len(route),
        replans=replans,
        planning_time=planning_time,
        execution_time=execution_time,
        reached_goal=reached_goal,
    )


if __name__ == "__main__":
    main()
