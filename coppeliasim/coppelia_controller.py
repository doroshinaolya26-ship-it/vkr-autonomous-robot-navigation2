from __future__ import annotations

import heapq
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

GridPoint = Tuple[int, int]


def create_grid() -> List[List[int]]:
    grid = [[0 for _ in range(20)] for _ in range(20)]

    for r in range(3, 17):
        grid[r][7] = 1
    grid[10][7] = 0

    for c in range(8, 18):
        grid[5][c] = 1
    grid[5][12] = 0

    for r in range(8, 15):
        grid[r][14] = 1
    grid[11][14] = 0

    return grid


def manhattan(a: GridPoint, b: GridPoint) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def neighbors_4(grid: List[List[int]], node: GridPoint) -> List[GridPoint]:
    rows = len(grid)
    cols = len(grid[0])
    r, c = node
    candidates = [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]

    valid: List[GridPoint] = []
    for nr, nc in candidates:
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
            valid.append((nr, nc))
    return valid


def reconstruct_path(came_from: Dict[GridPoint, GridPoint], current: GridPoint) -> List[GridPoint]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


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
            tentative = g_cost[current] + 1.0
            if tentative < g_cost.get(nxt, float("inf")):
                came_from[nxt] = current
                g_cost[nxt] = tentative
                f_cost[nxt] = tentative + manhattan(nxt, goal)
                if nxt not in in_open:
                    heapq.heappush(open_heap, (int(f_cost[nxt]), nxt))
                    in_open.add(nxt)

    return []


def filter_state(previous_state: Tuple[float, float], measured_state: Tuple[float, float], alpha: float = 0.6) -> Tuple[float, float]:
    x = alpha * measured_state[0] + (1.0 - alpha) * previous_state[0]
    y = alpha * measured_state[1] + (1.0 - alpha) * previous_state[1]
    return x, y


def grid_to_scene(cell: GridPoint, scale: float = 0.35, x_offset: float = -3.0, y_offset: float = 3.0, z: float = 0.15) -> List[float]:
    row, col = cell
    x = col * scale + x_offset
    y = -row * scale + y_offset
    return [x, y, z]


def clear_demo_objects(sim) -> None:
    for prefix in ("/obstacle_", "/point_", "/dynamic_obstacle_"):
        idx = 0
        while True:
            alias = f"{prefix}{idx}"
            try:
                handle = sim.getObject(alias)
            except Exception:
                break
            sim.removeObject(handle)
            idx += 1


def create_obstacle_cube(sim, idx: int, cell: GridPoint) -> None:
    options = 0
    size = [0.28, 0.28, 0.28]
    cube = sim.createPrimitiveShape(sim.primitiveshape_cuboid, size, options)
    sim.setObjectAlias(cube, f"obstacle_{idx}")
    sim.setShapeColor(cube, None, sim.colorcomponent_ambient_diffuse, [0.85, 0.2, 0.2])
    sim.setObjectPosition(cube, -1, grid_to_scene(cell, z=0.14))
    print(f"[coppeliasim] создано препятствие obstacle_{idx} в клетке {cell}")


def create_point_marker(sim, idx: int, cell: GridPoint) -> None:
    sphere = sim.createPrimitiveShape(sim.primitiveshape_spheroid, [0.08, 0.08, 0.08], 0)
    sim.setObjectAlias(sphere, f"point_{idx}")
    sim.setShapeColor(sphere, None, sim.colorcomponent_ambient_diffuse, [0.2, 0.7, 0.25])
    sim.setObjectPosition(sphere, -1, grid_to_scene(cell, z=0.06))


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
    output_path = Path("results") / "coppelia_metrics.txt"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
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
    start = (1, 1)
    goal = (18, 18)

    print("[planner] построение маршрута A*")
    t0 = time.perf_counter()
    path = astar(grid, start, goal)
    planning_time = time.perf_counter() - t0

    if not path:
        print("[planner] маршрут не найден")
        write_metrics(scenario_name, grid, start, goal, sum(cell for row in grid for cell in row), 0, 0, planning_time, 0.0, False)
        return

    print(f"[planner] длина маршрута: {len(path)}")

    try:
        client = RemoteAPIClient()
        sim = client.require("sim")
    except Exception as exc:
        print("[error] Не удалось подключиться к CoppeliaSim.")
        print("[error] Проверь, что CoppeliaSim запущен;")
        print("[error] Проверь, что включен ZeroMQ Remote API server running;")
        print("[error] Проверь, что модель робота переименована в robot.")
        print(f"[debug] {exc}")
        return

    try:
        robot = sim.getObject("/robot")
    except Exception as exc:
        print("[error] Не найден объект /robot.")
        print("[error] Проверь, что CoppeliaSim запущен;")
        print("[error] Проверь, что включен ZeroMQ Remote API server running;")
        print("[error] Проверь, что модель робота переименована в robot.")
        print(f"[debug] {exc}")
        return

    clear_demo_objects(sim)

    obstacle_idx = 0
    for r, row in enumerate(grid):
        for c, value in enumerate(row):
            if value == 1:
                create_obstacle_cube(sim, obstacle_idx, (r, c))
                obstacle_idx += 1

    for i, wp in enumerate(path):
        create_point_marker(sim, i, wp)

    replans = 0
    current = start
    route = list(path)
    dynamic_inserted = False

    sim_start = time.perf_counter()

    for i in range(1, len(route)):
        next_wp = route[i]

        if not dynamic_inserted and i + 2 < len(route):
            dyn_cell = route[i + 1]
            grid[dyn_cell[0]][dyn_cell[1]] = 1
            dynamic_inserted = True
            cube = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [0.28, 0.28, 0.28], 0)
            sim.setObjectAlias(cube, f"dynamic_obstacle_{replans}")
            sim.setShapeColor(cube, None, sim.colorcomponent_ambient_diffuse, [0.2, 0.2, 0.85])
            sim.setObjectPosition(cube, -1, grid_to_scene(dyn_cell, z=0.14))
            print(f"[coppeliasim] создано препятствие dynamic_obstacle_{replans} в клетке {dyn_cell}")

        if grid[next_wp[0]][next_wp[1]] == 1:
            print("[local] обнаружено препятствие, выполняется перестроение")
            replans += 1
            new_path = astar(grid, current, goal)
            if not new_path:
                break
            route = route[:i] + new_path[1:]
            next_wp = route[i]

        measured_scene = grid_to_scene(next_wp)
        previous_scene = grid_to_scene(current)
        smooth_xy = filter_state((previous_scene[0], previous_scene[1]), (measured_scene[0], measured_scene[1]), alpha=0.6)
        target_pos = [smooth_xy[0], smooth_xy[1], measured_scene[2]]

        sim.setObjectPosition(robot, -1, target_pos)
        print(f"[coppeliasim] движение к waypoint {next_wp}")
        current = next_wp
        time.sleep(0.03)

    execution_time = time.perf_counter() - sim_start
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
