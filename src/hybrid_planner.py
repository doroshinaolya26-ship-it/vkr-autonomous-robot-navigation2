from __future__ import annotations

from dataclasses import dataclass
from typing import List, Set, Tuple

from .astar import astar_path
from .local_navigation import adjust_path_near_obstacles, local_detour

GridPos = Tuple[int, int]


@dataclass
class HybridResult:
    path: List[GridPos]
    replans: int
    reached_goal: bool


def hybrid_plan(
    start: GridPos,
    goal: GridPos,
    static_obstacles: Set[GridPos],
    dynamic_obstacles_timeline: List[Set[GridPos]],
    grid_size: int = 20,
) -> HybridResult:
    """Гибридный алгоритм: глобальный A* + локальный обход + replanning."""
    known_obstacles = set(static_obstacles)
    global_path = astar_path(start, goal, known_obstacles, grid_size)
    if not global_path:
        return HybridResult(path=[start], replans=0, reached_goal=False)

    robot_path: List[GridPos] = [start]
    current = start
    replans = 0
    step_idx = 0

    while current != goal:
        if step_idx < len(dynamic_obstacles_timeline):
            known_obstacles |= dynamic_obstacles_timeline[step_idx]

        if current not in global_path:
            global_path = astar_path(current, goal, known_obstacles, grid_size)
            replans += 1
            if not global_path:
                return HybridResult(path=robot_path, replans=replans, reached_goal=False)

        current_index = global_path.index(current)
        if current_index + 1 >= len(global_path):
            break
        next_wp = global_path[current_index + 1]

        if next_wp in known_obstacles:
            detour = local_detour(current, global_path[min(current_index + 2, len(global_path) - 1)], known_obstacles, grid_size)
            if detour and len(detour) > 1:
                for p in detour[1:]:
                    if p in known_obstacles:
                        break
                    robot_path.append(p)
                    current = p
                step_idx += 1
                continue

            global_path = astar_path(current, goal, known_obstacles, grid_size)
            replans += 1
            if not global_path:
                return HybridResult(path=robot_path, replans=replans, reached_goal=False)
            continue

        robot_path.append(next_wp)
        current = next_wp
        step_idx += 1

        if step_idx > grid_size * grid_size * 4:
            return HybridResult(path=robot_path, replans=replans, reached_goal=False)

    final_path = adjust_path_near_obstacles(robot_path, known_obstacles, grid_size)
    return HybridResult(path=final_path, replans=replans, reached_goal=(final_path[-1] == goal if final_path else False))
