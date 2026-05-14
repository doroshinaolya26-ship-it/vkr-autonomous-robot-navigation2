from __future__ import annotations

import heapq
from typing import Dict, List, Optional, Set, Tuple

GridPos = Tuple[int, int]


def heuristic(a: GridPos, b: GridPos) -> int:
    """Манхэттенское расстояние."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(pos: GridPos, grid_size: int) -> List[GridPos]:
    x, y = pos
    candidates = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
    return [p for p in candidates if 0 <= p[0] < grid_size and 0 <= p[1] < grid_size]


def reconstruct_path(came_from: Dict[GridPos, GridPos], current: GridPos) -> List[GridPos]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return list(reversed(path))


def astar_path(
    start: GridPos,
    goal: GridPos,
    obstacles: Set[GridPos],
    grid_size: int = 20,
) -> Optional[List[GridPos]]:
    """Возвращает путь от start до goal или None, если путь не найден."""
    if start in obstacles or goal in obstacles:
        return None

    open_heap: List[Tuple[int, GridPos]] = []
    heapq.heappush(open_heap, (0, start))

    came_from: Dict[GridPos, GridPos] = {}
    g_score: Dict[GridPos, float] = {start: 0.0}
    f_score: Dict[GridPos, float] = {start: heuristic(start, goal)}

    open_set = {start}

    while open_heap:
        _, current = heapq.heappop(open_heap)
        open_set.discard(current)

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in get_neighbors(current, grid_size):
            if neighbor in obstacles:
                continue

            tentative_g = g_score[current] + 1.0
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                if neighbor not in open_set:
                    heapq.heappush(open_heap, (int(f_score[neighbor]), neighbor))
                    open_set.add(neighbor)

    return None
