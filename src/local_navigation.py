from __future__ import annotations

from typing import List, Optional, Set, Tuple

GridPos = Tuple[int, int]


def line_cells(a: GridPos, b: GridPos) -> List[GridPos]:
    """Возвращает клетки от a до b по Брезенхему."""
    x0, y0 = a
    x1, y1 = b
    cells: List[GridPos] = []

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1

    err = dx - dy

    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return cells


def is_segment_free(a: GridPos, b: GridPos, obstacles: Set[GridPos]) -> bool:
    return all(cell not in obstacles for cell in line_cells(a, b))


def local_detour(
    current: GridPos,
    target: GridPos,
    obstacles: Set[GridPos],
    grid_size: int,
    max_radius: int = 3,
) -> Optional[List[GridPos]]:
    """Локальный обход препятствия: ищет промежуточные безопасные точки в окрестности."""
    if is_segment_free(current, target, obstacles):
        return [current, target]

    for radius in range(1, max_radius + 1):
        candidates: List[GridPos] = []
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if abs(dx) + abs(dy) != radius:
                    continue
                px, py = current[0] + dx, current[1] + dy
                if not (0 <= px < grid_size and 0 <= py < grid_size):
                    continue
                candidate = (px, py)
                if candidate in obstacles:
                    continue
                candidates.append(candidate)

        candidates.sort(key=lambda p: abs(p[0] - target[0]) + abs(p[1] - target[1]))
        for mid in candidates:
            if is_segment_free(current, mid, obstacles) and is_segment_free(mid, target, obstacles):
                return [current, mid, target]

    return None


def adjust_path_near_obstacles(path: List[GridPos], obstacles: Set[GridPos], grid_size: int) -> List[GridPos]:
    """Дополнительно корректирует траекторию, отдаляя точки от препятствий."""
    if not path:
        return path

    adjusted = [path[0]]
    for i in range(1, len(path) - 1):
        point = path[i]
        if point in obstacles:
            continue

        neighborhood = [
            (point[0] + 1, point[1]),
            (point[0] - 1, point[1]),
            (point[0], point[1] + 1),
            (point[0], point[1] - 1),
        ]
        if any(n in obstacles for n in neighborhood):
            alternatives = [
                (point[0] + 1, point[1] + 1),
                (point[0] + 1, point[1] - 1),
                (point[0] - 1, point[1] + 1),
                (point[0] - 1, point[1] - 1),
            ]
            moved = False
            for alt in alternatives:
                if 0 <= alt[0] < grid_size and 0 <= alt[1] < grid_size and alt not in obstacles:
                    adjusted.append(alt)
                    moved = True
                    break
            if not moved:
                adjusted.append(point)
        else:
            adjusted.append(point)

    if len(path) > 1:
        adjusted.append(path[-1])

    deduped = [adjusted[0]]
    for p in adjusted[1:]:
        if p != deduped[-1]:
            deduped.append(p)
    return deduped
