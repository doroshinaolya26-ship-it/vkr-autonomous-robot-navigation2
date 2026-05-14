from __future__ import annotations

from typing import Iterable, List, Tuple

Point = Tuple[float, float]


def exponential_smoothing(points: Iterable[Point], alpha: float = 0.6) -> List[Point]:
    """Сглаживание: filtered = alpha*current + (1-alpha)*previous_filtered."""
    points = list(points)
    if not points:
        return []
    filtered = [points[0]]
    for current in points[1:]:
        prev = filtered[-1]
        fx = alpha * current[0] + (1 - alpha) * prev[0]
        fy = alpha * current[1] + (1 - alpha) * prev[1]
        filtered.append((fx, fy))
    return filtered
