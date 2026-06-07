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
