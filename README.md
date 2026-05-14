# vkr-autonomous-robot-navigation2
Разработка алгоритма построения маршрута и навигации автономного робота 2.

## Запуск основного проекта

```bash
python main.py
```

## Запуск в CoppeliaSim

1. Установите CoppeliaSim (с поддержкой ZeroMQ Remote API).
2. Откройте подготовленную сцену (см. `coppeliasim/SCENE_SETUP.md`).
3. Запустите симуляцию в CoppeliaSim.
4. Выполните:

```bash
python coppeliasim/coppelia_controller.py
```

Скрипт подключится к CoppeliaSim, получит объект `robot`, передаст маршрут (точки A* из `results/astar_path.txt`, если файл существует), пошагово переместит робота и сохранит метрики в `results/coppelia_metrics.txt`.
