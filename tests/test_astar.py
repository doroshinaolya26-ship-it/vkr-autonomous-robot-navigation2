from src.astar import astar_path


def test_astar_finds_path_around_obstacle():
    start = (0, 0)
    goal = (4, 4)
    obstacles = {(1, 0), (1, 1), (1, 2), (1, 3)}
    path = astar_path(start, goal, obstacles, grid_size=6)

    assert path is not None
    assert path[0] == start
    assert path[-1] == goal
    assert all(p not in obstacles for p in path)
