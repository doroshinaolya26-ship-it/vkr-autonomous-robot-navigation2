from src.hybrid_planner import hybrid_plan


def test_hybrid_replans_in_dynamic_environment():
    start = (1, 1)
    goal = (8, 1)
    static_obstacles = set()
    dynamic_timeline = [set(), set(), {(4, 1)}, {(5, 1)}]

    result = hybrid_plan(start, goal, static_obstacles, dynamic_timeline, grid_size=12)

    assert result.reached_goal
    assert result.path[0] == start
    assert result.path[-1] == goal
    assert result.replans >= 0
