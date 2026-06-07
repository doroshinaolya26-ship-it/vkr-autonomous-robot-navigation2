from src.state_filter import exponential_smoothing


def test_exponential_smoothing_matches_formula():
    points = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]
    alpha = 0.5
    filtered = exponential_smoothing(points, alpha)

    assert filtered[0] == (0.0, 0.0)
    assert filtered[1] == (5.0, 0.0)
    assert filtered[2] == (7.5, 5.0)
