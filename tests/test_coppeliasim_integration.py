from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def test_required_coppeliasim_markdown_files_exist() -> None:
    required_files = [
        REPO_ROOT / "coppeliasim" / "SCENE_SETUP.md",
        REPO_ROOT / "docs" / "ROBOT_COMPONENTS_SPECIFICATION.md",
        REPO_ROOT / "docs" / "COPPELIASIM_SCREENSHOT_GUIDE.md",
        REPO_ROOT / "coppeliasim" / "CREATE_SCENE_SCRIPT.md",
    ]

    for file_path in required_files:
        assert file_path.exists(), f"Missing documentation file: {file_path}"
        assert file_path.read_text(encoding="utf-8").strip(), f"Documentation file is empty: {file_path}"


def test_coppelia_controller_fallback_mode_runs_without_error(tmp_path: Path) -> None:
    metrics_file = tmp_path / "coppelia_metrics.txt"
    env = os.environ.copy()
    env["COPPELIASIM_FORCE_FALLBACK"] = "1"

    result = subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "coppeliasim" / "coppelia_controller.py"),
            "--metrics-file",
            str(metrics_file),
        ],
        cwd=tmp_path,
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "Fallback" in result.stdout
    assert metrics_file.exists()
    metrics = metrics_file.read_text(encoding="utf-8")
    assert "mode=fallback" in metrics
    assert "sensor_components=" in metrics


def test_coppeliasim_scene_creation_script_contains_required_visual_components() -> None:
    script = (REPO_ROOT / "coppeliasim" / "create_demo_scene.lua").read_text(encoding="utf-8")
    required_terms = [
        "robot",
        "vkr_left_drive_wheel_encoder",
        "vkr_right_drive_wheel_encoder",
        "vkr_lidar_range_sensor",
        "vkr_front_ultrasonic_sensor",
        "vkr_imu_orientation_sensor",
        "vkr_contact_bumper_sensor",
        "vkr_obstacle_1",
        "vkr_start_point",
        "vkr_goal_point",
        "Маршрут движения A*",
        "Корпус мобильного робота",
        "УЗ/ИК датчики расстояния",
    ]

    for term in required_terms:
        assert term in script, f"Scene creation script does not mention required component: {term}"
