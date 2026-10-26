"""Regression checks for the unified checkout, with no devices or network control."""
import os
from pathlib import Path
import subprocess
import sys

import pytest

from esrobo_laptop.config import ROOT, WORKSPACE, read_settings, read_robot_config
from esrobo_laptop.acquisition import load_bridge
from esrobo_teleop.debug.skeleton_viewer import __file__ as viewer_module


def test_config_and_resources_resolve_outside_workspace(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    settings = read_settings(ROOT / "config/laptop.yaml")
    cfg = read_robot_config(settings)
    assert Path(settings.teleop_config) == ROOT / "config/teleop_config.yaml"
    assert Path(cfg.ik.urdf_path).is_file()
    assert ROOT == WORKSPACE / "teleoperation"
    assert (ROOT / "web/index.html").is_file()
    assert (Path(viewer_module).parents[3] / "web/pico_skeleton_viewer/index.html").is_file()
    assert callable(load_bridge("xrobotoolkit_body_udp_bridge").parse_args)


@pytest.mark.parametrize("script", ["run_laptop.sh", "run_dashboard.sh"])
def test_cli_wrappers_from_unrelated_working_directory(tmp_path, script):
    env = dict(os.environ, LAPTOP_PYTHON=sys.executable)
    env.pop("PYTHONPATH", None)
    result = subprocess.run([str(ROOT / "scripts" / script), "--help"],
                            cwd=tmp_path, env=env, text=True,
                            capture_output=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "usage:" in result.stdout
