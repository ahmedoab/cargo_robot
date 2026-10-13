"""Shared helpers for the bench-test tools (they run on the Pi without ROS)."""
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "robot_ws" / "src" / "cargo_bot"
sys.path.insert(0, str(PKG))            # lets the tools import cargo_bot.hw / cargo_bot.pins


def config(node):
    """ros__parameters of one node from robot_ws/src/cargo_bot/config/robot.yaml."""
    data = yaml.safe_load((PKG / "config" / "robot.yaml").read_text())
    return data[node]["ros__parameters"]
