#!/usr/bin/env python3
"""AME Dev Environment — entry point.

Usage:
    python -m tools.devenv.main [--foxglove-url URL] [--no-ros2]

Or from the repository root:
    python -m tools.devenv
"""

from __future__ import annotations

import argparse
import sys

from .config import AppConfig
from .ui.app import App


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="AME Dev Environment — unified UI for ROS2 planning system",
    )
    parser.add_argument(
        "--foxglove-url",
        default="ws://localhost:8765",
        help="Foxglove WebSocket URL (default: ws://localhost:8765)",
    )
    parser.add_argument(
        "--no-ros2",
        action="store_true",
        help="Disable ROS2 client (offline/replay mode only)",
    )
    parser.add_argument(
        "--width",
        type=int, default=1600,
        help="Window width (default: 1600)",
    )
    parser.add_argument(
        "--height",
        type=int, default=1000,
        help="Window height (default: 1000)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    config = AppConfig()
    config.connection.foxglove_url = args.foxglove_url
    config.ui.window_width = args.width
    config.ui.window_height = args.height

    if args.no_ros2:
        # Prevent rclpy import by marking unavailable
        import tools.devenv.comms.ros2_client as ros2_mod
        ros2_mod._HAS_RCLPY = False

    app = App(config)
    app.run()


if __name__ == "__main__":
    main()
