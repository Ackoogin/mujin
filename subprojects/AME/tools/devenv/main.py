#!/usr/bin/env python3
"""AME Dev Environment — entry point.

Usage:
    python -m subprojects.AME.tools.devenv.main [--foxglove-url URL] [--no-ros2]

Or from the repository root:
    python -m subprojects.AME.tools.devenv
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
        "--backend",
        choices=["ros2", "pcl", "none"],
        default="ros2",
        help="Communication backend: ros2 (default), pcl (direct C++ bindings), none (offline)",
    )
    parser.add_argument(
        "--no-ros2",
        action="store_true",
        help="Deprecated: use --backend none instead",
    )
    parser.add_argument(
        "--domain",
        default="",
        help="Path to PDDL domain file (required for pcl backend)",
    )
    parser.add_argument(
        "--problem",
        default="",
        help="Path to PDDL problem file (required for pcl backend)",
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

    # Handle backend selection
    backend = args.backend
    if args.no_ros2:
        # Deprecated flag - treat as "none"
        backend = "none"

    if backend == "none":
        # Disable both backends
        import subprojects.AME.tools.devenv.comms.ros2_client as ros2_mod
        ros2_mod._HAS_RCLPY = False
        config.connection.backend = "none"
    elif backend == "pcl":
        # Use direct PCL bindings
        config.connection.backend = "pcl"
        config.connection.domain_path = args.domain
        config.connection.problem_path = args.problem
    else:
        # Default: ros2
        config.connection.backend = "ros2"

    app = App(config)
    app.run()


if __name__ == "__main__":
    main()
