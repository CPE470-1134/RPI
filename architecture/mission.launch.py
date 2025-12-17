#!/usr/bin/env python3
"""
ROS2 Launch file for the robot mission architecture.

Launches all nodes in the proper sequence:
1. Hardware/Sensor Layer - driver, lidar, aruco
2. Perception & Safety Layer - localizer, safety
3. Decision & Control Layer - controller, director
"""

import os
import sys

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for the mission."""

    # Get the directory where this launch file is located
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Environment variables to pass to nodes
    env = os.environ.copy()

    return LaunchDescription(
        [
            # --- Hardware / Sensor Layer ---
            Node(
                executable=sys.executable,
                arguments=[os.path.join(script_dir, "driver_node.py")],
                name="driver_node",
                output="screen",
                env=env,
            ),
            Node(
                executable=sys.executable,
                arguments=[os.path.join(script_dir, "lidar_node.py")],
                name="lidar_node",
                output="screen",
                env=env,
            ),
            Node(
                executable=sys.executable,
                arguments=[os.path.join(script_dir, "aruco_node.py")],
                name="aruco_node",
                output="screen",
                env=env,
            ),
            # --- Perception & Safety Layer ---
            Node(
                executable=sys.executable,
                arguments=[os.path.join(script_dir, "localizer_node.py")],
                name="localizer_node",
                output="screen",
                env=env,
            ),
            Node(
                executable=sys.executable,
                arguments=[os.path.join(script_dir, "safety_node.py")],
                name="safety_node",
                output="screen",
                env=env,
            ),
            # --- Decision & Control Layer ---
            Node(
                executable=sys.executable,
                arguments=[os.path.join(script_dir, "controller_node.py")],
                name="controller_node",
                output="screen",
                env=env,
            ),
            Node(
                executable=sys.executable,
                arguments=[os.path.join(script_dir, "director_node.py")],
                name="director_node",
                output="screen",
                env=env,
            ),
        ]
    )


if __name__ == "__main__":
    # Allow running directly for testing
    from launch import LaunchService

    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
