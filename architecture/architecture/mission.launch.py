#!/usr/bin/env python3
"""
ROS2 Launch file for the robot mission architecture.

Launches all nodes in the proper sequence:
1. Hardware/Sensor Layer - driver, lidar, aruco
2. Perception & Safety Layer - localizer, safety
3. Decision & Control Layer - controller, director
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for the mission."""

    # Environment variables to pass to nodes
    env = os.environ.copy()

    return LaunchDescription(
        [
            # --- Hardware / Sensor Layer ---
            Node(
                package="architecture",
                executable="driver_node",
                name="driver_node",
                output="screen",
                env=env,
            ),
            Node(
                package="architecture",
                executable="lidar_node",
                name="lidar_node",
                output="screen",
                env=env,
            ),
            Node(
                package="architecture",
                executable="aruco_node",
                name="aruco_node",
                output="screen",
                env=env,
            ),
            # --- Perception & Safety Layer ---
            Node(
                package="architecture",
                executable="localizer_node",
                name="localizer_node",
                output="screen",
                env=env,
            ),
            Node(
                package="architecture",
                executable="safety_node",
                name="safety_node",
                output="screen",
                env=env,
            ),
            # --- Decision & Control Layer ---
            Node(
                package="architecture",
                executable="controller_node",
                name="controller_node",
                output="screen",
                env=env,
            ),
            Node(
                package="architecture",
                executable="director_node",
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
