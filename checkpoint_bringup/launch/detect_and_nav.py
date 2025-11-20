from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        #----------------------
        # ARUCO DETECTION NODE
        #----------------------
        Node(
            package="aruco_pkg",
            executable="aruco_node",
            name="aruco_node",
            output="screen",
        ),
        
        #----------------------
        # CONTROL / NAVIGATION NODES
        #----------------------
        Node(
            package='robot_control_pkg',
            executable='align_robot',
            name='align_robot_node',
            output='screen',
        ),
        Node(
            package='robot_control_pkg',
            executable='approach_marker',
            name='approach_marker_node',
            output='screen',
        ),
    ])