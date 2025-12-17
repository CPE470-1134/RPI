#!/usr/bin/env python3
"""
Director Node - Mission Orchestration

This node orchestrates the complete mission by executing the state machine,
commanding navigation, and managing mission-specific domain logic. It uses
the localizer node for pose estimation.
"""

import asyncio
import math
from enum import Enum
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from custom_interfaces.action import FollowMarker, GoToPose
from custom_interfaces.msg import ArucoMarkerArray


class MissionState(Enum):
    INIT = 0
    GET_USER_INPUT = 1
    LOCALIZE = 2
    NAV_TO_P = 3
    WAIT_AT_P = 4
    NAV_TO_GAP = 5
    CROSS_GAP = 6
    SEARCH_TAG_1 = 7
    APPROACH_TAG_1 = 8
    DONE = 9
    ERROR = 99


class DirectorNode(Node):
    """
    ROS 2 node for mission orchestration
    """

    def __init__(self) -> None:
        super().__init__("director_node")

        self._declare_parameters()

        self.state = MissionState.INIT
        self.target_p: Optional[Tuple[float, float]] = None
        self.current_pose: Optional[PoseStamped] = None
        self.is_localized = False
        self.marker_1_visible = False

        # Room dimensions (A=0,0)
        self.room_width = 0.93
        self.room_height = 1.15
        self.gap_center_x = 0.93
        self.gap_center_y = 0.33  # As per analysis

        self._setup_action_clients()
        self._setup_service_clients()
        self._setup_subscribers()
        self._setup_publishers()

        # Start mission loop
        self.create_timer(0.1, self.run_mission_loop)
        self.mission_task = None

        self.get_logger().info("Director node initialized")

    def _declare_parameters(self) -> None:
        pass

    def _setup_action_clients(self) -> None:
        self.goto_client = ActionClient(self, GoToPose, "/controller/go_to_pose")
        self.follow_client = ActionClient(
            self, FollowMarker, "/controller/follow_marker"
        )

    def _setup_service_clients(self) -> None:
        self.localize_client = self.create_client(Trigger, "/localizer/localize")

    def _setup_subscribers(self) -> None:
        self.create_subscription(
            PoseStamped, "/localizer/pose", self._pose_callback, 10
        )
        self.create_subscription(String, "/localizer/status", self._status_callback, 10)
        self.create_subscription(
            ArucoMarkerArray, "/aruco/marker_info", self._aruco_callback, 10
        )

    def _setup_publishers(self) -> None:
        self.mission_status_pub = self.create_publisher(
            String, "/director/mission_status", 10
        )

    def run_mission_loop(self):
        # Using a task to handle async state machine
        if self.mission_task is None or self.mission_task.done():
            self.mission_task = asyncio.create_task(self.run_mission())

    async def run_mission(self) -> None:
        # Simple state machine dispatcher
        if self.state == MissionState.INIT:
            self.state = MissionState.GET_USER_INPUT

        elif self.state == MissionState.GET_USER_INPUT:
            await self._state_get_user_input()

        elif self.state == MissionState.LOCALIZE:
            await self._state_localize()

        elif self.state == MissionState.NAV_TO_P:
            await self._state_nav_to_p()

        elif self.state == MissionState.WAIT_AT_P:
            await self._state_wait_at_p()

        elif self.state == MissionState.NAV_TO_GAP:
            await self._state_nav_to_gap()

        elif self.state == MissionState.CROSS_GAP:
            await self._state_cross_gap()

        elif self.state == MissionState.SEARCH_TAG_1:
            await self._state_search_tag_1()

        elif self.state == MissionState.APPROACH_TAG_1:
            await self._state_approach_tag_1()

        elif self.state == MissionState.DONE:
            pass

        elif self.state == MissionState.ERROR:
            pass

    async def _state_get_user_input(self):
        self._publish_status("Waiting for user input...")
        # Non-blocking input handling would require a separate thread/service
        # For this demo, we'll just block briefly or check a param
        loop = asyncio.get_running_loop()
        try:
            print("Enter coordinates for Point P (x y): ")
            user_input = await loop.run_in_executor(None, input)
            parts = user_input.split()
            if len(parts) == 2:
                self.target_p = (float(parts[0]), float(parts[1]))
                self.get_logger().info(f"Target P set to {self.target_p}")
                self.state = MissionState.LOCALIZE
            else:
                self.get_logger().error("Invalid input. Format: x y")
        except Exception as e:
            self.get_logger().error(f"Input error: {e}")

    async def _state_localize(self):
        self._publish_status("Localizing...")

        while not self.localize_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for localize service...")

        req = Trigger.Request()
        future = self.localize_client.call_async(req)

        # Wait for future
        while not future.done():
            await asyncio.sleep(0.1)

        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info("Localization successful")
                self.state = MissionState.NAV_TO_P
            else:
                self.get_logger().warn(f"Localization failed: {resp.message}")
                # Retry
                await asyncio.sleep(2.0)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    async def _state_nav_to_p(self):
        self._publish_status(f"Navigating to P {self.target_p}...")
        success = await self._send_goto_goal(self.target_p[0], self.target_p[1])
        if success:
            self.state = MissionState.WAIT_AT_P
        else:
            self.get_logger().error("Failed to reach P")
            self.state = MissionState.ERROR

    async def _state_wait_at_p(self):
        self._publish_status("Waiting at P...")
        await asyncio.sleep(3.0)
        self.state = MissionState.NAV_TO_GAP

    async def _state_nav_to_gap(self):
        # Target: Inside room, aligned with gap
        # Gap at (0.93, 0.33). Go to (0.75, 0.33)
        target_x = 0.75
        target_y = self.gap_center_y

        self._publish_status("Navigating to Gap Approach...")
        success = await self._send_goto_goal(target_x, target_y)
        if success:

            # Rotate to face gap (0 degrees)
            success = await self._send_goto_goal(target_x, target_y, target_theta=0.0)
            self.state = MissionState.CROSS_GAP
        else:
            self.state = MissionState.ERROR

    async def _state_cross_gap(self):
        # Target: Outside room
        target_x = 1.30  # Outside room, aligned with gap
        target_y = self.gap_center_y

        self._publish_status("Crossing Gap...")
        success = await self._send_goto_goal(target_x, target_y)
        if success:
            self.state = MissionState.SEARCH_TAG_1
        else:
            self.state = MissionState.ERROR

    async def _state_search_tag_1(self):
        self._publish_status("Searching for Tag 1...")

        if self.marker_1_visible:
            self.state = MissionState.APPROACH_TAG_1
            return

        # Rotate slowly to find it

        current_yaw = self._get_current_yaw()
        target_yaw = current_yaw + math.radians(45)
        # We need a way to send just rotation.
        # GoToPose with current X,Y and new Theta.

        if self.current_pose:
            success = await self._send_goto_goal(
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                target_theta=target_yaw,
            )
            if not success:
                self.state = MissionState.ERROR
        else:
            self.get_logger().error("Lost pose during search")
            self.state = MissionState.ERROR

    async def _state_approach_tag_1(self):
        self._publish_status("Approaching Tag 1...")

        goal = FollowMarker.Goal()
        goal.marker_id = 1
        goal.stop_distance = 0.10

        future = self.follow_client.send_goal_async(goal)
        while not future.done():
            await asyncio.sleep(0.1)

        # Wait for goal to be accepted
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Follow goal rejected")
            self.state = MissionState.ERROR
            return

        # Wait for result
        res_future = goal_handle.get_result_async()
        while not res_future.done():
            await asyncio.sleep(0.1)

        res = res_future.result()
        if res.result.success:
            self.state = MissionState.DONE
            self._publish_status("Mission Complete!")
        else:
            self.state = MissionState.ERROR

    async def _send_goto_goal(self, x, y, target_theta=None):

        # Simple goal creation and sending
        if not self.goto_client.wait_for_server(timeout_sec=1.0):
            return False
        #
        goal = GoToPose.Goal()
        goal.target_x = float(x)
        goal.target_y = float(y)
        if target_theta is not None:
            goal.target_theta = float(target_theta)
        else:
            # Keep current orientation
            goal.target_theta = self._get_current_yaw()
        future = self.goto_client.send_goal_async(goal)
        while not future.done():
            await asyncio.sleep(0.1)

        goal_handle = future.result()
        if not goal_handle.accepted:
            return False

        res_future = goal_handle.get_result_async()
        while not res_future.done():
            await asyncio.sleep(0.1)

        return res_future.result().result.success

    def _pose_callback(self, msg):
        self.current_pose = msg

    def _status_callback(self, msg):
        pass

    def _aruco_callback(self, msg):
        for m in msg.markers:
            if m.marker_id == 1:
                self.marker_1_visible = True
                # If searching, we could interrupt, but simple state machine will catch it next loop

    def _publish_status(self, msg):
        self.mission_status_pub.publish(String(data=msg))
        self.get_logger().info(f"Mission: {msg}")

    def _get_current_yaw(self):

        # Quaternion to yaw
        if self.current_pose:
            q = self.current_pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny_cosp, cosy_cosp)
        return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = DirectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
