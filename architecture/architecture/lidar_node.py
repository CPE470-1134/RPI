#!/usr/bin/env python3
"""
LiDAR Sensor Node

This node is responsible for interfacing with the LD19 LiDAR sensor via serial
communication and publishing raw scan data for use by other nodes.
"""

import math
import struct
import sys

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# ============================================================================
# LD19 Packet Parsing Logic
# ============================================================================

CRC_TABLE = [
    0x00,
    0x4D,
    0x9A,
    0xD7,
    0x79,
    0x34,
    0xE3,
    0xAE,
    0xF2,
    0xBF,
    0x68,
    0x25,
    0x8B,
    0xC6,
    0x11,
    0x5C,
    0xA9,
    0xE4,
    0x33,
    0x7E,
    0xD0,
    0x9D,
    0x4A,
    0x07,
    0x5B,
    0x16,
    0xC1,
    0x8C,
    0x22,
    0x6F,
    0xB8,
    0xF5,
    0x1F,
    0x52,
    0x85,
    0xC8,
    0x66,
    0x2B,
    0xFC,
    0xB1,
    0xED,
    0xA0,
    0x77,
    0x3A,
    0x94,
    0xD9,
    0x0E,
    0x43,
    0xB6,
    0xFB,
    0x2C,
    0x61,
    0xCF,
    0x82,
    0x55,
    0x18,
    0x44,
    0x09,
    0xDE,
    0x93,
    0x3D,
    0x70,
    0xA7,
    0xEA,
    0x3E,
    0x73,
    0xA4,
    0xE9,
    0x47,
    0x0A,
    0xDD,
    0x90,
    0xCC,
    0x81,
    0x56,
    0x1B,
    0xB5,
    0xF8,
    0x2F,
    0x62,
    0x97,
    0xDA,
    0x0D,
    0x40,
    0xEE,
    0xA3,
    0x74,
    0x39,
    0x65,
    0x28,
    0xFF,
    0xB2,
    0x1C,
    0x51,
    0x86,
    0xCB,
    0x21,
    0x6C,
    0xBB,
    0xF6,
    0x58,
    0x15,
    0xC2,
    0x8F,
    0xD3,
    0x9E,
    0x49,
    0x04,
    0xAA,
    0xE7,
    0x30,
    0x7D,
    0x88,
    0xC5,
    0x12,
    0x5F,
    0xF1,
    0xBC,
    0x6B,
    0x26,
    0x7A,
    0x37,
    0xE0,
    0xAD,
    0x03,
    0x4E,
    0x99,
    0xD4,
    0x7C,
    0x31,
    0xE6,
    0xAB,
    0x05,
    0x48,
    0x9F,
    0xD2,
    0x8E,
    0xC3,
    0x14,
    0x59,
    0xF7,
    0xBA,
    0x6D,
    0x20,
    0xD5,
    0x98,
    0x4F,
    0x02,
    0xAC,
    0xE1,
    0x36,
    0x7B,
    0x27,
    0x6A,
    0xBD,
    0xF0,
    0x5E,
    0x13,
    0xC4,
    0x89,
    0x63,
    0x2E,
    0xF9,
    0xB4,
    0x1A,
    0x57,
    0x80,
    0xCD,
    0x91,
    0xDC,
    0x0B,
    0x46,
    0xE8,
    0xA5,
    0x72,
    0x3F,
    0xCA,
    0x87,
    0x50,
    0x1D,
    0xB3,
    0xFE,
    0x29,
    0x64,
    0x38,
    0x75,
    0xA2,
    0xEF,
    0x41,
    0x0C,
    0xDB,
    0x96,
    0x42,
    0x0F,
    0xD8,
    0x95,
    0x3B,
    0x76,
    0xA1,
    0xEC,
    0xB0,
    0xFD,
    0x2A,
    0x67,
    0xC9,
    0x84,
    0x53,
    0x1E,
    0xEB,
    0xA6,
    0x71,
    0x3C,
    0x92,
    0xDF,
    0x08,
    0x45,
    0x19,
    0x54,
    0x83,
    0xCE,
    0x60,
    0x2D,
    0xFA,
    0xB7,
    0x5D,
    0x10,
    0xC7,
    0x8A,
    0x24,
    0x69,
    0xBE,
    0xF3,
    0xAF,
    0xE2,
    0x35,
    0x78,
    0xD6,
    0x9B,
    0x4C,
    0x01,
    0xF4,
    0xB9,
    0x6E,
    0x23,
    0x8D,
    0xC0,
    0x17,
    0x5A,
    0x06,
    0x4B,
    0x9C,
    0xD1,
    0x7F,
    0x32,
    0xE5,
    0xA8,
]


class LiDARPoint:
    def __init__(self, dist, intensity, angle):
        self.distance_mm = dist
        self.intensity = intensity
        self.angle_deg = angle


class LD19Packet:
    FRAME_SIZE = 47
    HEADER = 0x54
    POINT_PER_PACK = 12

    def __init__(self, raw: bytes):
        if len(raw) != self.FRAME_SIZE or not raw:
            raise ValueError("Invalid Frame Length")
        self.raw = raw
        self.points = []
        self._parse()
        if not self._valcrc():
            raise ValueError("Invalid CRC")

    def _valcrc(self):
        crc = 0
        for b in self.raw[0:-1]:
            crc = CRC_TABLE[(crc ^ b) & 0xFF]
        return crc == self.raw[-1]

    def _parse(self):
        START_ANGLE_OFFSET = 4
        START_POINTS_OFFSET = 6
        END_ANGLE_OFFSET = 42

        self.start_angle = (
            struct.unpack_from("<H", self.raw, START_ANGLE_OFFSET)[0] / 100.0
        )
        self.end_angle = struct.unpack_from("<H", self.raw, END_ANGLE_OFFSET)[0] / 100.0

        angle_diff = self.end_angle - self.start_angle
        if angle_diff < 0:
            angle_diff += 360.0

        point_count = 0
        next_point_offset = START_POINTS_OFFSET

        for _ in range(self.POINT_PER_PACK):
            dist = struct.unpack_from("<H", self.raw, next_point_offset)[0]
            intensity = self.raw[next_point_offset + 2]

            t = (
                point_count / (self.POINT_PER_PACK - 1)
                if self.POINT_PER_PACK > 1
                else 0
            )
            angle = self.start_angle + angle_diff * t

            self.points.append(LiDARPoint(dist, intensity, angle))
            next_point_offset += 3
            point_count += 1


class LiDARNode(Node):
    """
    ROS 2 node for LD19 LiDAR sensor data acquisition and publishing
    """

    def __init__(self) -> None:
        super().__init__("lidar_node")

        # Parameters
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 230400)
        self.declare_parameter("frame_id", "laser_frame")
        self.declare_parameter("scan_resolution_deg", 1.0)

        self.port = self.get_parameter("serial_port").value
        if sys.platform == "darwin":
            self.port = "/dev/cu.usbserial-0001"

        self.baud = self.get_parameter("baud_rate").value
        self.frame_id = self.get_parameter("frame_id").value

        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, "/lidar/scan", 10)

        # Serial
        self.ser = None
        self._setup_serial()

        # Buffers
        self.points_buffer = []
        self.last_angle = 0.0

        # Timer
        self.create_timer(0.01, self._read_loop)

        self.get_logger().info("LiDAR node initialized")

    def _declare_parameters(self) -> None:
        pass  # Done in init for brevity

    def _setup_serial(self) -> None:
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"Connected to LiDAR on {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to LiDAR: {e}")

    def _read_loop(self) -> None:
        if not self.ser:
            return

        try:
            while self.ser.in_waiting >= LD19Packet.FRAME_SIZE:
                b = self.ser.read(1)
                if b[0] != LD19Packet.HEADER:
                    continue

                rest = self.ser.read(LD19Packet.FRAME_SIZE - 1)
                if len(rest) != LD19Packet.FRAME_SIZE - 1:
                    continue

                try:
                    packet = LD19Packet(b + rest)
                    self._process_packet(packet)
                except ValueError:
                    continue

        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

    def _process_packet(self, packet: LD19Packet) -> None:
        # Check wrap around
        if packet.start_angle < self.last_angle - 100.0:
            self._publish_scan()
            self.points_buffer = []

        self.last_angle = packet.end_angle
        self.points_buffer.extend(packet.points)

    def _publish_scan(self) -> None:
        if not self.points_buffer:
            return

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = math.radians(
            self.get_parameter("scan_resolution_deg").value
        )
        msg.range_min = 0.1
        msg.range_max = 12.0

        num_bins = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        msg.ranges = [float("inf")] * num_bins
        msg.intensities = [0.0] * num_bins

        for pt in self.points_buffer:
            angle_rad = math.radians(pt.angle_deg)
            # Normalize
            while angle_rad >= 2 * math.pi:
                angle_rad -= 2 * math.pi
            while angle_rad < 0:
                angle_rad += 2 * math.pi

            bin_idx = int(angle_rad / msg.angle_increment)
            if 0 <= bin_idx < num_bins:
                dist_m = pt.distance_mm / 1000.0
                if dist_m < msg.ranges[bin_idx]:
                    msg.ranges[bin_idx] = dist_m
                    msg.intensities[bin_idx] = float(pt.intensity)

        self.scan_pub.publish(msg)

    def destroy_node(self) -> bool:
        if self.ser:
            self.ser.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LiDARNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
