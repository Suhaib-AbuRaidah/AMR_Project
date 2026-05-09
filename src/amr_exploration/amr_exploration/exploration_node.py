#!/usr/bin/env python3

import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Explore(Node):
    def __init__(self):
        super().__init__("explore")

        # Parameters
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("linear_speed", 0.35)
        self.declare_parameter("turn_speed", 0.75)
        self.declare_parameter("safe_distance", 0.85)
        self.declare_parameter("stop_distance", 0.35)
        self.declare_parameter("front_angle_deg", 35.0)
        self.declare_parameter("side_angle_deg", 80.0)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.safe_distance = float(self.get_parameter("safe_distance").value)
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.front_angle = math.radians(float(self.get_parameter("front_angle_deg").value))
        self.side_angle = math.radians(float(self.get_parameter("side_angle_deg").value))

        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            scan_qos,
        )

        self.latest_scan = None
        self.last_scan_time = None

        # Random exploration bias helps the robot not repeat exactly the same path
        self.random_turn_bias = 0.0
        self.next_bias_time = time.time() + 3.0

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Wander SLAM node started.")
        self.get_logger().info(f"Subscribing to: {self.scan_topic}")
        self.get_logger().info(f"Publishing to: {self.cmd_vel_topic}")

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.last_scan_time = self.get_clock().now()

    def get_sector_min(self, scan: LaserScan, angle_min: float, angle_max: float) -> float:
        """
        Returns the minimum valid range in a sector.
        Angles are in radians, relative to the robot front.
        """
        ranges = scan.ranges
        valid_ranges = []

        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            if r < scan.range_min or r > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment

            if angle_min <= angle <= angle_max:
                valid_ranges.append(r)

        if not valid_ranges:
            return scan.range_max

        return min(valid_ranges)

    def publish_cmd(self, linear_x: float, angular_z: float):
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def control_loop(self):
        if self.latest_scan is None:
            self.stop_robot()
            return

        # Stop if scan data is old
        now = self.get_clock().now()
        if self.last_scan_time is None:
            self.stop_robot()
            return

        age = (now - self.last_scan_time).nanoseconds * 1e-9
        if age > 1.0:
            self.get_logger().warn("No recent /scan data. Stopping robot.")
            self.stop_robot()
            return

        scan = self.latest_scan

        # Main sectors
        front_min = self.get_sector_min(scan, -self.front_angle, self.front_angle)

        left_min = self.get_sector_min(
            scan,
            math.radians(25.0),
            self.side_angle,
        )

        right_min = self.get_sector_min(
            scan,
            -self.side_angle,
            math.radians(-25.0),
        )

        front_left_min = self.get_sector_min(
            scan,
            math.radians(0.0),
            self.front_angle,
        )

        front_right_min = self.get_sector_min(
            scan,
            -self.front_angle,
            math.radians(0.0),
        )

        # Update random exploration bias every few seconds
        if time.time() > self.next_bias_time:
            self.random_turn_bias = random.uniform(-0.25, 0.25)
            self.next_bias_time = time.time() + random.uniform(2.5, 5.0)

        cmd_linear = 0.0
        cmd_angular = 0.0

        # Emergency: very close obstacle
        if front_min < self.stop_distance:
            cmd_linear = -0.08

            # Turn toward the more open side
            if front_left_min < front_right_min:
                cmd_angular = -self.turn_speed
            else:
                cmd_angular = self.turn_speed

        # Obstacle ahead: turn away
        elif front_min < self.safe_distance:
            cmd_linear = 0.05

            if front_left_min < front_right_min:
                cmd_angular = -self.turn_speed
            else:
                cmd_angular = self.turn_speed

        # Path is clear: move forward, with mild wall balancing and random bias
        else:
            cmd_linear = self.linear_speed

            # Positive angular_z turns left.
            # If right side is closer, turn left.
            # If left side is closer, turn right.
            wall_balance = 0.35 * (left_min - right_min)

            cmd_angular = wall_balance + self.random_turn_bias

            # Clamp angular speed
            cmd_angular = max(min(cmd_angular, 0.45), -0.45)

        self.publish_cmd(cmd_linear, cmd_angular)

        self.get_logger().info(
            f"front={front_min:.2f}, left={left_min:.2f}, right={right_min:.2f}, "
            f"v={cmd_linear:.2f}, w={cmd_angular:.2f}",
            throttle_duration_sec=1.0,
        )

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Explore()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
