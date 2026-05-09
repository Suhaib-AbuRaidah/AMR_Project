#!/usr/bin/env python3

import math
import random
import csv
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan


class Explore(Node):
    def __init__(self):
        super().__init__("explore")

        # Parameters
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter(
            "image_topic",
            "/world/car_world/model/vehicle_blue/link/chassis/sensor/front_cam/image",
        )
        self.declare_parameter("qr_output_file", "qr_detections.csv")
        self.declare_parameter("linear_speed", 0.35)
        self.declare_parameter("turn_speed", 0.75)
        self.declare_parameter("safe_distance", 0.85)
        self.declare_parameter("stop_distance", 0.35)
        self.declare_parameter("front_angle_deg", 35.0)
        self.declare_parameter("side_angle_deg", 80.0)
        self.declare_parameter("camera_process_rate", 5.0)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.image_topic = self.get_parameter("image_topic").value
        self.qr_output_file = os.path.expanduser(
            self.get_parameter("qr_output_file").value
        )
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.safe_distance = float(self.get_parameter("safe_distance").value)
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.front_angle = math.radians(float(self.get_parameter("front_angle_deg").value))
        self.side_angle = math.radians(float(self.get_parameter("side_angle_deg").value))
        self.camera_process_period = 1.0 / max(
            float(self.get_parameter("camera_process_rate").value),
            0.1,
        )

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            sensor_qos,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            sensor_qos,
        )

        self.latest_scan = None
        self.last_scan_time = None
        self.latest_pose = None
        self.latest_pose_time = None

        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()
        self.qr_detections = {}
        self.last_image_process_time = 0.0
        self.prepare_qr_output_file()

        # Random exploration bias helps the robot not repeat exactly the same path
        self.random_turn_bias = 0.0
        self.next_bias_time = time.time() + 3.0

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Wander SLAM node started.")
        self.get_logger().info(f"Subscribing to: {self.scan_topic}")
        self.get_logger().info(f"Subscribing to: {self.odom_topic}")
        self.get_logger().info(f"Subscribing to: {self.image_topic}")
        self.get_logger().info(f"Publishing to: {self.cmd_vel_topic}")
        self.get_logger().info(f"Saving QR detections to: {self.qr_output_file}")

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.last_scan_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        self.latest_pose = msg.pose.pose
        self.latest_pose_time = self.get_clock().now()

    def prepare_qr_output_file(self):
        output_dir = os.path.dirname(self.qr_output_file)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        if os.path.exists(self.qr_output_file):
            return

        with open(self.qr_output_file, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([
                "qr_data",
                "x",
                "y",
                "z",
                "qx",
                "qy",
                "qz",
                "qw",
                "stamp_sec",
                "stamp_nanosec",
            ])

    def image_callback(self, msg: Image):
        now = time.time()
        if now - self.last_image_process_time < self.camera_process_period:
            return

        self.last_image_process_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Could not convert camera image: {exc}")
            return

        qr_payloads = self.detect_qr_codes(frame)
        for qr_data in qr_payloads:
            self.store_qr_detection(qr_data, msg)

    def detect_qr_codes(self, frame):
        detected_payloads = []

        try:
            found, decoded_info, _, _ = self.qr_detector.detectAndDecodeMulti(frame)
            if found:
                detected_payloads.extend(
                    data.strip() for data in decoded_info if data and data.strip()
                )
        except cv2.error:
            decoded_data, _, _ = self.qr_detector.detectAndDecode(frame)
            if decoded_data:
                detected_payloads.append(decoded_data.strip())

        return sorted(set(detected_payloads))

    def store_qr_detection(self, qr_data: str, image_msg: Image):
        if qr_data in self.qr_detections:
            return

        if self.latest_pose is None:
            self.get_logger().warn(
                f"Detected QR '{qr_data}' but no odometry pose is available yet.",
                throttle_duration_sec=2.0,
            )
            return

        pose = self.latest_pose
        position = pose.position
        orientation = pose.orientation

        self.qr_detections[qr_data] = {
            "x": position.x,
            "y": position.y,
            "z": position.z,
            "qx": orientation.x,
            "qy": orientation.y,
            "qz": orientation.z,
            "qw": orientation.w,
        }

        with open(self.qr_output_file, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([
                qr_data,
                f"{position.x:.6f}",
                f"{position.y:.6f}",
                f"{position.z:.6f}",
                f"{orientation.x:.6f}",
                f"{orientation.y:.6f}",
                f"{orientation.z:.6f}",
                f"{orientation.w:.6f}",
                image_msg.header.stamp.sec,
                image_msg.header.stamp.nanosec,
            ])

        self.get_logger().info(
            f"Stored QR '{qr_data}' at x={position.x:.2f}, y={position.y:.2f}, "
            f"z={position.z:.2f}"
        )

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
