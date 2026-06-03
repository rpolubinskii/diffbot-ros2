#!/usr/bin/env python3
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0.0:
        raise argparse.ArgumentTypeError("must be >= 0")
    return parsed


class RplidarMotorClient(Node):
    def __init__(self, start_service: str, stop_service: str) -> None:
        super().__init__("diffbot_rplidar_motor")
        self.start_client = self.create_client(Empty, start_service)
        self.stop_client = self.create_client(Empty, stop_service)

    def wait_for_named_service(self, client, service_name: str, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.2):
                return True
        self.get_logger().error("Service %s is not available", service_name)
        return False

    def call_empty(self, client, service_name: str, timeout: float) -> bool:
        if not self.wait_for_named_service(client, service_name, timeout):
            return False

        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            self.get_logger().error("Timed out calling %s", service_name)
            return False

        exc = future.exception()
        if exc is not None:
            self.get_logger().error("Service %s failed: %s", service_name, exc)
            return False

        self.get_logger().info("Called %s", service_name)
        return True

    def start_motor(self, timeout: float) -> bool:
        return self.call_empty(self.start_client, self.start_client.srv_name, timeout)

    def stop_motor(self, timeout: float) -> bool:
        return self.call_empty(self.stop_client, self.stop_client.srv_name, timeout)


def wait_for_scan(node: Node, topic: str, timeout: float) -> bool:
    if timeout <= 0.0:
        return True

    received = {"ok": False}

    def callback(_msg: LaserScan) -> None:
        received["ok"] = True

    sub = node.create_subscription(LaserScan, topic, callback, 10)
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if received["ok"]:
                node.get_logger().info("Received scan from %s", topic)
                return True
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(sub)

    node.get_logger().error("No scan received from %s within %.1f seconds", topic, timeout)
    return False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Start or stop the RPLidar motor through the rplidar_ros services."
    )
    parser.add_argument("action", choices=["on", "off", "restart"], help="Motor action to perform")
    parser.add_argument("--start-service", default="/start_motor", help="RPLidar start service")
    parser.add_argument("--stop-service", default="/stop_motor", help="RPLidar stop service")
    parser.add_argument("--timeout", type=positive_float, default=5.0, help="Service timeout in seconds")
    parser.add_argument(
        "--restart-delay",
        type=positive_float,
        default=1.0,
        help="Delay between stop and start for the restart action",
    )
    parser.add_argument("--scan-topic", default="/scan", help="LaserScan topic used for start verification")
    parser.add_argument(
        "--scan-timeout",
        type=positive_float,
        default=2.0,
        help="Seconds to wait for a scan after on/restart; set 0 to skip",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    rclpy.init()
    node = RplidarMotorClient(args.start_service, args.stop_service)
    try:
        if args.action == "off":
            return 0 if node.stop_motor(args.timeout) else 1

        if args.action == "restart":
            if not node.stop_motor(args.timeout):
                return 1
            time.sleep(args.restart_delay)

        if not node.start_motor(args.timeout):
            return 1

        return 0 if wait_for_scan(node, args.scan_topic, args.scan_timeout) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
