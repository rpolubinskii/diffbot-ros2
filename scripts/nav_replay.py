#!/usr/bin/env python3
"""Replay a waypoint YAML as NavigateToPose goals."""
import argparse
import math
import signal
import subprocess
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# Avoid camera image topics; they starve RTAB-Map on this Jetson.
RECORD_TOPICS = [
    "/tf", "/tf_static", "/scan", "/odom", "/diffbot_base_controller/odom",
    "/dynamic_joint_states", "/imu/data_body", "/imu/data_raw", "/imu/mag",
    "/camera/camera/imu", "/rtabmap/icp_odom", "/rtabmap/icp_odom_reweighted",
    "/rtabmap/icp_odom_info", "/info", "/mapGraph", "/map",
]


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class RouteReplayer(Node):
    def __init__(self, frame_id: str):
        super().__init__("nav_replay")
        self.frame_id = frame_id
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def send(self, x: float, y: float, yaw: float, timeout_sec: float = 120.0) -> bool:
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("NavigateToPose action server not available")
            return False
        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = self.frame_id
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = x, y
        qx, qy, qz, qw = yaw_to_quat(yaw)
        p.pose.orientation.x, p.pose.orientation.y = qx, qy
        p.pose.orientation.z, p.pose.orientation.w = qz, qw
        goal.pose = p

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"goal ({x:.2f},{y:.2f},{yaw:.2f}) REJECTED")
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            self.get_logger().error(f"goal ({x:.2f},{y:.2f}) TIMEOUT after {timeout_sec}s; canceling")
            handle.cancel_goal_async()
            return False
        status = result_future.result().status
        ok = status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(
            f"goal ({x:.2f},{y:.2f},{yaw:.2f}) -> {'SUCCEEDED' if ok else f'status {status}'}")
        return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("route", help="waypoint YAML file")
    ap.add_argument("--record", metavar="BAGNAME", help="record a bag to BAGNAME during replay")
    ap.add_argument("--settle", type=float, default=3.0,
                    help="seconds to hold still after EACH SUCCESSFUL goal so "
                         "icp_odometry/rtabmap settle at rest (mapping + closure quality); "
                         "0 to disable")
    ap.add_argument("--goal-timeout", type=float, default=120.0, help="per-goal timeout (s)")
    args = ap.parse_args()

    with open(args.route) as f:
        route = yaml.safe_load(f)
    frame_id = route.get("frame_id", "map")
    waypoints = route["waypoints"]
    if not waypoints:
        print("no waypoints in route", file=sys.stderr)
        sys.exit(1)

    bag_proc = None
    if args.record:
        print(f"[replay] recording bag -> {args.record}")
        bag_proc = subprocess.Popen(["ros2", "bag", "record", "-o", args.record, *RECORD_TOPICS])
        time.sleep(2.0)

    rclpy.init()
    node = RouteReplayer(frame_id)
    failed = 0
    try:
        for i, wp in enumerate(waypoints):
            print(f"[replay] {i + 1}/{len(waypoints)}: x={wp['x']:.2f} y={wp['y']:.2f} yaw={wp['yaw']:.2f}")
            if node.send(float(wp["x"]), float(wp["y"]), float(wp["yaw"]), args.goal_timeout):
                if args.settle > 0:
                    node.get_logger().info(f"settling {args.settle:.1f}s at goal {i + 1}")
                    time.sleep(args.settle)
            else:
                failed += 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if bag_proc is not None:
            print("[replay] stopping bag recorder")
            bag_proc.send_signal(signal.SIGINT)
            try:
                bag_proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                bag_proc.kill()

    print(f"[replay] done: {len(waypoints) - failed}/{len(waypoints)} goals succeeded")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
