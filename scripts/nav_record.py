#!/usr/bin/env python3
"""Record nav goals from /goal_pose into a replayable route YAML.

Subscribes to /goal_pose (geometry_msgs/PoseStamped) and appends each goal as
{x, y, yaw} to a YAML file in the format nav_replay.py consumes. Drive a vetted
return-to-origin run sending goals to /goal_pose (e.g. rviz "2D Nav Goal"), then
Ctrl-C; the file is the route to replay.

    python3 nav_record.py route.yaml
    python3 nav_record.py route.yaml --topic /goal_pose --frame map

IMPORTANT capture-point note: this robot's agent sends goals via the
/navigate_to_pose ACTION, whose goal pose is NOT on a subscribable topic. If your
vetted run is driven by the agent (not rviz/goal_pose), record AGENT-SIDE instead:
the agent already builds each {x,y,yaw}, so have it append to the same YAML:
    frame_id: map
    waypoints:
      - {x: ..., y: ..., yaw: ...}
nav_replay.py reads either source identically.
"""
import argparse
import math

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GoalRecorder(Node):
    def __init__(self, out_path: str, topic: str, frame: str):
        super().__init__("nav_record")
        self.out_path = out_path
        self.frame = frame
        self.waypoints = []
        self.create_subscription(PoseStamped, topic, self.on_goal, 10)
        self.get_logger().info(f"recording {topic} -> {out_path} (Ctrl-C to finish)")

    def on_goal(self, msg: PoseStamped):
        wp = {
            "x": round(float(msg.pose.position.x), 4),
            "y": round(float(msg.pose.position.y), 4),
            "yaw": round(quat_to_yaw(msg.pose.orientation), 4),
        }
        self.waypoints.append(wp)
        self.get_logger().info(f"#{len(self.waypoints)}: x={wp['x']} y={wp['y']} yaw={wp['yaw']}")
        self.flush()

    def flush(self):
        with open(self.out_path, "w") as f:
            yaml.safe_dump({"frame_id": self.frame, "waypoints": self.waypoints},
                           f, default_flow_style=None, sort_keys=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", help="output route YAML")
    ap.add_argument("--topic", default="/goal_pose")
    ap.add_argument("--frame", default="map")
    args = ap.parse_args()

    rclpy.init()
    node = GoalRecorder(args.out, args.topic, args.frame)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"saved {len(node.waypoints)} waypoints to {args.out}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
