#!/usr/bin/env python3
import argparse
import math
import statistics
import time
from dataclasses import dataclass, field
from types import SimpleNamespace

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField


DEFAULT_IMU_TOPICS = ["/camera/camera/imu", "/imu/data_body", "/imu/data_raw", "/imu/external/data_body"]
DEFAULT_ODOM_TOPICS = ["/diffbot_base_controller/odom", "/rtabmap/icp_odom", "/odom"]
DEFAULT_MAG_TOPICS = ["/imu/mag"]


@dataclass
class AngleSample:
    t: float
    yaw: float


@dataclass
class ScalarSample:
    t: float
    value: float


@dataclass
class TopicStats:
    yaws: list[AngleSample] = field(default_factory=list)
    gyro_z: list[ScalarSample] = field(default_factory=list)
    mag_norm: list[ScalarSample] = field(default_factory=list)
    mag_heading: list[AngleSample] = field(default_factory=list)


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def unwrap(values: list[float]) -> list[float]:
    if not values:
        return []
    unwrapped = [values[0]]
    for value in values[1:]:
        delta = value - unwrapped[-1]
        while delta > math.pi:
            value -= 2.0 * math.pi
            delta = value - unwrapped[-1]
        while delta < -math.pi:
            value += 2.0 * math.pi
            delta = value - unwrapped[-1]
        unwrapped.append(value)
    return unwrapped


def mean(values: list[float]) -> float:
    return statistics.fmean(values) if values else math.nan


def stdev(values: list[float]) -> float:
    return statistics.stdev(values) if len(values) > 1 else math.nan


def summarize_angles(samples: list[AngleSample]) -> str:
    if len(samples) < 2:
        return "samples=<2"

    times = [sample.t for sample in samples]
    yaws = unwrap([sample.yaw for sample in samples])
    duration = times[-1] - times[0]
    if duration <= 0.0:
        return f"samples={len(samples)} duration=0"

    delta_deg = math.degrees(yaws[-1] - yaws[0])
    rate_dps = delta_deg / duration
    span_deg = math.degrees(max(yaws) - min(yaws))
    return (
        f"samples={len(samples)} duration={duration:.1f}s "
        f"delta={delta_deg:+.3f}deg rate={rate_dps:+.4f}deg/s span={span_deg:.3f}deg"
    )


def summarize_scalars(samples: list[ScalarSample], unit: str) -> str:
    if not samples:
        return "samples=0"
    values = [sample.value for sample in samples]
    return (
        f"samples={len(samples)} mean={mean(values):+.6f}{unit} "
        f"std={stdev(values):.6f}{unit} min={min(values):+.6f}{unit} max={max(values):+.6f}{unit}"
    )


def has_orientation(msg: Imu) -> bool:
    covariance = msg.orientation_covariance
    return len(covariance) == 0 or covariance[0] >= 0.0


class YawDriftAnalyzer(Node):
    def __init__(self, imu_topics: list[str], odom_topics: list[str], mag_topics: list[str]):
        super().__init__("yaw_drift_analyzer")
        self.stats: dict[str, TopicStats] = {}
        self.expected_types = {
            **{topic: "sensor_msgs/msg/Imu" for topic in imu_topics},
            **{topic: "nav_msgs/msg/Odometry" for topic in odom_topics},
            **{topic: "sensor_msgs/msg/MagneticField" for topic in mag_topics},
        }

        for topic in imu_topics:
            self.stats[topic] = TopicStats()
            self.create_subscription(Imu, topic, self._imu_cb(topic), qos_profile_sensor_data)

        for topic in odom_topics:
            self.stats[topic] = TopicStats()
            self.create_subscription(Odometry, topic, self._odom_cb(topic), qos_profile_sensor_data)

        for topic in mag_topics:
            self.stats[topic] = TopicStats()
            self.create_subscription(MagneticField, topic, self._mag_cb(topic), qos_profile_sensor_data)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _imu_cb(self, topic: str):
        def cb(msg: Imu):
            t = self._now()
            self.stats[topic].gyro_z.append(ScalarSample(t, msg.angular_velocity.z))
            if has_orientation(msg):
                self.stats[topic].yaws.append(AngleSample(t, yaw_from_quaternion(msg.orientation)))

        return cb

    def _odom_cb(self, topic: str):
        def cb(msg: Odometry):
            self.stats[topic].yaws.append(AngleSample(self._now(), yaw_from_quaternion(msg.pose.pose.orientation)))

        return cb

    def _mag_cb(self, topic: str):
        def cb(msg: MagneticField):
            t = self._now()
            x = msg.magnetic_field.x
            y = msg.magnetic_field.y
            z = msg.magnetic_field.z
            self.stats[topic].mag_norm.append(ScalarSample(t, math.sqrt(x * x + y * y + z * z)))
            self.stats[topic].mag_heading.append(AngleSample(t, math.atan2(y, x)))

        return cb

    def topic_warnings(self) -> list[str]:
        discovered = dict(self.get_topic_names_and_types())
        warnings = []
        for topic, expected_type in sorted(self.expected_types.items()):
            types = discovered.get(topic)
            if types is None:
                warnings.append(f"{topic}: not discovered")
            elif expected_type not in types:
                warnings.append(f"{topic}: expected {expected_type}, discovered {', '.join(types)}")
        return warnings


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Measure stationary yaw drift and IMU gyro/magnetometer statistics."
    )
    parser.add_argument("--duration", type=float, default=60.0, help="Collection duration in seconds")
    parser.add_argument("--robot-ip", default="192.168.0.147", help="Robot label for the report")
    parser.add_argument("--self-test", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument(
        "--imu-topic",
        action="append",
        default=None,
        help="IMU topic to sample; may be repeated",
    )
    parser.add_argument(
        "--odom-topic",
        action="append",
        default=None,
        help="Odometry topic to sample; may be repeated",
    )
    parser.add_argument(
        "--mag-topic",
        action="append",
        default=None,
        help="Magnetometer topic to sample; may be repeated. Defaults to /imu/mag.",
    )
    parser.add_argument("--no-mag", action="store_true", help="Skip magnetometer subscriptions")
    args = parser.parse_args()

    if args.self_test:
        assert has_orientation(SimpleNamespace(orientation_covariance=[]))
        assert has_orientation(SimpleNamespace(orientation_covariance=[0.0] * 9))
        assert not has_orientation(SimpleNamespace(orientation_covariance=[-1.0] + [0.0] * 8))
        print("self-test ok")
        return 0

    if args.duration <= 0.0:
        parser.error("--duration must be positive")

    imu_topics = args.imu_topic if args.imu_topic is not None else DEFAULT_IMU_TOPICS
    odom_topics = args.odom_topic if args.odom_topic is not None else DEFAULT_ODOM_TOPICS
    mag_topics = [] if args.no_mag else (args.mag_topic if args.mag_topic is not None else DEFAULT_MAG_TOPICS)

    rclpy.init()
    node = YawDriftAnalyzer(imu_topics, odom_topics, mag_topics)
    deadline = time.monotonic() + args.duration

    print(f"Sampling robot {args.robot_ip} for {args.duration:.1f}s...")
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        print("\nYaw drift:")
        for topic, stats in sorted(node.stats.items()):
            if stats.yaws:
                print(f"  {topic}: {summarize_angles(stats.yaws)}")

        print("\nGyro Z:")
        for topic, stats in sorted(node.stats.items()):
            if stats.gyro_z:
                print(f"  {topic}: {summarize_scalars(stats.gyro_z, 'rad/s')}")

        print("\nMagnetometer:")
        for topic, stats in sorted(node.stats.items()):
            if stats.mag_norm:
                print(f"  {topic} norm: {summarize_scalars(stats.mag_norm, 'T')}")
                print(f"  {topic} heading: {summarize_angles(stats.mag_heading)}")

        no_data = [topic for topic, stats in sorted(node.stats.items()) if not any((
            stats.yaws,
            stats.gyro_z,
            stats.mag_norm,
            stats.mag_heading,
        ))]
        if no_data:
            print("\nNo samples:")
            for topic in no_data:
                print(f"  {topic}")

        warnings = node.topic_warnings()
        if warnings:
            print("\nTopic warnings:")
            for warning in warnings:
                print(f"  {warning}")

        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
