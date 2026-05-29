# Agent Instructions

## Project Overview

This repository contains a ROS 2 robot project for a physical differential-drive
DiffBot-like robot. The target robot stack is ROS 2 Humble and includes:

- `ros2_control` hardware for wheel encoders, motor PWM commands, and an
  external ICM-20948 IMU connected through a NodeMCU.
- RTAB-Map SLAM/odometry, RealSense RGB-D camera input, RPLidar scan input,
  `robot_localization` EKF, Nav2, and IMU filtering.
- ESP8266/NodeMCU Arduino firmware for the motor controller and
  IMU/encoder node.
- FreeCAD CAD files for the physical robot.

## Repository Layout

- `ros2_ws/`: ROS 2 package source for the `diffbot` package.
- `ros2_ws/hardware/`: C++ `ros2_control` hardware plugin.
- `ros2_ws/bringup/launch/`: main robot, SLAM, and navigation launch files.
- `ros2_ws/bringup/config/`: controller, EKF, Nav2, RTAB-Map, and IMU filter
  parameters.
- `ros2_ws/description/`: URDF/Xacro robot description.
- `embedded/`: Arduino firmware, udev rules, and hardware deployment helpers.
- `scripts/`: local serial utilities and teleop helpers.
- `robot_specifications/`: living robot memory and operating context for agents.
- `cad/`: binary FreeCAD design files. Do not edit these unless explicitly asked.

## Robot Memory

Before using ROS MCP, connecting through rosbridge, or issuing any robot command,
read `robot_specifications/diffbot.txt`. Treat it as the current project-local
robot memory for identity, connection details, control surfaces, movement policy,
sensor behavior, localization lessons, and tested movement recipes.

Keep this memory file updated whenever new movement behaviors, constraints,
reliable procedures, limits, or failure modes are learned. It is a plain text
agent memory file and is not currently a ros-mcp verified robot specification.

## Build

Use the target ROS distro when validating robot behavior:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
rosdep install --from-paths . --ignore-src -r -y
colcon build --packages-select diffbot
```

This development host may not match the target robot exactly. If `/opt/ros/humble`
is unavailable, report that clearly and treat builds from another ROS distro as
smoke builds only.

## Hardware And Launch Caution

- Do not launch the full robot stack unless the user confirms the robot hardware
  is connected and ready.
- The main launch file opens physical devices such as `/dev/imu-encoder-node`,
  `/dev/motor-controller`, and `/dev/rplidar`.
- The scripts in `embedded/*redeploy.sh` upload firmware and currently perform a
  `git pull`; do not run them casually during repository cleanup.
- `embedded/apply-udev-rules.sh` writes system udev rules with `sudo`; do not run
  it without explicit user approval.
- The checked-in udev rules are host/topology-specific. Treat them as an example
  unless the user says this is the target robot host.

## ROS Interfaces To Preserve

The hardware plugin currently publishes:

- `/imu/data_raw` as `sensor_msgs/msg/Imu`.
- `/imu/mag` as `sensor_msgs/msg/MagneticField` when magnetometer fields are
  present in the serial line.
- `/diffbot_hw_debug` as `std_msgs/msg/Float64MultiArray` when debug telemetry is
  enabled.

The external IMU frame is configured as `imu_link`. The robot base frame is
generally `base_footprint`, while some Nav2 parameters still reference
`base_link`; check frame consistency carefully before changing localization or
navigation behavior.

## Development Guidelines

- Keep SLAM, EKF, IMU, Nav2, and firmware experiments separable. Avoid changing
  multiple layers in one step unless the user asks for an integrated change.
- When adding or removing launch-time dependencies, update `ros2_ws/package.xml`
  in the same change.
- Prefer launch arguments for optional hardware subsystems over commenting nodes
  in and out.
- Keep robot-specific calibration values documented near the config or code that
  uses them.
- Avoid broad rewrites of the hardware plugin. It controls physical motors; make
  small, reviewable changes and validate parsing, units, signs, and timeout
  behavior.
- Do not commit generated ROS workspace outputs such as `build/`, `install/`, or
  `log/`.
