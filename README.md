# DiffBot

ROS 2 Humble package for a physical differential-drive robot with:

- custom `ros2_control` hardware interface
- RealSense RGB-D camera
- RPLidar
- RTAB-Map SLAM
- Nav2
- wheel odometry, camera IMU, and external ICM-20948 IMU

## Build

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
rosdep install --from-paths . --ignore-src -r -y
colcon build --packages-select diffbot
source install/setup.bash
```

## Run On The Robot

The full launch expects the robot hardware and these device names:

- `/dev/imu-encoder-node`
- `/dev/motor-controller`
- `/dev/rplidar`

Run:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
source install/setup.bash
ros2 launch diffbot diffbot.launch.py
```

## Firmware And Udev

Firmware lives in `embedded/`:

- `imu-encoder-node/` publishes encoder ticks, IMU, and magnetometer data.
- `motor-controller/` receives serial PWM commands.

The helper scripts in `embedded/` are for the target robot host. They may upload
firmware, run `git pull`, or write udev rules with `sudo`, so review them before
running on another machine.

## License

Apache-2.0. See `LICENSE`.
