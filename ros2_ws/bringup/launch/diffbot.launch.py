# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch_ros.descriptions
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "manage_lidar_standby",
            default_value="false",
            description="Pause RTAB-Map/ICP and stop the RPLidar motor when Nav2 has no active goals.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "standby_scan_topic",
            default_value="/diffbot/standby_scan",
            description="Managed scan topic used by RTAB-Map and ICP when lidar standby is enabled.",
        )
    )

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    manage_lidar_standby = LaunchConfiguration("manage_lidar_standby")
    standby_scan_topic = LaunchConfiguration("standby_scan_topic")

    rplidar_pkg = get_package_share_directory('rplidar_ros')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("diffbot"), "urdf", "diffbot.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": launch_ros.descriptions.ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("diffbot"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description")
        ],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nav2_params = PathJoinSubstitution(
        [
            FindPackageShare("diffbot"),
            "config",
            "nav2_params.yaml",
        ]
    )

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_pkg, "launch", "rplidar_a1_launch.py")
        ),
        launch_arguments={
            "serial_baudrate": "115200",
            "frame_id": "scan",
            "serial_port": "/dev/rplidar"
        }.items()
    )

    ekf_params = PathJoinSubstitution(
        [
            FindPackageShare("diffbot"),
            "config",
            "ekf.yaml",
        ]
    )

    imu_filter_params = PathJoinSubstitution(
        [
            FindPackageShare("diffbot"),
            "config",
            "imu_filter.yaml",
        ]
    )
    external_imu_filter_params = PathJoinSubstitution(
        [
            FindPackageShare("diffbot"),
            "config",
            "external_imu_filter.yaml",
        ]
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        parameters=[ekf_params],
        remappings=[
            ("odometry/filtered", "/odom"),
        ],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('diffbot'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': nav2_params
        }.items(),
    )

    rtabmap_parameters = [{
        'frame_id': 'base_footprint',
        'subscribe_depth': True,
        'subscribe_scan': True,
        'subscribe_odom_info': False,
        'approx_sync': True,
        'topic_queue_size': 30,
        'sync_queue_size': 30,
        'wait_imu_to_init': True,
        'Reg/Force3DoF': 'true',
        # GHOSTING -- loop-closure registration strategy. History:
        #  - Reg/Strategy=0 (Vis, default): bag diffbot_MaxTranslation rejected
        #    ALL 28 closure candidates, 25 at "0/20 inliers". Bag-of-words FINDS
        #    revisits (48-144 word matches) but the visual RANSAC gets 0 geometric
        #    inliers at 640x360 -- too few crisp features on monochrome walls. No
        #    closures -> drift never corrected -> walls mapped twice at offset
        #    poses (the "copy of the wall hanging in the air"), which the
        #    lidar-built /map carries into Nav2's static costmap layer as a solid.
        #  - Reg/Strategy=1 (Icp): bag diffbot_icp_loop_closure cut rejections
        #    28->12 BUT made the map WORSE -- a corridor got a "shifted brother".
        #    Pure 2D-lidar ICP is under-constrained along a corridor's long axis
        #    (aperture problem): parallel walls align but the match slides
        #    lengthwise, so the proximity correction duplicated the corridor.
        #  - Reg/Strategy=2 (VisIcp, current): visual feature match supplies the
        #    loop-closure transform + the longitudinal constraint pure ICP lacks,
        #    then lidar ICP refines it. Tested in bag diffbot_vis_loop_closure:
        #    NO doubled wall (corridor slide gone) and CPU fine (rtabmap delay
        #    ~0.4 s), BUT still ~0 applied closures -> drift uncorrected -> /map
        #    quality issues remain. Cause (from the log): candidates get HEALTHY
        #    2D appearance matches (54-85) but ~0 3D inliers -- the visual
        #    geometric verification fails because RealSense depth at the matched
        #    keypoints is unreliable on this apartment's monochrome/far walls
        #    (NOT a resolution problem: camera was 1280x720 all along, see
        #    realsense args). The one closure that did pass (145<->91) was then
        #    rejected by RGBD/OptimizeMaxError (ratio 3.1 vs 3.0). Next levers if
        #    continuing on visual: cap feature depth to the reliable range
        #    (Kp/MaxDepth + Vis/MaxDepth ~3-4 m so far garbage-depth keypoints
        #    stop poisoning RANSAC), Vis/MinInliers 20->12, RGBD/OptimizeMaxError
        #    3->5. Kept Reg/Strategy=2 as the safest of {0 no closures, 1 corridor
        #    slide, 2 safe-but-currently-inert}.
        'Reg/Strategy': '2',
        # Proximity-by-space (default on) now also registers via lidar ICP -- this
        # closes drift on nearby revisits even when appearance recognition misses.
        'RGBD/ProximityBySpace': 'true',
        # Keep the graph anchored at its start (do NOT optimize-from-end, which
        # would let the whole map shift under a new closure).
        'RGBD/OptimizeFromGraphEnd': 'false',
        # Loop-closure ICP tolerances. This is the rtabmap SLAM node's OWN Icp/*
        # set -- separate from icp_odometry_parameters below; rtabmap defaults
        # (MaxCorrespondenceDistance ~0.1 m) are too tight to bridge the drift
        # present at a revisit, so widen the search a bit and require a modest
        # overlap before accepting a closure (guards against false closures in
        # self-similar geometry).
        'Icp/MaxCorrespondenceDistance': '0.3',
        'Icp/CorrespondenceRatio': '0.3',
        # SECOND GATE (bag diffbot_DetectionRate): 2 of the 30 rejected closures
        # actually PASSED the visual stage (>=12 inliers) and reached ICP
        # refinement, then died on libpointmatcher "limit out of bounds":
        #   237->274  tr 0.231394/0.2   (log line 3819)
        #   192->386  tr 0.207859/0.2   (log line 4696)
        # The SLAM node's Icp/MaxTranslation was still the default 0.2 m -- our
        # 0.5 m fix was applied ONLY to icp_odometry_parameters, not here. A loop
        # closure's translation IS the accumulated drift it corrects (~0.2-0.23 m
        # over a living-room loop), so the 0.2 m cap rejects exactly the useful
        # closures. Raise to 0.5 m (rotation 0.05-0.10 rad was well under the 0.78
        # default, so Icp/MaxRotation is left alone). Complements the visual fix
        # below: depth-quality gets MORE closures through the visual gate; this
        # stops the survivors dying at the ICP gate.
        'Icp/MaxTranslation': '0.5',
        # SURGICAL VISUAL-CLOSURE FIX (bag diffbot_vis_loop_closure showed good 2D
        # matches=54-85 but ~0 3D inliers -> visual geometric verification failing
        # on unreliable depth). The camera is a RealSense D455 (~95 mm baseline,
        # global-shutter RGB), reliable to ~6 m. By default Kp/MaxDepth and
        # Vis/MaxDepth are 0 (no limit), so keypoints beyond reliable range carry
        # garbage 3D positions that poison the PnP RANSAC -> 0 inliers. Cap both
        # to 5 m (inside the D455's reliable range) so only trustworthy-depth
        # keypoints feed loop-closure matching + registration, without discarding
        # the good mid-range features the D455 resolves well.
        'Kp/MaxDepth': '5.0',
        'Vis/MaxDepth': '5.0',
        # Ease the inlier gate slightly: a candidate already reached 16/20, and
        # 6/20, so 12 lets marginal-but-real closures through (still well above
        # noise). Raise back toward 20 if false closures appear.
        'Vis/MinInliers': '12',
        # 0-INLIER ROOT-CAUSE FIX (bag diffbot_DetectionRate, living room): the
        # rejected closures had STRONG 2D appearance overlap (matches 34-95; e.g.
        # 92->200/201/202 at 85/95/64 words) yet ~0 3D inliers. So 2D matching
        # WORKS -- the DEPTH at the matched keypoints is the problem. The default
        # GFTT detector picks CORNERS, which sit on object edges / depth
        # discontinuities where RealSense stereo depth is noisiest and "bleeds",
        # so those features carry valid-but-wrong 3D -> PnP finds no consistent
        # transform -> 0 inliers even in a feature-rich room. The depth CAP didn't
        # help because it's depth NOISE (at edges), not depth RANGE. Loosen the
        # PnP reprojection gate 2->4 px to tolerate that noise; paired with the
        # RealSense spatial depth filter now enabled in the realsense args, which
        # smooths the depth edges at the source. (Next lever if still weak:
        # Vis/FeatureType=8 GFTT/ORB or SIFT for better-localized features.)
        'Vis/PnPReprojError': '4.0',
        # The one closure that passed (145<->91) was rejected at error ratio 3.1
        # vs the default 3.0 -- but it was correcting REAL accumulated drift, so a
        # large residual against the drifted graph is expected. Raise the guard to
        # 5.0 so legitimate drift corrections survive. This is the riskiest knob
        # here (it's the last defense against a wrong closure warping the map) --
        # if the map folds/teleports after a closure, drop it back to 3.0 first.
        'RGBD/OptimizeMaxError': '5.0',
        # Capture more data WHILE MOVING. rtabmap filters input frames down to
        # this rate (default 1 Hz -> our log showed "Rate=1.00s"), dropping the
        # rest, so this -- NOT camera fps -- is the real "data density" knob. At
        # 2 Hz the graph gets ~2x the nodes during driving -> finer map + more
        # loop-closure opportunities. CPU: rtabmap processing was ~0.13-0.22 s per
        # iteration at 1 Hz; 2 Hz roughly doubles that thread load and the working
        # memory grows faster, so WATCH the "delay=" stat -- if it climbs toward
        # ~1 s, drop to 1.5.
        'Rtabmap/DetectionRate': '2'
    }]

    rtabmap_remappings = [
        # IMU = RealSense /imu/data_body, NOT the external ICM-20948
        # /imu/external/data_body. The external IMU is the worse source (gyro std
        # ~0.029 rad/s + vibration spikes, madgwick absolute yaw drift 30-81 deg,
        # magnetometer unusable indoors) and is no longer used by any consumer.
        # The whole SLAM layer (icp_odometry + rtabmap) now takes its gravity
        # vector and fast-spin rotation guess from the clean ~198 Hz RealSense
        # gyro -- the same source the EKF already trusts. /imu/data_body is in
        # camera_imu_frame, which is identity-rotated to base_footprint (static TF
        # chain base_footprint->base_link->camera_link->camera_gyro_frame->
        # camera_imu_frame is all rpy 0), and rtabmap/icp resolve the IMU frame
        # via TF regardless. The external_imu_filter/_transformer nodes still run
        # (so /imu/external/* stays recordable for future mag calibration) but
        # nothing consumes their output now.
        ('imu', '/imu/data_body'),
        ('odom', '/odom'),
        # ('odom_info', '/rtabmap/odom_info'),
        ('rgb/image', '/camera/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw')]

    rgbd_odometry_parameters = [{
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'publish_tf': False,
        'approx_sync': False,
        'topic_queue_size': 30,
        'sync_queue_size': 30,
        'wait_imu_to_init': True,
        'always_check_imu_tf': False
    }]

    rgbd_odometry_remappings = [
        ('imu', '/imu/data_body'),  # RealSense IMU (see rtabmap_remappings rationale)
        ('odom', '/rtabmap/odom'),
        ('odom_info', '/rtabmap/odom_info'),
        ('rgb/image', '/camera/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw')]

    icp_odometry_parameters = [{
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'publish_tf': False,
        'topic_queue_size': 30,
        'sync_queue_size': 30,
        # wait_imu_to_init must stay False: with True, icp_odometry seeds its
        # ENTIRE output frame from the external IMU's ABSOLUTE orientation at
        # startup. That orientation has no valid zero in the odom frame -- its
        # heading reference is (gyro-init OR magnetic north) + the -90 deg
        # imu_link mount -- so icp_odom is born rotated by a constant offset vs
        # the wheel/EKF frame (measured: -128.7 deg with mag on, ~-80 deg mag
        # off; icp's first message == /imu/external/data_body to <0.1 deg).
        # icp scan-matching rotation is itself good and drift-free, so we only
        # need to drop the bad absolute-yaw seed: start at identity (base frame
        # yaw 0, aligned with wheel/EKF). The IMU stays subscribed for gravity,
        # deskew, and the inter-scan rotation guess during fast spins.
        'wait_imu_to_init': False,
        'always_check_imu_tf': True,
        'Reg/Force3DoF': 'true',
        # Auto-recover from lost tracking, but DON'T over-reset.
        # History: ResetCountdown=0 caused a permanent-loss spiral -- ONE failed
        # registration (e.g. a scan dropped while icp_odometry starved for CPU
        # during a loop-closure graph optimization, delay ~0.14s) zeroed the
        # motion-model velocity, and every later frame then failed with
        # "RegistrationIcp cannot do registration with a null guess" FOREVER. So
        # it was set to 1 (reset after a single lost frame). But =1 turned out to
        # be too aggressive: EACH reset re-initializes (clears) the local scan
        # map, and a reset MID-SPIN leaves a ~400-point map that cannot register
        # rotation -> icp under-counts heading. Bag diffbot_no_external_imu_
        # 20260615_220146: a cluster of 3 resets in 1.5s during a spin lost ~90
        # deg, and the run ended 165 deg behind the gyro -> crash (icp tracked
        # fine BETWEEN resets; the resets were the damage).
        # Now that icp gets a clean rotation guess from the RealSense IMU
        # (/imu/data_body), the null-guess that motivated =1 should not recur, so
        # we coast through short transient losses (observed clusters were 1-3
        # frames) WITHOUT nuking the local map, and only reset if stuck for 5
        # consecutive frames. WATCH the icp_odometry log: if "null guess" spam
        # returns, the IMU guess is not feeding registration -> revert toward 1
        # and pursue the architectural route (gyro as heading authority).
        'Odom/ResetCountdown': '5',
        # Tolerate faster motion between scans. icp runs at ~7.5 Hz (134 ms), so
        # at 1 m/s the robot moves ~0.13 m per frame. The rtabmap default
        # Icp/MaxCorrespondenceDistance (~0.1 m) is SMALLER than that, so once the
        # robot drives fast the scan<->local-map point correspondences can no
        # longer be found and registration collapses. Data: bag diffbot_nav_
        # 20260615_230406, t=54.93s -- icp_translation spiked to 0.144 m/frame
        # (~1.07 m/s) and the inlier ratio cratered 0.58->0.11->0 (corr 265->50->0)
        # -> 5 lost frames -> reset -> robot drove into the wall it was speeding
        # toward. icp_rotation was only ~0.1-7 deg, so this was a fast STRAIGHT-LINE
        # failure, not a rotation one. Raise the search radius to ~0.3 m so icp can
        # associate points up to ~2 m/s. If it starts producing confidently-wrong
        # poses in cluttered/feature-poor spots (spurious matches), lower it.
        # Note: Icp/MaxTranslation (rtabmap default ~0.2 m, the per-frame motion it
        # will ACCEPT) is still above the 0.144 m seen here; raise it too only if
        # the robot will exceed ~1.5 m/s.
        'Icp/MaxCorrespondenceDistance': '0.3',
        # Also raise the per-frame translation ACCEPTANCE limit. rtabmap default
        # Icp/MaxTranslation=0.2 m REJECTS any registration whose translation
        # exceeds 0.2 m as "out of bounds", which nulls the velocity model and
        # triggers the null-guess spiral. Data: log diffbot_2026-06-16_000212,
        # 00:06:01 -- "libpointmatcher has failed: limit out of bounds:
        # rot 0.107/0.78  tr 0.207606/0.2" was the FIRST failure (rotation was
        # fine; translation 0.2076 m just crossed 0.2), then 8x "cannot do
        # registration with a null guess" + 2 auto-resets. The robot was driving
        # ~1.6 m/s (0.207 m / 0.13 s) and the constant-velocity guess
        # under-predicted (guess was only 0.017 m), so icp found the real ~0.21 m
        # match but the cap rejected it. 0.5 m accepts motion up to ~4 m/s at
        # 7.5 Hz (well above the robot's speed, even with a delayed frame) while
        # still rejecting true teleport glitches. Pair with MaxCorrespondenceDistance
        # above (the search radius must also cover the post-guess residual).
        # Complementary non-icp lever (user chose icp tuning): cap Nav2 max_vel_x
        # -- 1.6 m/s indoors is what keeps over-running the scan matcher.
        'Icp/MaxTranslation': '0.5'
    }]

    icp_odometry_remappings = [
        ('imu', '/imu/data_body'),  # RealSense IMU (see rtabmap_remappings rationale)
        ('odom', '/rtabmap/icp_odom'),
        ('odom_info', '/rtabmap/icp_odom_info'),
        ('scan', '/scan')]

    managed_icp_odometry_remappings = [
        ('imu', '/imu/data_body'),  # RealSense IMU (see rtabmap_remappings rationale)
        ('odom', '/rtabmap/icp_odom'),
        ('odom_info', '/rtabmap/icp_odom_info'),
        ('scan', standby_scan_topic)]

    # Make sure IR emitter is enabled
    depth_module = SetParameter(name='depth_module.emitter_enabled', value=1)

    # Launch camera driver
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py']),
        launch_arguments={
            'enable_color': 'true',
            'align_depth.enable': 'true',
            'enable_depth': 'true',
            'enable_sync': 'true',
            'enable_motion': 'false',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            # CORRECT param names: this realsense2_camera build uses
            # rgb_camera.color_profile / depth_module.depth_profile. The old
            # 'rgb_camera.profile' / 'depth_module.profile' were SILENTLY IGNORED
            # ("Parameter ... is not supported" in the log), so across every run
            # the camera ran at its DEFAULTS -- color 1280x720x30, depth
            # 848x480x30 -- NOT the 640x360 the config implied. Resolution was
            # therefore NEVER the visual-loop-closure bottleneck (always 720p).
            # fps choice (researched): rtabmap drops input frames to satisfy
            # Rtabmap/DetectionRate (see rtabmap_parameters), so fps above that
            # rate adds NO map data -- the data-density knob is DetectionRate, not
            # fps. But higher fps still helps two real things during motion: a
            # sharper frame is available to SELECT at each detection tick, and the
            # camera<->lidar approx_sync skew shrinks (~2.5 cm at 30 fps vs ~5 cm
            # at 15 fps @ 1.6 m/s). The Jetson already ran 720p@30 + depth 30 fine
            # (rtabmap delay ~0.4 s), so keep BOTH full res and 30 fps -- no
            # fps/res trade needed. (Odometry is lidar icp_odometry, unaffected by
            # camera fps; the "fast motion loses tracking" reports are for visual
            # rgbd_odometry, which is disabled here.)
            'rgb_camera.color_profile': '1280x720x30',
            'depth_module.depth_profile': '848x480x30'
        }.items(),
    )

    imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        output="screen",
        parameters=[imu_filter_params],
        remappings=[
            ("imu/data_raw", "/camera/camera/imu"),
            ("imu/data", "/imu/data"),
        ],
    )

    imu_transform = Node(
        package="imu_transformer",
        executable="imu_transformer_node",
        name="imu_data_transformer",
        output="screen",
        parameters=[{
            "target_frame": "camera_imu_frame",
        }],
        remappings=[
            ("imu_in", "/imu/data"),
            ("imu_out", "/imu/data_body"),
        ],
    )

    external_imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="external_imu_filter",
        output="screen",
        parameters=[external_imu_filter_params],
        remappings=[
            ("imu/data_raw", "/imu/data_raw"),
            ("imu/mag", "/imu/mag"),
            ("imu/data", "/imu/external/data"),
        ],
    )

    external_imu_transform = Node(
        package="imu_transformer",
        executable="imu_transformer_node",
        name="external_imu_transformer",
        output="screen",
        parameters=[{
            "target_frame": "base_footprint",
        }],
        remappings=[
            ("imu_in", "/imu/external/data"),
            ("imu_out", "/imu/external/data_body"),
            ("mag_in", "/imu/mag"),
            ("mag_out", "/imu/external/mag_body"),
        ],
    )

    # rgbd_odometry = Node(
    #     package='rtabmap_odom',
    #     executable='rgbd_odometry',
    #     name='rgbd_odometry',
    #     output='screen',
    #     parameters=rgbd_odometry_parameters,
    #     remappings=rgbd_odometry_remappings,
    # )

    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        condition=UnlessCondition(manage_lidar_standby),
        parameters=icp_odometry_parameters,
        remappings=icp_odometry_remappings,
    )

    managed_icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        condition=IfCondition(manage_lidar_standby),
        parameters=icp_odometry_parameters,
        remappings=managed_icp_odometry_remappings,
    )

    rtabmap_slam = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        condition=UnlessCondition(manage_lidar_standby),
        parameters=rtabmap_parameters,
        remappings=rtabmap_remappings,
        arguments=['-d'])

    managed_rtabmap_remappings = rtabmap_remappings + [('scan', standby_scan_topic)]

    managed_rtabmap_slam = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        condition=IfCondition(manage_lidar_standby),
        parameters=rtabmap_parameters,
        remappings=managed_rtabmap_remappings,
        arguments=['-d'])

    lidar_standby_manager = Node(
        package='diffbot',
        executable='diffbot_lidar_standby_manager',
        name='diffbot_lidar_standby_manager',
        output='screen',
        condition=IfCondition(manage_lidar_standby),
        parameters=[{
            'idle_timeout_sec': 10.0,
            'initial_idle_timeout_sec': 25.0,
            'scan_timeout_sec': 3.0,
            'start_motor_service': '/start_motor',
            'stop_motor_service': '/stop_motor',
            'pause_rtabmap_service': '/rtabmap/pause',
            'resume_rtabmap_service': '/rtabmap/resume',
            'pause_odom_service': '/pause_odom',
            'resume_odom_service': '/resume_odom',
            'scan_topic': '/scan',
            'managed_scan_topic': standby_scan_topic,
            'nav_action_status_topics': [
                '/navigate_to_pose/_action/status',
                '/navigate_through_poses/_action/status',
                '/spin/_action/status',
            ],
            'publish_standby_scan_heartbeat': True,
            'standby_scan_heartbeat_hz': 1.0,
        }],
    )

    rosbridge_server_pkg = get_package_share_directory('rosbridge_server')
    rosbridge_server_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(rosbridge_server_pkg, 'launch', 'rosbridge_websocket_launch.xml')
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        depth_module,
        realsense,
        imu_filter,
        imu_transform,
        external_imu_filter,
        external_imu_transform,
        ekf,
        rplidar,
        # rgbd_odometry,
        icp_odometry,
        managed_icp_odometry,
        rtabmap_slam,
        managed_rtabmap_slam,
        lidar_standby_manager,
        nav2,
        rosbridge_server_launch
    ]

    return LaunchDescription(declared_arguments + nodes)
