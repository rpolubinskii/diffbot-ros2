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
    realsense_params = PathJoinSubstitution(
        [
            FindPackageShare("diffbot"),
            "config",
            "realsense_params.yaml",
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
        #  - LIDAR-PROXIMITY PIVOT 2026-06-20: visual closure is dead across ALL
        #    detectors (GFTT/BRIEF, SIFT, GFTT/ORB all landed 0 closures, best
        #    7-10/12 inliers, never clearing the 12 gate -- bags diffbot_run /
        #    diffbot_gftt_orb_robust; see diffbot-dev/task-notes/odometry-slam.md).
        #    VisIcp(2) keeps proximity closures gated on that dead visual
        #    verification, so switch to Icp(1): proximity/closure now registers via
        #    LIDAR ICP, heading- and appearance-independent. The prior Icp(1)
        #    corridor-slide was SINGLE-scan ICP (ProximityPathMaxNeighbors unset=0);
        #    the fix is merging neighbor scans (below) so the match has corners to
        #    lock onto instead of sliding the corridor lengthwise.
        'Reg/Strategy': '1',
        # Proximity-by-space (default on) now registers via lidar ICP (Reg/Strategy
        # =1 above) -- closes drift on nearby revisits independent of appearance.
        'RGBD/ProximityBySpace': 'true',
        # Merge up to 10 neighbor scans into a local map before proximity ICP. This
        # is the corridor-slide fix: a single 2D scan is under-constrained along a
        # corridor's long axis (parallel walls align at any offset), but a merged
        # multi-scan local map carries the corner / cross-wall geometry that pins
        # the longitudinal position. Without this (unset=0) the prior Reg/Strategy=1
        # run duplicated a corridor. Tune down if CPU/delay climbs; up if slide
        # persists.
        'RGBD/ProximityPathMaxNeighbors': '10',
        # Proximity attempt #1 (bag diffbot_lidar_proximity) fired ZERO proximity
        # closures: the default RGBD/ProximityMaxGraphDepth=50 only compares nodes
        # within 50 graph-links of the latest node, but this route's origin-revisits
        # are 92-262 links apart -> ALL excluded. 0 = no graph-depth limit, so
        # proximity-by-space can match spatially-near nodes no matter how far apart
        # they are in the graph (i.e. the actual revisits). REQUIRED companion:
        # ProximityOdomGuess=true seeds the proximity ICP with the odometry transform
        # -- needed for lidar registration, and it also bounds false matches under
        # drift (with Icp/MaxTranslation=0.5 + the robust optimizer). Ref:
        # rtabmap_ros#654. WATCH the next map for false closures (a wall/corridor
        # teleporting); if they appear, this gate is the cause.
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityOdomGuess': 'true',
        # PROXIMITY RESULT GATE (bag diffbot_proximity_gate, log
        # diffbot_2026-06-20_134148). Opening the graph-depth + odom-guess above
        # FINALLY made proximity-by-space FIRE on the origin revisit: it selected
        # node 1, ran VISUAL registration (>=12 inliers -> non-null transform), and
        # proposed a correction -- then a SINGLE gate killed it:
        #   Rtabmap.cpp:2829 "Ignoring local loop closure with 1 because resulting
        #   transform is too large!? (1.013119m > 1.000000m)".
        # Verified against rtabmap 0.22.1 source: the 1.0 m threshold is
        # RGBD/ProximityPathFilteringRadius (default 1.0). It gates the proximity
        # closure TWICE -- (a) candidate selection by optimized-pose distance
        # (passed) and (b) the magnitude of the RESULTING registration transform
        # (line 2781, failed). A proximity closure's transform IS the accumulated
        # drift it corrects; over this route the origin drift reached ~1.01 m, so
        # the 1.0 m default rejects EXACTLY the closure we want. NOT clamped lower
        # by RGBD/MaxLoopClosureDistance (unset=0). Raise to 2.0 m: admits the
        # observed 1.01 m correction with headroom for a longer loop while still
        # rejecting a >2 m teleport. The closure is already inlier-verified
        # (Vis/MinInliers>=12), so widening this is not "accepting garbage" -- it
        # lets the verified drift correction apply. WATCH the next map for a wall/
        # corridor snapping far on closure (a false positive); if so, drop to ~1.5.
        'RGBD/ProximityPathFilteringRadius': '2.0',
        # Keep the graph anchored at its start (do NOT optimize-from-end, which
        # would let the whole map shift under a new closure).
        'RGBD/OptimizeFromGraphEnd': 'false',
        # THIRD GATE -- the dominant failure in bag diffbot_latest (log
        # diffbot_2026-06-17_213820). After the depth-cap rollback + DetectionRate
        # =10, closures FINALLY pass the visual gate with good inliers AND survive
        # the ICP/MaxTranslation gate -- then 21 of them die HERE, at the graph
        # optimizer: "wrong loop closure ... maximum graph error ratio 3.05-4.49 ...
        # RGBD/OptimizeMaxError is 3.000000". The error is purely ROTATIONAL
        # (abs error 5.5-7.9 deg, stddev ~1.8 deg) and concentrates on one edge,
        # 186->197 type=2 (a ProximityBySpace link), cited in 16 of the 21 -- i.e.
        # icp_odometry accumulated ~6-7 deg of YAW drift across one spin, the visual
        # closure correctly proposes that 6-7 deg correction, and the default 3.0
        # gate (= reject above ~5.3 deg) throws out the real fix. THIS is the user's
        # "last visual feature gets misplaced when rotating": the correction exists
        # but is rejected. Raise to 5.0 (= reject above ~8.9 deg): admits every one
        # of the 21 observed real closures (max ratio 4.49) while still rejecting a
        # true teleport/false-positive (those land well above 5x stddev). The
        # closures are already double-verified (Vis/MinInliers>=12 + Icp/MaxTrans
        # <=0.5), so loosening this gate is not "discarding data" -- it lets the
        # verified drift correction apply. Root cause behind the 6-7 deg (icp yaw
        # drift during spins) is the separate rgbd_odometry-fusion lever.
        #
        # 5.0 -> 6.0 on 2026-06-17 (bag diffbot_gftt_orb). GFTT/ORB fixed the
        # 0-inlier problem -> 18 REAL closures now pass the inlier gate and reach
        # this optimizer gate, but all are rejected with error ratios tightly
        # clustered 5.007-5.686 -- just above 5.0. The culprit is NOT the closures
        # (node pairs are consistent revisits: 400-404 -> 1/250/346, 345 -> many
        # early nodes) but ONE bad odometry NEIGHBOR edge: 16 of 18 cite
        # "edge 146->147, type=0" (a sequential icp-odom link) carrying a ~9.6 deg
        # rotation glitch that becomes the max residual on every closure attempt.
        # 6.0 admits the cluster (max 5.69) so the real closures can apply and the
        # optimizer can distribute that one bad edge's error.
        #
        # SWITCHED TO ROBUST OPTIMIZER 2026-06-17 (bag diffbot_orb_optmax6_2). The
        # gate-bumping is over: a 2nd good-revisit run hit the SAME failure -- a real
        # closure (355<->234) reached the optimizer but was batch-rejected because a
        # single bad NEIGHBOR (odometry) edge "354->355, type=0" carried a ~12.6 deg
        # rotation glitch (ratio 7.24 > 6.0). The glitch magnitude VARIES run-to-run
        # (9.6 deg in gftt_orb, 12.6 deg here), so NO fixed OptimizeMaxError reliably
        # admits real closures while an icp rotation glitch exists. The right tool is
        # robust graph optimization (Vertigo switchable constraints / dynamic
        # covariance scaling): it DOWN-WEIGHTS the individual outlier odometry edge
        # instead of rejecting the whole batch, so the good closure applies and the
        # bad edge is discounted. RGBD/OptimizeMaxError MUST be 0 with Robust=true
        # (they are mutually exclusive in rtabmap). Optimizer/Strategy must be a
        # robust-capable backend (1=g2o or 2=GTSAM, NOT 0=TORO) -- forcing g2o(1),
        # which carries Vertigo. VERIFY from the next log that rtabmap reports g2o +
        # robust enabled; if g2o/Vertigo isn't in the Jetson build, try Strategy=2
        # (GTSAM). If robust destabilizes (over-down-weights real closures), revert
        # to OptimizeMaxError=6.0 + Robust=false.
        'RGBD/OptimizeMaxError': '0',
        'Optimizer/Robust': 'true',
        'Optimizer/Strategy': '1',
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
        # Ease the inlier gate slightly: a candidate already reached 16/20, and
        # 6/20, so 12 lets marginal-but-real closures through (still well above
        # noise). Raise back toward 20 if false closures appear.
        'Vis/MinInliers': '12',
        # FEATURE TYPE: SIFT (1). History: default GFTT/BRIEF (6) gave a flat wall of
        # "0/12 inliers"; GFTT/ORB (8) broke that on GOOD-DEPTH views (bag
        # diffbot_gftt_orb hit 10/12 + 7/12, 18 closures reached the optimizer) BUT
        # most views still return 0 inliers (bags diffbot_orb_optmax6 / _robust: ALL
        # candidates 0/12 despite 39-77 matches). Root cause = the keypoint DETECTOR:
        # GFTT (in both 6 and 8) picks CORNERS, which sit on object edges / depth
        # discontinuities where RealSense stereo depth is noisiest -> the matched
        # keypoints get valid-but-WRONG 3D -> PnP RANSAC finds no geometric consensus
        # -> 0 inliers, regardless of descriptor. SIFT uses a DoG BLOB detector that
        # places keypoints on the interior of textured regions, OFF the depth-edges
        # -> cleaner 3D at the keypoints -> the 3D verification can actually pass.
        # Set on BOTH detection vocabulary (Kp/DetectorStrategy) and registration/PnP
        # (Vis/FeatureType). CAVEATS: (a) SIFT must be in the Jetson's OpenCV build --
        # VERIFY from the next log that rtabmap initializes SIFT and does NOT fall
        # back to GFTT/BRIEF or error; if unavailable, revert to ORB (8). (b) SIFT is
        # heavier than ORB -- WATCH rtabmap delay= (was ~0.18s; if it climbs toward
        # ~1s, drop Rtabmap/DetectionRate or Kp/MaxFeatures). Robust optimizer stays
        # (it handles the bad-odom-edge batch-reject once inliers arrive).
        # VALIDATION GATE: inliers climbing past 12 on revisits + "Added loop closure".
        # VALIDATION RESULT 2026-06-20 (bag diffbot_run, 1913 nodes, full route): GATE FAILED.
        # SIFT loaded fine (no fallback, delay ~0.16s) but landed 0 closures; best 7/12 inliers
        # over 18 candidates, 17 at 0/12 -- i.e. SIFT ALSO hits the 0-inlier wall, no better
        # than GFTT/ORB. Hypothesis above (blob detector -> cleaner 3D) is REFUTED: the wall is
        # not a detector problem. FOLLOW-UP 2026-06-20: before the lidar pivot, retest GFTT/ORB
        # (8/8) WITH the robust optimizer -- that combo was never run together (GFTT/ORB's 10/12
        # run predates the robust optimizer), so a landed closure is still plausible. Set to 8/8
        # for this test; if it ALSO lands 0 closures, pivot to lidar-proximity (the wall is then
        # confirmed depth/scene, not detector). See diffbot-dev/task-notes/odometry-slam.md.
        'Kp/DetectorStrategy': '8',
        'Vis/FeatureType': '8',
        # Capture more data WHILE MOVING. rtabmap filters input frames down to
        # this rate (default 1 Hz -> our log showed "Rate=1.00s"), dropping the
        # rest, so this -- NOT camera fps -- is the real "data density" knob. At
        # 2 Hz the graph gets ~2x the nodes during driving -> finer map + more
        # loop-closure opportunities. CPU: rtabmap processing was ~0.13-0.22 s per
        # iteration at 1 Hz; 2 Hz roughly doubles that thread load and the working
        # memory grows faster, so WATCH the "delay=" stat -- if it climbs toward
        # ~1 s, drop to 1.5.
        'Rtabmap/DetectionRate': '5'
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
            # Param names: this realsense2_camera build uses
            # rgb_camera.color_profile / depth_module.depth_profile (the old
            # 'rgb_camera.profile'/'depth_module.profile' were silently ignored).
            # RESOLUTION LOWERED 2026-06-20 (720p/848x480 -> 640x360/424x240) to
            # cut CPU. Once visual loop closure was REFUTED (see task-notes, lidar
            # proximity is the SLAM unlock now), high camera resolution buys
            # nothing for SLAM: odometry is lidar icp_odometry, the 2D nav grid is
            # lidar, and rtabmap closures register via lidar ICP (Reg/Strategy=1).
            # But the NEW depth point cloud for the STVL collision layer is built
            # by deprojecting depth per frame, which cost ~1 full Jetson core at
            # 848x480 (realsense node ~94-110% CPU) -> pushed rtabmap delay 0.18 ->
            # 0.7 s and made rviz sluggish + the rtabmap depth map thin. Quartering
            # the pixels (~4x cheaper deprojection) restores headroom while staying
            # plenty dense for a 0.05 m collision voxel grid at <2.5 m. Bump back up
            # only if collision needs finer depth AND the CPU budget allows.
            'rgb_camera.color_profile': '640x360x30',
            'depth_module.depth_profile': '424x240x30',
            # ---- DEPTH CLEANUP FOR THE VISUAL COLLISION LAYER (2026-06-20) ----
            # Goal: a depth point cloud clean enough that the STVL obstacle layer
            # doesn't mark on RealSense flying-pixels / "shading".
            #   pointcloud.enable -> publish /camera/camera/depth/color/points, the
            #     LIVE per-frame cloud STVL consumes. Deliberately fed to collision
            #     DIRECTLY (not rtabmap's /cloud_obstacles) so collision does NOT
            #     depend on the rtabmap SLAM node (which can crash; see task-notes).
            #   spatial_filter -> cheap resolution-preserving edge-preserving denoise.
            # IMPORTANT -- this depth stream is SHARED with rtabmap. Two filters were
            # tried and REMOVED 2026-06-20 because they regressed SLAM / load:
            #   * clip_distance=3.0 -> clipped the SHARED depth so rtabmap could not
            #     map past 3 m: /mapData + /grid_prob_map went to thin sparse patches.
            #     Collision range is bounded by STVL itself (obstacle_range=2.5,
            #     max_z) so the camera-level clip is unnecessary -> REMOVED.
            #   * temporal_filter -> ~30 fps full-frame CPU cost (helped make rviz/
            #     topics sluggish + starved rtabmap of frames) and it ghosts depth
            #     during motion (stale obstacles) -> REMOVED. STVL voxel_min_points +
            #     decay do the transient-noise rejection downstream instead.
            # Also NOT enabled: decimation_filter (halves depth res -> risks the
            # aligned-depth/camera_info path rtabmap needs) and hole_filling
            # (fabricates depth -> phantom obstacles).
            # On this ARM/NEON build the actual filter instance is exposed as
            # pointcloud__neon_; realsense_params.yaml enables that launch-time
            # node parameter. Do not rely on the standard pointcloud.enable key:
            # the live /camera/camera node does not declare it on this build.
            'pointcloud.enable': 'true',
            'spatial_filter.enable': 'true',
            'config_file': realsense_params
            # accelerate_gpu_with_glsl was TESTED 2026-06-20 and REJECTED. It works
            # (camera healthy, no GL errors) and halves CPU at 848x480 (94% -> 52%),
            # BUT it does NOT help the metric that matters for collision -- cloud
            # RATE: 848x480 ran 1.2 Hz with GPU (vs 11.5 Hz at 424x240), too slow
            # for a collision cloud (robot moves ~0.25 m between updates and STVL
            # decay barely refreshes). At the 424x240 res we keep FOR that rate, GPU
            # accel is marginal (47% vs 53% CPU) and adds a GL-context dependency to
            # the camera that everything depends on. Not worth the risk -> left off.
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

    # Honest-yaw-covariance relay: republishes /rtabmap/icp_odom as
    # /rtabmap/icp_odom_reweighted with the yaw covariance inflated whenever icp's
    # rotation rate disagrees with the RealSense gyro (i.e. spins icp mis-measures
    # on self-similar walls). The EKF fuses the reweighted topic (ekf.yaml odom1),
    # so it pins heading to icp when they agree and yields to the gyro during
    # spins. icp is blind to its own rotational error (covariance is a fixed
    # 0.1x-x scalar, quality signals don't degrade on spins -- bags _2/_3); the
    # gyro is the only independent reference. Both managed + unmanaged
    # icp_odometry publish /rtabmap/icp_odom, so one always-on relay covers both.
    icp_odom_reweighter = Node(
        package='diffbot',
        executable='diffbot_icp_odom_reweighter',
        name='diffbot_icp_odom_reweighter',
        output='screen',
        parameters=[{
            'input_odom_topic': '/rtabmap/icp_odom',
            'imu_topic': '/imu/data_body',
            'output_odom_topic': '/rtabmap/icp_odom_reweighted',
            'yaw_disagreement_gain': 2.0,      # KEY knob: added yaw std per rad/s mismatch
            'disagreement_deadband': 0.1,      # rad/s below = noise, no inflation
            'min_yaw_variance': 0.0,           # 0 = pass icp's value when agreeing
            'max_yaw_variance': 1.0,           # clamp (~57 deg = ignore icp yaw)
            'reweight_twist': True,            # also down-weight icp vyaw on mismatch
            'gyro_timeout_sec': 0.5,           # stale gyro -> pass through, warn
        }],
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
        icp_odom_reweighter,
        rtabmap_slam,
        managed_rtabmap_slam,
        lidar_standby_manager,
        nav2,
        rosbridge_server_launch
    ]

    return LaunchDescription(declared_arguments + nodes)
