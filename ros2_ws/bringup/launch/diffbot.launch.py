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
            default_value="true",
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
        'Reg/Force3DoF': 'true'
    }]

    rtabmap_remappings = [
        ('imu', '/imu/external/data_body'),
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
        ('imu', '/imu/external/data_body'),
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
        'wait_imu_to_init': True,
        'always_check_imu_tf': True,
        'Reg/Force3DoF': 'true'
    }]

    icp_odometry_remappings = [
        ('imu', '/imu/external/data_body'),
        ('odom', '/rtabmap/icp_odom'),
        ('odom_info', '/rtabmap/icp_odom_info'),
        ('scan', '/scan')]

    managed_icp_odometry_remappings = [
        ('imu', '/imu/external/data_body'),
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
            'rgb_camera.profile': '640x360x15'
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
