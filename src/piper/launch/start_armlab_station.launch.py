from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
import os
import re
import socket

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"   # Force colored output in the terminal for ROS 2 logging

def get_station_number():
    # Hostnames look like robotics-education-lab<N>; returns 0 if no match
    match = re.search(r'robotics-education-lab(\d+)(?:\..*)?$', socket.gethostname())
    return int(match.group(1)) if match else 0

def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal).'
    )
    # Declare the launch arguments
    station_id_arg = DeclareLaunchArgument(
        'station_id',
        default_value=str(get_station_number()),
        description='Station number (1-14). Sets ROS_DOMAIN_ID and the station<N> namespace. '
                    'Defaults to the number in the hostname; 0 leaves ROS_DOMAIN_ID untouched.'
    )

    apply_joint_offsets_arg = DeclareLaunchArgument(
        'apply_joint_offsets',
        default_value='false',
        description='Apply the per-station joint offsets for station_id. Off by default for now.'
    )

    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='can0',
        description='CAN port to be used by the Piper node.'
    )
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        description='Automatically enable the Piper node.'
    )

    gripper_exist_arg = DeclareLaunchArgument(
        'gripper_exist',
        default_value='true',
        description='gripper'
    )

    gripper_val_mutiple_arg = DeclareLaunchArgument(
        'gripper_val_mutiple',
        default_value='1',
        description='gripper'
    )

    logitech_video_device_arg = DeclareLaunchArgument(
        'logitech_video_device',
        default_value='/dev/v4l/by-path/pci-0000:80:14.0-usb-0:7:1.0-video-index0',
        description='V4L2 device path of the Logitech C922 webcam.'
    )

    realsense_top_usb_port_arg = DeclareLaunchArgument(
        'realsense_top_usb_port',
        default_value='4-9',
        description='USB port ID of the top RealSense D435i.'
    )

    realsense_wrist_usb_port_arg = DeclareLaunchArgument(
        'realsense_wrist_usb_port',
        default_value='4-10',
        description='USB port ID of the wrist RealSense D435i.'
    )

    realsense_profile_arg = DeclareLaunchArgument(
        'realsense_profile',
        default_value='848x480x30',
        description='Color/depth/infra profile (WxHxFPS) for both RealSense cameras.'
    )

    realsense_pointcloud_arg = DeclareLaunchArgument(
        'realsense_pointcloud',
        default_value='false',
        description='Publish the RealSense driver\'s own depth/color/points on both cameras. '
                    'armlab_object_tracker builds its clouds from aligned depth and does not need it.'
    )

    # Run every node below on ROS_DOMAIN_ID = station_id
    domain_id_env = SetEnvironmentVariable(
        'ROS_DOMAIN_ID',
        LaunchConfiguration('station_id'),
        condition=LaunchConfigurationNotEquals('station_id', '0')
    )

    ros_log_args = ['--log-level', LaunchConfiguration('log_level')]

    # Define the nodes
    logitech_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='logitech_c922',
        namespace='logitech_c922',
        output='screen',
        ros_arguments=ros_log_args,
        parameters=[{
            'video_device': LaunchConfiguration('logitech_video_device'),
        }]
    )

    realsense_top_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='intel_realsense_d435i_top',
        output='screen',
        ros_arguments=ros_log_args,
        parameters=[{
            'rgb_camera.color_profile': LaunchConfiguration('realsense_profile'),
            'depth_module.depth_profile': LaunchConfiguration('realsense_profile'),
            'depth_module.infra_profile': LaunchConfiguration('realsense_profile'),
            'pointcloud.enable': LaunchConfiguration('realsense_pointcloud'),
            'align_depth.enable': True,
            'usb_port_id': LaunchConfiguration('realsense_top_usb_port'),
        }]
    )

    realsense_wrist_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='intel_realsense_d435i_wrist',
        output='screen',
        ros_arguments=ros_log_args,
        parameters=[{
            'rgb_camera.color_profile': LaunchConfiguration('realsense_profile'),
            'depth_module.depth_profile': LaunchConfiguration('realsense_profile'),
            'depth_module.infra_profile': LaunchConfiguration('realsense_profile'),
            'pointcloud.enable': LaunchConfiguration('realsense_pointcloud'),
            'align_depth.enable': True,
            'usb_port_id': LaunchConfiguration('realsense_wrist_usb_port'),
        }]
    )

    piper_node = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='piper_ctrl_single_node',
        output='screen',
        ros_arguments=ros_log_args,
        parameters=[{
            'can_port': LaunchConfiguration('can_port'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'gripper_val_mutiple': LaunchConfiguration('gripper_val_mutiple'),
            'gripper_exist': LaunchConfiguration('gripper_exist'),
            # station_id 0 tells the node to run with zero joint offsets
            'station_id': PythonExpression([
                "'", LaunchConfiguration('station_id'), "' if '",
                LaunchConfiguration('apply_joint_offsets'), "'.lower() == 'true' else '0'"
            ]),
        }],
        remappings=[
            ('joint_ctrl_single', '/joint_states'),
        ]
    )

    station_group = GroupAction([
        PushRosNamespace(['station', LaunchConfiguration('station_id')]),
        logitech_node,
        realsense_top_node,
        realsense_wrist_node,
        piper_node,
    ])

    # Return the LaunchDescription
    return LaunchDescription([
        log_level_arg,
        station_id_arg,
        apply_joint_offsets_arg,
        can_port_arg,
        auto_enable_arg,
        gripper_exist_arg,
        gripper_val_mutiple_arg,
        logitech_video_device_arg,
        realsense_top_usb_port_arg,
        realsense_wrist_usb_port_arg,
        realsense_profile_arg,
        realsense_pointcloud_arg,
        domain_id_env,
        station_group
    ])
