from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    send_realsense = LaunchConfiguration('send_realsense')
    send_robot_state = LaunchConfiguration('send_robot_state')


    return LaunchDescription([

        DeclareLaunchArgument('send_realsense', default_value='false'),
        DeclareLaunchArgument('send_robot_state', default_value='true'),

        Node(
            package='ar_hrc_server_comm',
            executable='realsenseTopicsSubscriber',
            name='send_realsense_images_to_server',
            output='screen',
            condition=IfCondition(send_realsense)
        ),
        Node(
            package='ar_hrc_server_comm',
            executable='robotStateComm',
            name='robot_state_server_comm',
            output='screen',
            condition=IfCondition(send_robot_state)
        ),
        Node(
            package='ar_hrc_server_comm',
            executable='robotCommandListener',
            name='robot_command_listener',
            output='screen'
        ),
        Node(
            package='ar_hrc_server_comm',
            executable='voxelMapper',
            name='voxel_mapper',
            output='screen'
        ),
        Node(
            package='ar_hrc_server_comm',
            executable='findArucoFrame',
            name='find_aruco_frame',
            output='screen'
        ),
        Node(
            package='ar_hrc_server_comm',
            executable='showObjsRviz',
            name='show_objs_rviz',
            output='screen'
        )
    ])
