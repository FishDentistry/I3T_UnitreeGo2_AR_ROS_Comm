from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
import os

def generate_launch_description():

    send_realsense = LaunchConfiguration('send_realsense')
    transport_type = LaunchConfiguration('transport_type')


    return LaunchDescription([

        DeclareLaunchArgument('send_realsense', default_value='false'),
        DeclareLaunchArgument('transport_type', default_value='http'),

        Node(
            package='ar_hrc_server_comm',
            executable='realsenseTopicsSubscriber',
            name='send_realsense_images_to_server',
            output='screen',
            condition=IfCondition(send_realsense),
            parameters=[{
                'transport_type': transport_type
            }]
        ),
        Node(
            package='ar_hrc_server_comm',
            executable='robotStateComm',
            name='robot_state_server_comm',
            output='screen',
            parameters=[{
                'transport_type': transport_type
            }]
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
            output='screen',
            condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('transport_type'), "' == 'http'"])
                )
        ),
        Node(
            package='ar_hrc_server_comm',
            executable='findArucoFrameTCP',
            name='find_aruco_frame_tcp',
            output='screen',
            condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('transport_type'), "' == 'tcp'"])
                )
        ),
        Node(
            package='ar_hrc_server_comm',
            executable='showObjsRviz',
            name='show_objs_rviz',
            output='screen'
        )
    ])
