import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo_pkg_path = FindPackageShare("gazebo_ros")
    urdf_pkg_path = FindPackageShare("xpkg_urdf_echo")
    rviz_config_path = PathJoinSubstitution(
        [urdf_pkg_path, "config", "ROS2", "urdf.rviz"])
    urdf_file_path = os.path.join(
        get_package_share_directory('xpkg_urdf_echo'), 'urdf',
        'sim_base.xacro')
    description_content = xacro.process_file(urdf_file_path).toxml()

    sim_arg = DeclareLaunchArgument('use_sim_time',
                                    default_value='true',
                                    choices=['true', 'false'],
                                    description='Use sim time if true')
    visual_arg = DeclareLaunchArgument(name='visual',
                                       default_value='true',
                                       choices=['true', 'false'],
                                       description='Flag to turn on rviz')

    robot_state_node = Node(package='robot_state_publisher',
                            executable='robot_state_publisher',
                            output='screen',
                            parameters=[{
                                'robot_description': description_content,
                                'use_sim_time': use_sim_time
                            }])

    joint_state_node = Node(package='joint_state_publisher',
                            executable='joint_state_publisher')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch',
                         'gazebo.launch.py')
        ]),)

    spawn_entity = Node(package='gazebo_ros',
                        executable='spawn_entity.py',
                        output='screen',
                        arguments=[
                            '-topic', 'robot_description', '-entity',
                            'sim', '-x', '0.0', '-y', '0.0', '-z', '0.0',
                            '-Y', '0.0'
                        ])

    rviz_node = Node(name="rviz2",
                     package="rviz2",
                     executable="rviz2",
                     arguments=["-d", rviz_config_path],
                     condition=IfCondition(LaunchConfiguration('visual')))

    launch_description = LaunchDescription([
        sim_arg, visual_arg, robot_state_node, joint_state_node, rviz_node,
        gazebo, spawn_entity
    ])

    return launch_description