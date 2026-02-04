from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_pkg_path = FindPackageShare("xpkg_urdf_echo")
    rviz_config_path = PathJoinSubstitution(
        [urdf_pkg_path, "config", "ROS2", "urdf.rviz"])
    urdf_file_path = PathJoinSubstitution(
        [urdf_pkg_path, "urdf", "model.urdf"])

    visual_arg = DeclareLaunchArgument(name='visual',
                                       default_value='true',
                                       choices=['true', 'false'],
                                       description='Flag to turn on rviz')

    description_content = ParameterValue(Command(['xacro ', urdf_file_path]),
                                         value_type=str)
    robot_state_node = Node(package='robot_state_publisher',
                            executable='robot_state_publisher',
                            parameters=[{
                                'robot_description': description_content,
                            }])

    joint_state_node = Node(package='joint_state_publisher',
                            executable='joint_state_publisher')

    rviz_node = Node(name="rviz2",
                     package="rviz2",
                     executable="rviz2",
                     arguments=["-d", rviz_config_path],
                     condition=IfCondition(LaunchConfiguration('visual')))

    launch_description = LaunchDescription(
        [visual_arg, robot_state_node, joint_state_node, rviz_node])

    return launch_description
