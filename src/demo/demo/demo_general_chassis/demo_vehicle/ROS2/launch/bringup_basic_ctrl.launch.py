import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("xpkg_demo"), "demo_vehicle/ROS2/config",
        "rviz_vehicle.rviz"
    ])

    vehicle_ini_path = PathJoinSubstitution(
        [FindPackageShare("xpkg_vehicle"), "ini", "device_id_list.ini"])

    rviz_node = Node(name="rviz2",
                     package="rviz2",
                     executable="rviz2",
                     arguments=["-d", rviz_config_path])

    xnode_comm = Node(name="xnode_comm",
                      package="xpkg_comm",
                      executable="xnode_comm",
                      output="screen",
                      parameters=[{
                          "dev_list": False,
                          "com_enable": True,
                          "com_channel_common": False,
                          "com_channel_xstd": True,
                      }])

    xnode_vehicle = Node(name="xnode_vehicle",
                         package="xpkg_vehicle",
                         executable="xnode_vehicle",
                         output="screen",
                         parameters=[{
                             "ini_path": vehicle_ini_path,
                             "show_path": True,
                             "show_loc": False,
                             "calc_speed": False,
                             "mode_can_lock": False,
                             "rate_x": 1.0,
                             "rate_y": 1.0,
                             "rate_z": 1.0,
                             "rate_az": 1.0,
                         }])

    launch_description = LaunchDescription(
        [rviz_node, xnode_comm, xnode_vehicle])

    return launch_description
