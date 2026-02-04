from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    xnode_comm = Node(
        name="xnode_comm",
        package="xpkg_comm",
        executable="xnode_comm",
        output="screen",
        parameters=[{
            "dev_list": True,
            "com_enable": True,
            "com_channel_common": True,
            "com_channel_xstd": True,
            "tcp_enable": False,
            "tcp_channel_common": True,
            "tcp_channel_xstd": False,
            "tcp_addr": "115.29.240.46",
            "tcp_port": 9000,
        }]
    )
    
    launch_description = LaunchDescription([xnode_comm])
    
    return launch_description

