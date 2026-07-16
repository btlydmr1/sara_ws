from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    mavros_launch = os.path.join(
        get_package_share_directory('mavros'), 'launch', 'px4.launch'
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mavros_launch),
            launch_arguments={'fcu_url': '/dev/ttyACM0:921600'}.items()
        ),
        Node(
            package='sara_control',
            executable='sara_simple_control',
            name='sara_simple_control_node',
            output='screen'
        )
    ])