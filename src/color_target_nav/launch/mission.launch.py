from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('color_target_nav'))
    return LaunchDescription([Node(package='color_target_nav', executable='watchdog', output='screen'),
                              Node(package='color_target_nav', executable='navigator',
                                   output='screen', parameters=[str(share / 'config/mission.yaml')])])
