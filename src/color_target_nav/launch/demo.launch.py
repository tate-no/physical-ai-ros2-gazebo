from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('color_target_nav'))
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    models = Path(get_package_share_directory('turtlebot3_gazebo')) / 'models'
    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(
            filter(None, (str(models), os.environ.get('GAZEBO_MODEL_PATH', ''))))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo / 'launch/gazebo.launch.py')),
                                 launch_arguments={'world': str(share / 'worlds/color_targets.world')}.items()),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'burger_camera_main', '-file',
                        str(share / 'models/burger_contact.sdf'), '-z', '0.05'])])
