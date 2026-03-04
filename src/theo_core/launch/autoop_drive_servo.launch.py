from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    launch_dir = PathJoinSubstitution(
        [
            FindPackageShare('theo_core'), 
            'launch'
        ]
    )

    #  1. Launch File Deploying Pantilt Servoing Interfaces
    #  2. Launch File Deploying Autonomous Operation Interfaces

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [
                    launch_dir, 
                    'pantilt.launch.py'
                ]
            )
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [
                    launch_dir, 
                    'autoop_drive.launch.py'
                ]
            )
        ), 
    ])
