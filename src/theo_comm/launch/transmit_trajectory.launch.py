from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    launch_dir = PathJoinSubstitution(
        [
            FindPackageShare('theo_comm'), 
            'launch'
        ]
    )
    viconr_launch_dir = PathJoinSubstitution(
        [
            FindPackageShare('vicon_receiver'), 
            'launch'
        ]
    )

    #  1. Launch File Deploying Brokerage
    #  2. Launch File Deploying Transmitter


    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [
                    launch_dir, 
                    'broker.launch.py'
                ]
            )
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [
                    launch_dir, 
                    'transmission.launch.py'
                ]
            )
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [
                    viconr_launch_dir, 
                    'client.launch.py'
                ]
            ),
            launch_arguments={
                'hostname': '192.168.0.62', # needs to change if hostname does, CHANGE THIS
                # for example, AHG vicon cage, 192.168.50.10
            }.items()
        ),
    ])
