import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription, Substitution
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution

def generate_launch_description():
    ld = LaunchDescription()

    # ! -------- MOTOR CONTROLLER ----------------->
    launch_motor_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('car_motor_control'),
                'launch/motor_controller.launch.py'
            )
        )
    )

    # ! -------- RELATIVE SPEEDS ---------------->
    launch_relative_speeds = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('car_motor_control'),
                'launch/relative_speeds.launch.py'
            )
        )
    )

    # ! ------------ MOTOR ENCODER ---------------->
    launch_wheel_encoder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('car_motor_control'),
                'launch/motor_encoder.launch.py'
            )
        )
    )

     # ! ------------ CAR POSE ---------------->
    launch_car_pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('car_motor_control'),
                'launch/car_pose.launch.py'
            )
        )
    )

     # ! ------------ CAR ACTION SERVER ---------------->
    launch_action_srv = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('car_motor_control'),
                'launch/car_action_server.launch.py'
            )
        )
    )

    # ! ------------ PanTilt  ---------------->
    launch_pantilt = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('car_motor_control'),
                'launch/pantilt.launch.py'
            )
        )
    )

    ld.add_action(launch_motor_controller) 
    ld.add_action(launch_relative_speeds)
    ld.add_action(launch_wheel_encoder) 
    ld.add_action(launch_car_pose)
    ld.add_action(launch_action_srv)
    ld.add_action(launch_pantilt)
   
   
    return ld