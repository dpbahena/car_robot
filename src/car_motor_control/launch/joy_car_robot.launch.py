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

    # ! -------- JOY NODE ----------------->
    launch_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('joy'),
                'launch/joy.launch.py'
            )
        )
    )


    joystick_car_control_node = Node(
        package='car_motor_control',
        executable='joystickcarcontrol',
        #output='screen'  //use for debugging
        
    )

    

    ld.add_action(launch_joy) 
    ld.add_action(joystick_car_control_node)
    
   
   
    return ld