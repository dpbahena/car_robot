from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    ld = LaunchDescription()
    # args that can be set from the command line or a default will be used
    maxtime_arg = DeclareLaunchArgument(
        "maxtime", default_value=TextSubstitution(text="3")
    )
    
           
    motor_controller = Node(
        package='car_motor_control',
        executable='motorcontroller',
        output='screen',
        parameters=[
            {"maxtime": LaunchConfiguration('maxtime')}           
        ]
    )
   
    ld.add_action(maxtime_arg)
    ld.add_action(motor_controller)
    return ld