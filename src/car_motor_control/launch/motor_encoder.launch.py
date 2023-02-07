from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    ld = LaunchDescription()
       
    
           
    motor_encoder = Node(
        package='car_motor_control',
        executable='motorencoder',
        output='screen'
        
    )
    

   
    
    ld.add_action(motor_encoder)

    return ld