from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    ld = LaunchDescription()
       
    
           
    relative_speeds = Node(
        package='car_motor_control',
        executable='relativespeeds',
        output='screen'
        
    )
    

   
    
    ld.add_action(relative_speeds)

    return ld