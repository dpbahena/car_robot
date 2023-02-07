from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    ld = LaunchDescription()
       
    
           
    action_server_node = Node(
        package='car_motor_control',
        executable='caracctionserver',
        output='screen'
        
    )
    

   
    
    ld.add_action(action_server_node)

    return ld