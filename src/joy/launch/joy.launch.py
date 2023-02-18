from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    ld = LaunchDescription()
       
    
           
    joy_node = Node(
        package='joy',
        executable='joy_node',
        #output='screen'  //use for debugging
        
    )
    

   
    
    ld.add_action(joy_node)

    return ld