from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    ld = LaunchDescription()
       
    
           
    pantilt_node = Node(
        package='car_motor_control',
        executable='pantilt',
        #output='screen'  //use for debugging
        
    )
    

   
    
    ld.add_action(pantilt_node)

    return ld