from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    ld = LaunchDescription()
       
    
           
    car_pose = Node(
        package='car_motor_control',
        executable='carpose',
        output='screen'
        
    )
    

   
    
    ld.add_action(car_pose)

    return ld