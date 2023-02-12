from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    
    # args that can be set from the command line or a default will be used
    tolerance_arg = DeclareLaunchArgument(
        "tolerance", default_value= TextSubstitution(text="6")
    )
    kp_arg = DeclareLaunchArgument(
        "kp", default_value= TextSubstitution(text="0.112")
    )
    ki_arg = DeclareLaunchArgument(
        "ki", default_value= TextSubstitution(text="-0.0011")
    )
    kd_arg = DeclareLaunchArgument(
        "kd", default_value= TextSubstitution(text="-0.0925")
    )
   
           
    tracking_node = Node(
        package='object_detect_camera',
        executable='tracking',
        output='screen',
        parameters=[{
            "tolerance": LaunchConfiguration('tolerance'),
            "kp": LaunchConfiguration('kp'),
            "ki": LaunchConfiguration('ki'),
            "kd": LaunchConfiguration('kd')    
        }]
    )

   
   
   
    ld.add_action(tolerance_arg)
    ld.add_action(kp_arg)
    ld.add_action(ki_arg)
    ld.add_action(kd_arg)
    ld.add_action(tracking_node)
    

    return ld