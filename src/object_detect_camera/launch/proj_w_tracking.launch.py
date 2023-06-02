import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription, Substitution
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution

##################################3
#
#    ros2 launch machine_learning_pkg tracking.launch.py input:=/dev/video2 model_name:=ssd-mobilenet-v2 maxtime:=1
#    NOTE: use parameter of input:=/dev/video3 or 4 if needed  see above example
#
#    ros2 launch object_detect_camera project.launch.py model_name:=coco-bottle maxtime:=1 threshold:=.6
#
#    model_name:=ssd-mobilenet-v2 maxtime:=1 threshold:=.6
#
#    ros2 launch object_detect_camera proj_w_tracking.launch.py model_name:=coco-bottle maxtime:=1 threshold:=.6 output:=rtp://10.0.0.253:8080 output_codec:=h264
#
#
############################

def generate_launch_description():
    ld = LaunchDescription()
    

    # !---- Launch Detectnet.launch.py ------detect opjects -------->
    launch_detectnet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('object_detect_camera'),
                'launch/detectnet.launch.py'
            )
        )
    )

   
     # ! ------Launch robot.launch.py------for car control----->
    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('car_motor_control'),
                'launch/car_robot.launch.py'
            )
            
        )
        
    )
   
     # ! ------Launch tracking.launch.py------for tracking images----->
    launch_tracking_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('object_detect_camera'),
                'launch/tracking.launch.py'
            )
            
        )
        
    )
   
   
    ld.add_action(launch_detectnet)
    ld.add_action(launch_robot)
    ld.add_action(launch_tracking_node)
    
    
   
    
    return ld