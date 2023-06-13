import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription, Substitution
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution

##################################333
#
#    ros2 launch machine_learning_pkg tracking.launch.py input:=/dev/video2 model_name:=ssd-mobilenet-v2 maxtime:=1
#    NOTE: input:=/dev/video2   will be needed in case video changes from 2, or 3 or 4,  etc
#
#    ros2 launch object_detect_camera project.launch.py model_name:=ssd-mobilenet-v2 maxtime:=1 threshold:=.6
#
#    h264 code:
#
#    gst-launch-1.0 -v udpsrc port=8080  caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtpbin ! rtph264depay ! decodebin ! queue ! autovideoconvert !  videorate ! xvimagesink sync=false    
#
#    vp9 code :
#    gst-launch-1.0 -v udpsrc port=8080  caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)VP9, payload=(int)96" ! rtpbin ! rtpvp9depay ! decodebin ! queue ! autovideoconvert !  videorate ! xvimagesink sync=false
#
############################33

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
    launch_joystick_control_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('car_motor_control'),
                'launch/joy_car_robot.launch.py'
            )
            
        )
        
    )
   
   
    ld.add_action(launch_detectnet)
    ld.add_action(launch_robot)
    ld.add_action(launch_joystick_control_node)
    
    
   
    
    return ld