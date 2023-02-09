import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription, Substitution
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution


def generate_launch_description():
    ld = LaunchDescription()
    

    # !---- VIDEO SOURCE ------this is another launch file-------->
    launch_video_source = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('machine_learning_pkg'),
                'launch/video_source.launch.py'
            )
        )
    )

    # ! ------LAUNCH ARGUMENT: to be passed to another launch file ------>
    # args that can be set from the command line or a default may be used
    
    # This is the topic variable that Output node will subscribed to
    topic_arg = DeclareLaunchArgument(
        'topic', default_value='raw'
    )


    # ! ------ VIDEO OUTPUT ------- another launch file----->
    launch_video_output = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('machine_learning_pkg'),
                'launch/video_output.launch.py'
            )
            
        )
        
    )


   

   
    ld.add_action(launch_video_source)
    ld.add_action(topic_arg)    ##  <---------------the order is important---so it is passed on to the video output---<<<
    ld.add_action(launch_video_output)
    
   
    
    return ld

##### EQUIVALENT CODE IN XML
    
# <launch>

# 	<!-- VIDEO SOURCE -->
# 	<include file="$(find-pkg-share ros_deep_learning)/launch/video_source.ros2.launch"/>

# 	<!-- VIDEO OUTPUT -->
# 	<include file="$(find-pkg-share ros_deep_learning)/launch/video_output.ros2.launch">
# 		<arg name="topic" value="/video_source/raw"/>
# 	</include>

# </launch>