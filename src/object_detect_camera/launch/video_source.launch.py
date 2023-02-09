from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    
    # args that can be set from the command line or a default will be used
    input_arg = DeclareLaunchArgument(
        "input", default_value=TextSubstitution(text="/dev/video4")
    )
    input_width_arg = DeclareLaunchArgument(
        "input_width", default_value=TextSubstitution(text="0")
    )
    input_height_arg = DeclareLaunchArgument(
        "input_height", default_value=TextSubstitution(text="0")
    )
    input_codec_arg = DeclareLaunchArgument(
        "input_codec", default_value=TextSubstitution(text="unknown")
    )
    input_loop_arg = DeclareLaunchArgument(
        "input_loop", default_value=TextSubstitution(text="0")
    )
    input_latency_arg = DeclareLaunchArgument(
        "input_latency", default_value=TextSubstitution(text="2000")
    )
           
    video_source = Node(
        package='object_detect_camera',
        executable='video_source',
        output='screen',
        parameters=[{
            "resource": LaunchConfiguration("input"),
            "codec": LaunchConfiguration('input_codec'),
            "width": LaunchConfiguration('input_width'),
            "height": LaunchConfiguration('input_height'),
            "loop": LaunchConfiguration('input_loop'),
            #"flip": LaunchConfiguration(""),
            "rtsp_latency": LaunchConfiguration('input_latency')
            
        }]
    )
    ld = LaunchDescription()

   
    ld.add_action(input_arg)
    ld.add_action(input_codec_arg)
    ld.add_action(input_height_arg)
    ld.add_action(input_width_arg)
    ld.add_action(input_loop_arg)
    ld.add_action(input_latency_arg)
    ld.add_action(video_source)

    return ld




    #!  ------- Equivalent code in XML ------->


# <launch>
# 	<arg name="input" default="csi://0"/>
# 	<arg name="input_width" default="0"/>
# 	<arg name="input_height" default="0"/>
# 	<arg name="input_codec" default="unknown"/>
# 	<arg name="input_loop" default="0"/>
#   <arg name="input_latency" default="2000"/>

# 	<node pkg="ros_deep_learning" exec="video_source" output="screen">
# 		<param name="resource" value="$(var input)"/>
# 		<param name="width" value="$(var input_width)"/>
# 		<param name="height" value="$(var input_height)"/>
# 		<param name="loop" value="$(var input_loop)"/>
# 		<param name="rtsp_latency" value="$(var input_latency)"/>
# 	</node>

# </launch>