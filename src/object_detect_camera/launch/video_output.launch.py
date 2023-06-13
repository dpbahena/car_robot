from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    # args that can be set from the command line or a default will be used
    output_arg = DeclareLaunchArgument(
        #"output", default_value=TextSubstitution(text="display://0")
        #"output", default_value=TextSubstitution(text="display://10.0")
        "output", default_value=TextSubstitution(text="rtp://192.168.0.247:8080")

    )
    output_codec_arg = DeclareLaunchArgument(
        #"output_codec", default_value=TextSubstitution(text="unknown")
        "output_codec", default_value=TextSubstitution(text="vp9")
        #"output_codec", default_value=TextSubstitution(text="h264")
    )
    output_bitrate_arg = DeclareLaunchArgument(
        "output_bitrate", default_value=TextSubstitution(text="0")
    )
    topic_launch_arg = DeclareLaunchArgument(
        "topic", default_value=""
    )
    
    video_output = Node(
        package='object_detect_camera',           
        executable='video_output',
        output='screen',
        remappings=[
            #('image_in', 'raw')   #  <---- works too!
            ('image_in', LaunchConfiguration("topic"))   # <------ this is the ideal  use a variable 
        ],
        parameters=[{
            "resource": LaunchConfiguration("output"),
            "codec": LaunchConfiguration("output_codec"),
            "bitrate": LaunchConfiguration("output_bitrate")
        }]
    )

    
    ld.add_action(output_arg)
    ld.add_action(output_codec_arg)
    ld.add_action(output_bitrate_arg)
    ld.add_action(topic_launch_arg)

    ld.add_action(video_output)

    return ld


    # !--------------EQUIVALENT XML code --------------->

#     <launch>
# 	<arg name="output" default="display://0"/>
# 	<arg name="output_codec" default="unknown"/>
# 	<arg name="output_bitrate" default="0"/>
# 	<arg name="topic"/>

# 	<node pkg="ros_deep_learning" exec="video_output" output="screen">
# 		<remap from="/video_output/image_in" to="$(var topic)"/>
# 		<param name="resource" value="$(var output)"/>
# 		<param name="codec" value="$(var output_codec)"/>
# 		<param name="bitrate" value="$(var output_bitrate)"/>
# 	</node>

# </launch>