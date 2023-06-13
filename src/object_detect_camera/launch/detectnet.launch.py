import os
from xml.etree.ElementInclude import default_loader

#from pyparsing import str_type
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution

#
#launch - example:
#
#  ros2 launch object_detect_camera detectnet.launch.py output_bitrate:=3000000  model_name:=ssd-mobilenet-v2
#
#   ros2 launch object_detect_camera detectnet.launch.py output_bitrate:=3000000  model_name:=ssd-mobilenet-v2 output:=rtp://10.0.0.253:8080 output_codec:=h264
#
#   for rtp in h264 mode use: gst-launch-1.0 -v udpsrc port=8080  caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtpbin ! rtph264depay ! decodebin ! queue ! autovideoconvert !  videorate ! xvimagesink sync=false
#
#   for rtp in vp9 mode use: gst-launch-1.0 -v udpsrc port=8080  caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)VP9, payload=(int)96" ! rtpbin ! rtpvp9depay ! decodebin ! queue ! autovideoconvert !  videorate ! xvimagesink sync=false

#

def generate_launch_description():
    ld = LaunchDescription()
    
    

    # !---- VIDEO SOURCE ------this is another launch file-------->
    
    launch_video_source = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('object_detect_camera'),
                'launch/video_source.launch.py'
            )
        )
    )

    # ! -------- arguments that can be set from command line or a default will be used for imagenet_node ------>

    model_name_arg = DeclareLaunchArgument(
        "model_name", default_value=TextSubstitution(text="ssd-mobilenet-v2")
    )
    model_path_arg = DeclareLaunchArgument(
        "model_path", default_value=TextSubstitution(text=str(""))
    )
    prototxt_path_arg = DeclareLaunchArgument(
        'prototxt_path', default_value=TextSubstitution(text="")
    )
    class_labels_path_arg = DeclareLaunchArgument(
        'class_labels_path', default_value=TextSubstitution(text="")
    )
    input_blob_arg = DeclareLaunchArgument(
        'input_blob', default_value=TextSubstitution(text="")
    )
    output_cvg_arg = DeclareLaunchArgument(
        'output_cvg', default_value=TextSubstitution(text="")
    )
    output_bbox_arg = DeclareLaunchArgument(
        'output_bbox', default_value=TextSubstitution(text="")
    )
    overlay_flags_arg = DeclareLaunchArgument(
        'overlay_flags', default_value=TextSubstitution(text="0")   # works only with "0" instead of "box, labels, conf" as suggested in the original code
    )
    mean_pixel_value_arg = DeclareLaunchArgument(
        'mean_pixel_value', default_value=TextSubstitution(text="0.0")
    )
    threshold_arg  = DeclareLaunchArgument(
        'threshold', default_value=TextSubstitution(text="0.5")
    )


    # This is the topic variable that Output node will subscribed to
    topic_arg = DeclareLaunchArgument(
        'topic', default_value='overlay'
    )



    detectnet_node = Node(
        package='object_detect_camera',
        executable='detectnet',
        output='screen',
        remappings=[
           ('image_in', 'raw' )
        ],
        parameters=[{
            "model_name": LaunchConfiguration("model_name"),# LaunchConfiguration('model_name')},
            "model_path": LaunchConfiguration('model_path'),
            "prototxt_path": LaunchConfiguration('prototxt_path'),
            "class_labels_path": LaunchConfiguration("class_labels_path"),
            "input_blob": LaunchConfiguration("input_blob"),
            "output_cvg": LaunchConfiguration("output_cvg") ,
            "output_bbox": "", #LaunchConfiguration('output_bbox')},
            "overlay_flags": LaunchConfiguration('overlay_flags'),
            "mean_pixel_value": LaunchConfiguration('mean_pixel_value'),
            "threshold": LaunchConfiguration('threshold')
        }]
        
    )
  
    # ! ------ VIDEO OUTPUT ------- another launch file----->
    launch_video_output = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('object_detect_camera'),
                'launch/video_output.launch.py'
            )       
        )
    )

    # --- Run the video source first : video, live, or picture
    ld.add_action(launch_video_source)
    # --- Run the variables needed for Imagenet node -------->
    ld.add_action(model_path_arg)
    ld.add_action(model_name_arg)
    
    ld.add_action(prototxt_path_arg)
    ld.add_action(class_labels_path_arg)
    ld.add_action(input_blob_arg)
    ld.add_action(output_cvg_arg)
    #ld.add_action(output_bbox_arg)
    ld.add_action(overlay_flags_arg)
    ld.add_action(mean_pixel_value_arg)
    ld.add_action(threshold_arg)

    # --- Launch imagenet node -------------->
    ld.add_action(detectnet_node)


    # --- Run variable needed for video_output node ------->
    ld.add_action(topic_arg)
    
    # --- Run the output node
    ld.add_action(launch_video_output)
    
    
    return ld