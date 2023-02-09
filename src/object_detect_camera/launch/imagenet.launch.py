import os

from pyparsing import str_type
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution

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
        "model_name", default_value='googlenet'
    )
    model_path_arg = DeclareLaunchArgument(
        "model_path", default_value=TextSubstitution(text=str(""))
    )
    prototxt_path_arg = DeclareLaunchArgument(
        'prototxt', default_value=TextSubstitution(text="")
    )
    class_labels_path_arg = DeclareLaunchArgument(
        'class_labels_path', default_value=TextSubstitution(text="")
    )
    input_blob_arg = DeclareLaunchArgument(
        'input_blob', default_value=TextSubstitution(text="")
    )
    output_blob_arg = DeclareLaunchArgument(
        'output_blob', default_value=TextSubstitution(text="")
    )

    # This is the topic variable that Output node will subscribed to
    topic_arg = DeclareLaunchArgument(
        'topic', default_value='overlay'
    )



    imagenet_node = Node(
        package='object_detect_camera',
        executable='imagenet',
        output='screen',
        remappings=[
           ('image_in', 'raw' )
        ],
        parameters=[
            {"model_name": LaunchConfiguration('model_name')},
            {"model_path": ""},
            # {"model_name": LaunchConfiguration("model_name")},
            #{"model_path": LaunchConfiguration('model_path')},
            {"prototxt_path": ""},
            {"class_labels_path": ""},
            {"input_blob": ""},
            {"output_blob": ""}
        ]
        
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
    ld.add_action(output_blob_arg)
    # --- Launch imagenet node -------------->
    ld.add_action(imagenet_node)


    # --- Run variable needed for video_output node ------->
    ld.add_action(topic_arg)
    
    # --- Run the output node
    ld.add_action(launch_video_output)
    
    
    return ld