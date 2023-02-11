# car_robot

ros2 launch object_detect_camera detectnet.launch.py input:=/dev/video4 output_codec:=vp9 output_bitrate:=3000000  output:=rtp://192.168.0.247:8080 model_name:=ssd-mobilenet-v2

## Use the following commands for streamer - Use an alias



alias stream='gst-launch-1.0 -v udpsrc port=8080  caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)VP9, payload=(int)96" ! rtpbin ! rtpvp9depay ! decodebin ! queue ! autovideoconvert !  videorate ! xvimagesink sync=false'

Make sure Networks folder is inside the workspace level 
You can set your network folder with the command inside tools folder in Jetson inference:
download_networks.sh  

Make sure to edit the file in order to send the networks to the correct folder.