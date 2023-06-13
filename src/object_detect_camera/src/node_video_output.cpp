#include "ros2_definitions.h"
#include "image_converter.h"

#include <jetson-utils/videoOutput.h>

using std::placeholders::_1;

class SalidaVideo : public rclcpp::Node{
public:
    SalidaVideo();
    ~SalidaVideo();
private:
    videoOutput* stream = NULL;
    imageConverter* image_cvt = NULL;
    std::string topic_name;

    void init_subscribers();
    void init_parameters();
    void open_videoOutput(std::string resource_str, videoOptions video_options);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_ ;
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

};


int main(int argc, char ** argv){
    __node_name_ = "salida_del_video";
    rclcpp::init(argc, argv);
    __global_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    rclcpp::spin(std::make_shared<SalidaVideo>());
    rclcpp::shutdown();
    return 0;
}

SalidaVideo::SalidaVideo() : Node(__node_name_){

    
    init_parameters();
    init_subscribers();
      // Create image converter

    image_cvt = new imageConverter();

    if ( !image_cvt){
        ROS_ERROR("failed to create imageConverter");
        return;
    }else{
        ROS_INFO("image_cvt was created");
        
    }

       // Start streaming

    if( !stream->Open()){
        ROS_ERROR("failed to start streaming video source");
        return;
    }else{
        ROS_INFO("The stream is open...");
    }

}

void SalidaVideo::open_videoOutput(std::string resource_str, videoOptions video_options){
     
    ROS_INFO("opening video output: %s", resource_str.c_str());

    // create stream

    stream = videoOutput::Create(resource_str.c_str(), video_options);

    if (!stream){

        ROS_ERROR("failed to open video stream");
        return;
    }

 

  
}

void SalidaVideo::init_parameters(){

    // declare parameters
    videoOptions video_options;
    std::string resource_str;
    std::string codec_str;

    int video_bitrate = video_options.bitRate;

    this->declare_parameter("resource", resource_str);
    this->declare_parameter("codec", codec_str);
    this->declare_parameter("bitrate", video_bitrate);

    // retrieve parameters

    this->get_parameter("resource", resource_str);
    this->get_parameter("codec", codec_str);
    this->get_parameter("bitrate", video_bitrate);

    if( resource_str.size() == 0){
        ROS_ERROR("resource param wasn't set - please set the node's resource parameter to the input device/file/URL");
        return ;
    }

    if( codec_str.size() != 0){
        video_options.codec = videoOptions::CodecFromStr(codec_str.c_str());    
    }
    video_options.bitRate = video_bitrate;

    open_videoOutput(resource_str, video_options);

}

void SalidaVideo::init_subscribers(){
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_in", 5, std::bind(&SalidaVideo::image_callback, this,_1) );
    topic_name = image_sub_->get_topic_name();
    ROS_INFO("The topic name is %s", topic_name.c_str());
}

SalidaVideo::~SalidaVideo(){
    delete stream;
    delete image_cvt;
}

void SalidaVideo::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg){
    // Convert the image to reside on GPU
    
    if( !image_cvt || !image_cvt->Convert(msg)){
        ROS_INFO("failed to convert %ux%u %s image", msg->width, msg->height, msg->encoding.c_str());
        return;
    }
    // update the Status Bar 
    char str[256];
    sprintf(str, "%s (%ux%u) | %.1f FPS", topic_name.c_str(),  image_cvt->GetWidth(), image_cvt->GetHeight(), stream->GetFrameRate());
    stream->SetStatus(str);

    // Render the image

    stream->Render(image_cvt->ImageGPU(), image_cvt->GetWidth(), image_cvt->GetHeight());

}