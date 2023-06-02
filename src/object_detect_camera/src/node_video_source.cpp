#include "ros2_definitions.h"
#include "image_converter.h"
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <sensor_msgs/image_encodings.hpp>
// #include <vision_msgs/msg/classification2_d.hpp>
// #include <vision_msgs/msg/detection2_d_array.hpp>
// #include <vision_msgs/msg/vision_info.hpp>


#include <jetson-utils/videoSource.h>
using namespace std::chrono_literals;


//extern std::string __node_name_;





class FuenteVideo : public rclcpp::Node {
public:
    FuenteVideo();
    ~FuenteVideo();
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    //rclcpp::Clock::SharedPtr __global_clock_;
    rclcpp::TimerBase::SharedPtr timer_;

    videoSource *stream = NULL;
    imageConverter *image_cvt = NULL;
  

    // Declare parameters
    videoOptions video_options;
    std::string resource_str;
    std::string codec_str;
    std::string flip_str;
    int video_width;
    int video_height;
    int rtsp_latency;

    void init_publisher();
    bool aquireFrame();
    void open_video_source();
    void stream_callback();


}; // end of class


int main(int argc, char ** argv){
    
    __node_name_ = "Fuente_de_video";
    rclcpp::init(argc, argv);
    __global_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    rclcpp::spin(std::make_shared<FuenteVideo>());
    
    rclcpp::shutdown();
    
}

FuenteVideo::FuenteVideo(): Node(__node_name_){
    
    init_publisher();  
    
    // Declare parameters

    this->declare_parameter("resource", resource_str);
    this->declare_parameter("codec", codec_str);
    this->declare_parameter("width", video_width);
    this->declare_parameter("height", video_height);
    this->declare_parameter("framerate", video_options.frameRate);
    this->declare_parameter("loop", video_options.loop);
    this->declare_parameter("flip", flip_str);
    this->declare_parameter("rtsp_latency", rtsp_latency);

    // Retrive parameters

    this->get_parameter("resource", resource_str);
    this->get_parameter("codec", codec_str);
    this->get_parameter("width", video_width);
    this->get_parameter("height", video_height);
    this->get_parameter("framerate", video_options.frameRate);
    this->get_parameter("loop", video_options.loop);
    this->get_parameter("flip", flip_str);
    this->get_parameter("rtsp_latency", rtsp_latency);

    if( resource_str.size() == 0){
        RCLCPP_ERROR(this->get_logger(),"resource parameter wasn't set - please set the node's resource parameter to the input device/filename/URL");
        exit(0);
    }

    if(codec_str.size() != 0){
        
        video_options.codec = videoOptions::CodecFromStr(codec_str.c_str());

    }

    if(flip_str.size() != 0){
        
        video_options.flipMethod = videoOptions::FlipMethodFromStr(flip_str.c_str());
    }

    open_video_source();

}


void FuenteVideo::open_video_source(){

    video_width = video_options.width;
    video_height = video_options.height;
    rtsp_latency = video_options.rtspLatency;
    //rtsp_latency = video_options.latency;

    RCLCPP_INFO(this->get_logger(),"Opening video source: %s", resource_str.c_str());

    stream = videoSource::Create(resource_str.c_str(), video_options);
    
    if( !stream){
        RCLCPP_ERROR(this->get_logger(),"failed to open video source");
        exit(0);
    }else
        RCLCPP_INFO(this->get_logger(),"I am transmitting ...");

    image_cvt = new imageConverter();
    if( !image_cvt){
        RCLCPP_ERROR(this->get_logger(),"failed to create imageConverter");
        exit(0);
    }
   
    
}

void FuenteVideo::stream_callback(){
    
    
    
    //aquireFrame();
    
    if( !aquireFrame()){
        if( !stream->IsStreaming() ){
            ROS_INFO("stream is closed or reached EOS, existing node...");
            exit(0);
        }
    }

}

void FuenteVideo::init_publisher(){

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("raw", 2);
    timer_ = this->create_wall_timer(1ms, std::bind(&FuenteVideo::stream_callback, this));
    
}


bool FuenteVideo::aquireFrame(){
    imageConverter::PixelType* nextFrame = NULL;
    

    // gets the latest frame
    if( !stream->Capture(&nextFrame, 1000)){
        //RCLCPP_ERROR(this->get_logger(),"failed to capture next frame");
        return false;
    }else{
        //RCLCPP_DEBUG(this->get_logger(), " I am still here ... after capturing a frame!!!");
    }
    // assure correct image size
    if( !image_cvt->Resize(stream->GetWidth(), stream->GetHeight(), imageConverter::ROSOutputFormat)){
        //RCLCPP_ERROR(this->get_logger(),"failed to resize camera image converter");
        return false;
    }else{
        //RCLCPP_DEBUG(this->get_logger(), " I am still here part 2... after RESIZING a frame!!!");
    }

    // populate the message
    sensor_msgs::msg::Image msg;
    
    if(!image_cvt->Convert(msg, imageConverter::ROSOutputFormat, nextFrame)){
        //RCLCPP_ERROR(this->get_logger(),"failed to convert video stream frame to sensor_msgs::msg::Image");
        return false;
    }

    // populate timestamp in header field
    msg.header.stamp = __global_clock_->now();
    

    // publish the message
    image_pub_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "published %ux%u video frame", stream->GetWidth(), stream->GetHeight());

    return true;  
}

FuenteVideo::~FuenteVideo(){
    delete stream;
    delete image_cvt;

}