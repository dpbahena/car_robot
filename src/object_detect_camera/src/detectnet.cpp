#include "ros2_definitions.h"
#include "image_converter.h"

#include <jetson-inference/detectNet.h>
#include <unordered_map>

/*
*
*  ros2 launch machine_learning_pkg detectnet.launch.py input:=/dev/video2 threshold:=.6 model_name:=coco-bottle
*
*/






using std::placeholders::_1;
using namespace std::chrono_literals;

class DetectObjNet : public rclcpp::Node {
public:
    DetectObjNet();
    ~DetectObjNet();
private:
    rclcpp::TimerBase::SharedPtr timer_, timer2_;
    detectNet* net = NULL;
    bool detected = false;

    //std::string overlay_flags;
    int32_t overlay_flags = detectNet::OVERLAY_NONE;   // it does not work with uint32_t    as suggested in the original code

    imageConverter* input_cvt = NULL;
    imageConverter* overlay_cvt = NULL;

    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
    rclcpp::Publisher<vision_msgs::msg::VisionInfo>::SharedPtr info_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detection_status_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

    vision_msgs::msg::VisionInfo info_msg_;
    
    
    void init_publishers();
    void init_publisher_status();
    void init_subscribers();
    
    

    void info_callback();
    bool publish_overlay(detectNet::Detection* detections, int numDetections);
    void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr input);
    void detection_status_callback();

    void init_parameters();
    void create_class_labels();

};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    __node_name_ = "detectnet_node";
    __global_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    rclcpp::spin(std::make_shared<DetectObjNet>());
    rclcpp::shutdown();
    return 0;
}

DetectObjNet::DetectObjNet() : Node(__node_name_){
    init_publishers();
    init_publisher_status();
    init_subscribers();
    init_parameters();
    create_class_labels();

    timer2_ = this->create_wall_timer(2s,std::bind(&DetectObjNet::detection_status_callback,this));

}

void DetectObjNet::init_publishers(){
    detection_pub_= this->create_publisher<vision_msgs::msg::Detection2DArray>("detections",25);
    overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>("overlay", 2);
    detection_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("detection_status", 1);
    

    
    
}

void DetectObjNet::init_publisher_status(){

    info_pub_ = this->create_publisher<vision_msgs::msg::VisionInfo>("vision_info", 1);
   
  
    timer_ = this->create_wall_timer(500ms, [&](){ static uint32_t __subscribers_info_pub_=0; const size_t __subscription_count=info_pub_->get_subscription_count(); if(__subscribers_info_pub_ != __subscription_count){ if(__subscription_count > __subscribers_info_pub_)DetectObjNet::info_callback(); __subscribers_info_pub_=__subscription_count; }});
     
}

void DetectObjNet::init_subscribers(){
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_in", 5, std::bind(&DetectObjNet::img_callback, this, _1));

}


// Triggered when a new subscriber is connected
void DetectObjNet::info_callback(){
    ROS_INFO("new subscriber connected to vision_info topic, sending VisionInfo msg ");
    info_pub_->publish(info_msg_);

}

bool DetectObjNet::publish_overlay(detectNet::Detection* detections, int numDetections){

    // get the image dimensions
    const uint32_t width = input_cvt->GetWidth();
    const uint32_t height = input_cvt->GetHeight();

    // assure correct image size

    if( !overlay_cvt->Resize(width, height, imageConverter::ROSOutputFormat)){
        return false;
    }

    // generate the overlay
    if( !net->Overlay(input_cvt->ImageGPU(), overlay_cvt->ImageGPU(), width, height, 
                        imageConverter::InternalFormat, detections, numDetections, overlay_flags)){
        
        return false;
    }

    // populate the message
    sensor_msgs::msg::Image msg;

    if( !overlay_cvt->Convert(msg, imageConverter::ROSOutputFormat)){
        return false;
    }

    // populate timestamp in header file

    msg.header.stamp = __global_clock_->now();

    // publish the message

    overlay_pub_->publish(msg);
    ROS_DEBUG("publishing %ux%u overlay image", width, height);

    return true;

}


void DetectObjNet::img_callback(const sensor_msgs::msg::Image::ConstSharedPtr input){

    // convert the image to reside on GPU

    if(!input_cvt || !input_cvt->Convert(input)){
        ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
        return;
    }

    // classify the image
    detectNet::Detection* detections = NULL;

    const int numDetections = net->Detect(input_cvt->ImageGPU(), input_cvt->GetWidth(), input_cvt->GetHeight(), &detections, detectNet::OVERLAY_NONE);

    // verify success

    if(numDetections < 0 ){
        
        ROS_ERROR("failed to run object detection on %ux%u image", input->width, input->height);
        return;
    }
    if(numDetections == 0){
        detected = false;
    }

    // if objects were detected, send out message

    if(numDetections > 0){
        detected = true;
        ROS_INFO("Detected %i objects in %ux%u image", numDetections, input->width, input->height);

        // create a detection for each bounding box
        vision_msgs::msg::Detection2DArray msg;

        for( int n = 0; n < numDetections; n++){
            detectNet::Detection* det = detections + n;

            ROS_INFO("Object %i class #%u (%s) confidence = %f", n, det->ClassID, net->GetClassDesc(det->ClassID), det->Confidence);
            ROS_INFO("Object %i bounding box (%f, %f)  (%f, %f)  w=%f  h=%f", n, det->Left, det->Top, det->Right, det->Bottom, det->Width(), det->Height());

            // Create detection sub-message
            vision_msgs::msg::Detection2D detMsg;

            detMsg.bbox.size_x = det->Width();
            detMsg.bbox.size_y = det->Height();

            float cx, cy;
            det->Center(&cx, &cy);
            /* Foxy only  the following 2 lines */
            //detMsg.bbox.center.x = cx;
            //detMsg.bbox.center.y = cy;

            /* Following 2 lines only for HUMBLE ROS */
            detMsg.bbox.center.position.x = cx;
            detMsg.bbox.center.position.y = cy;

            detMsg.bbox.center.theta = 0.0f;   // TODO : optionally output object image

            // create classification hypothesis
            vision_msgs::msg::ObjectHypothesisWithPose hyp;

            

            // #if ROS_DISTRO >= ROS_GALACTIC
            hyp.hypothesis.class_id = det->ClassID;
            hyp.hypothesis.score = det->Confidence;
            // #else
            
            
            //hyp.id =  det->ClassID;
                                  
            //hyp.score = det->Confidence;
            // #endif
            
            detMsg.results.push_back(hyp);
            msg.detections.push_back(detMsg);

        }
        // populate timestamp in header field
        msg.header.stamp = __global_clock_->now();

        // publish the detection message
        detection_pub_->publish(msg);
    }

    // generate the overlay (if there are subscribers)
    if(overlay_pub_->get_subscription_count() > 0){
        publish_overlay(detections, numDetections);
    }

}


void DetectObjNet::init_parameters(){

    // retrive parameters

    //std::string model_name = "ssd-mobilenet-v2";
    std::string model_name;
    std::string model_path;
    std::string prototxt_path;
    std::string class_labels_path;

    std::string input_blob = DETECTNET_DEFAULT_INPUT;
    std::string output_cvg = DETECTNET_DEFAULT_COVERAGE;
    std::string output_bbox = DETECTNET_DEFAULT_BBOX;
    std::string overlay_str = "box,labels,conf";

    float mean_pixel = 0.0f;



    /*  Valid only in Humble ros the 2nd line */
    float threshold = DETECTNET_DEFAULT_THRESHOLD;
    //float threshold = DETECTNET_DEFAULT_CONFIDENCE_THRESHOLD;
    
    
    this->declare_parameter("model_name", model_name);
    this->declare_parameter("model_path", model_path);
    this->declare_parameter("prototxt_path", prototxt_path);
    this->declare_parameter("class_labels_path", class_labels_path);
    this->declare_parameter("input_blob", input_blob);
    this->declare_parameter("output_cvg", output_cvg);
    this->declare_parameter("output_bbox", output_bbox);
    this->declare_parameter("overlay_flags", overlay_flags);
    this->declare_parameter("mean_pixel_value", mean_pixel);
    this->declare_parameter("threshold", threshold);

    // retrieve parameters

    this->get_parameter("model_name", model_name);
    this->get_parameter("model_path", model_path);
    this->get_parameter("prototxt_path", prototxt_path);
    this->get_parameter("class_labels_path", class_labels_path);
    this->get_parameter("input_blob", input_blob);
    this->get_parameter("output_cvg", output_cvg);
    this->get_parameter("output_bbox", output_bbox);
    this->get_parameter("overlay_flags", overlay_flags);
    this->get_parameter("mean_pixel_value", mean_pixel);
    this->get_parameter("threshold", threshold);

    overlay_flags = detectNet::OverlayFlagsFromStr(overlay_str.c_str());

    // load object detection network

    if(model_path.size() > 0){
        // create network using custom model_paths
        net = detectNet::Create(prototxt_path.c_str(), model_path.c_str(), mean_pixel, class_labels_path.c_str(), threshold, input_blob.c_str(), output_cvg.c_str(), output_bbox.c_str());

    }else{
        // determine which built-in model was requested
        
        // detectNet::NetworkType model = detectNet::NetworkTypeFromStr(model_name_str.c_str());
        // if( model == detectNet::CUSTOM){
        //     ROS_ERROR("invalid built-in pretrained model name '%s', defaulting to pednet", model_name_str.c_str());
        //     model = detectNet::SSD_MOBILENET_V2;
        // }

        // create network using the built-in model
        //net = detectNet::Create(model, threshold);
        net = detectNet::Create(model_name.c_str());
    }

    if( !net ){
        ROS_ERROR("failed to load detectNet model");
        return;
    }

    

}


void DetectObjNet::create_class_labels(){

    std::hash<std::string> model_hasher;   // hash the model path to avoid collisions on the param server
    std::string model_hash_str = std::string(net->GetModelPath()) + std::string(net->GetClassPath());

    const size_t model_hash = model_hasher(model_hash_str);

    ROS_INFO("model hash => %zu", model_hash);
    ROS_INFO("hash string => %s", model_hash_str.c_str());

    // obtain the list of class descriptions
    std::vector<std::string> class_descriptions;
    const uint32_t num_classes = net->GetNumClasses();

    for( uint32_t n = 0; n < num_classes; n++){

        class_descriptions.push_back(net->GetClassDesc(n));

    }

    // create the key on the param server
    std::string class_key = std::string("class_labels_") + std::to_string(model_hash);

    this->declare_parameter(class_key, class_descriptions);
    this->set_parameter(rclcpp::Parameter(class_key, class_descriptions));

    // populate the vision info msg
    std::string node_namespace = this->get_namespace();
    ROS_INFO("node namespace => %s", node_namespace.c_str());

    info_msg_.database_location = node_namespace + std::string("/") + class_key;
    info_msg_.database_version = 0;
    info_msg_.method = net->GetModelPath();

    ROS_INFO("class labels => %s", info_msg_.database_location.c_str());

    // create image convert objects

    input_cvt = new imageConverter();
    overlay_cvt = new imageConverter();

    if( !input_cvt || !overlay_cvt ){
        ROS_ERROR("failed to create imageConverter objects");
        return;
    }   
}

DetectObjNet::~DetectObjNet(){

    delete net;
    delete input_cvt;
    delete overlay_cvt;

}

void DetectObjNet::detection_status_callback(){
    std_msgs::msg::Bool msg;

    msg.data = detected;

    detection_status_pub_->publish(msg);

    

}
