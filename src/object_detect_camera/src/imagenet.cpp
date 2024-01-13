#include "ros2_definitions.h"
#include "image_converter.h"

#include <jetson-inference/imageNet.h>
#include <jetson-utils/cudaFont.h>

#include <unordered_map>


using std::placeholders::_1;
using namespace std::chrono_literals;

class ImageNet : public rclcpp::Node {
public:
    ImageNet();
    ~ImageNet();
private:
    rclcpp::TimerBase::SharedPtr timer_;
    imageNet* net = NULL;
    cudaFont* font = NULL;
    vision_msgs::msg::VisionInfo info_msg;

    // parameter variables
    std::string model_name = "googlenet";
    std::string model_path;
    std::string prototxt_path;
    std::string class_labels_path;

    std::string input_blob = IMAGENET_DEFAULT_INPUT;
    std::string output_blob = IMAGENET_DEFAULT_OUTPUT;

    imageConverter* input_cvt = NULL;
    imageConverter* overlay_cvt = NULL;
    // publishers
    rclcpp::Publisher<vision_msgs::msg::Classification>::SharedPtr classify_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
    rclcpp::Publisher<vision_msgs::msg::VisionInfo>::SharedPtr info_pub_;
    // subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;


    // publisher callbacks
    void info_callback();
    bool publish_overlay( int img_class, float confidence);
    
    // subscriber callbacks
    void img_callback( const sensor_msgs::msg::Image::ConstSharedPtr input);

    void init_publishers();
    void init_info_publisher();
    void init_subscribers();
    void declare_set_parameters();
    void load_network();
    void create_label_vector();
    

};  // end class

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    __node_name_ = "imagenet_node";
    __global_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    rclcpp::spin(std::make_shared<ImageNet>());
    rclcpp::shutdown();
    return 0;
}

void ImageNet::info_callback(){
    ROS_INFO("new subscriber connected to vision_info, sending VisionInfo msg");
    info_pub_->publish(info_msg);

}

void ImageNet::init_publishers(){
    classify_pub_ = this->create_publisher<vision_msgs::msg::Classification>("classification", 5);
    overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>("overlay", 2);
    
    
    

}

void ImageNet::init_info_publisher(){
    info_pub_ = this->create_publisher<vision_msgs::msg::VisionInfo>("vision_info", 1);
   
  
    timer_ = this->create_wall_timer(500ms, [&](){ static int __subscribers_info_pub_=0; const size_t __subscription_count=info_pub_->get_subscription_count(); if(__subscribers_info_pub_ != __subscription_count) { if(__subscription_count > __subscribers_info_pub_) {ImageNet::info_callback(), this;} __subscribers_info_pub_=__subscription_count; }});
    
    
    // static int  subscribers_to_info_pub_= 0; 
    // const size_t __subscription_count = info_pub_->get_subscription_count(); 
    // if(subscribers_to_info_pub_ != __subscription_count) {
    //     if(__subscription_count > __subscription_count){
    //          timer_ = this->create_wall_timer(500ms, std::bind(&ImageNet::info_callback, this));
    //     }
    //     subscribers_to_info_pub_ = __subscription_count;         

    // }

    //ROS_CREATE_PUBLISHER_STATUS(vision_msgs::msg::VisionInfo, "vision_info", 1, ImageNet::info_callback, info_pub_);
}

void ImageNet::init_subscribers(){
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_in", 5, std::bind(&ImageNet::img_callback, this, _1) );
}

ImageNet::ImageNet(): Node(__node_name_){
    
    init_subscribers(); 
    init_publishers();
    init_info_publisher();
    
    declare_set_parameters();
    
    load_network();

    create_label_vector();
    
}

void ImageNet::declare_set_parameters(){
    // Declare parameters


    this->declare_parameter("model_name", model_name);
    this->declare_parameter("model_path", model_path);
    this->declare_parameter("prototxt_path", prototxt_path);
    this->declare_parameter("class_labels_path", class_labels_path);
    this->declare_parameter("input_blob", input_blob);
    this->declare_parameter("output_blob", output_blob);

    // Retrive parameters

    this->get_parameter("model_name", model_name);
    this->get_parameter("model_path", model_path);
    this->get_parameter("prototxt_path", prototxt_path);
    this->get_parameter("class_labels_path", class_labels_path);
    this->get_parameter("input_blob", input_blob);
    this->get_parameter("output_blob", output_blob);
}
void ImageNet::load_network(){
    
    if( model_path.size() > 0){
        // create networks using custom model paths
        
        net = imageNet::Create(prototxt_path.c_str(), model_path.c_str(), NULL, class_labels_path.c_str(), input_blob.c_str(), output_blob.c_str());
    }else{
        
        // Create network using the built-in model
        
        net = imageNet::Create(model_name.c_str());
        
    }

    if(!net){
        ROS_ERROR("failed to load imageNet model");
        return;
    }

    
}

void ImageNet::create_label_vector(){

    std::hash<std::string> model_hasher; // has the model path to avoid collisions on the param server
    std::string model_hash_str = std::string(net->GetModelPath()) + std::string(net->GetClassPath());
    const size_t model_hash = model_hasher(model_hash_str);

    ROS_INFO("model hash => %zu", model_hash);
    ROS_INFO("hash string => %s", model_hash_str.c_str());

    // obtain the list of class descriptions
    std::vector<std::string>class_descriptions;
    const uint32_t num_classes = net->GetNumClasses();
    
    for( uint32_t n=0; n < num_classes; n++){
        class_descriptions.push_back(net->GetClassDesc(n));
    }

    // create the key on the param server

    std::string class_key = std::string("class_labels_") + std::to_string(model_hash);
    
    this->declare_parameter(class_key, class_descriptions);
    this->set_parameter(rclcpp::Parameter(class_key, class_descriptions));

    // populate the vision info msg

    const std::string node_namespace = this->get_namespace();
    ROS_INFO("node namespace => %s", node_namespace.c_str());

    info_msg.database_location = node_namespace + std::string("/") + class_key;
    info_msg.database_version = 0;
    info_msg.method = net->GetModelPath();
    
    ROS_INFO("class labels=> %s", info_msg.database_location.c_str());

    // create image converter objects

    input_cvt = new imageConverter();
    overlay_cvt = new imageConverter();
    
    if (!input_cvt || !overlay_cvt){
        ROS_ERROR("failed to create imageConverter objects");
        return;
    }

}

// triggered when recieved a new image on input topic (subscriber)
void ImageNet::img_callback(const sensor_msgs::msg::Image::ConstSharedPtr input){
    // convert the image tyo reside on GPU
    if( !input_cvt || !input_cvt->Convert(input)){
        ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
        return;
    }

    // classify the image

    float confidence = 0.0f;
    const int img_class = net->Classify(input_cvt->ImageGPU(), input_cvt->GetWidth(), input_cvt->GetHeight(), &confidence);
                                        
    // verify the ouput
    if (img_class >= 0){
        ROS_INFO("classified image, %f %s (class=%i)", confidence, net->GetClassDesc(img_class), img_class);

        // create the classfication message
        vision_msgs::msg::Classification msg;
        vision_msgs::msg::ObjectHypothesis obj;

        //#if ROS_DISTRO >= ROS_GALACTIC
        obj.class_id = img_class;
        //#else 
        
        //obj.id = img_class;
        //#endif
        
        obj.score = confidence;
        msg.results.push_back(obj);  //TODO optionally add source image to msg
        





        
        // populate timestamp in header field;
        msg.header.stamp = __global_clock_->now();
        
        // publish the classification message
        classify_pub_->publish(msg);
        
        // generate the overlay (if there are subscribers)
        if(overlay_pub_->get_subscription_count() > 0){
            publish_overlay(img_class, confidence);
        }


    }else{
        // an error occured if the output class is < 0
        ROS_ERROR("failed to classify %ux%u image", input->width, input->height);
    }

}

bool ImageNet::publish_overlay( int img_class, float confidence){
    // create font for image overlay
    font = cudaFont::Create();
    
    if(!font){
        ROS_ERROR("failed to load font for everlay");
        return false;
    }

    // get the image dimensions
    const uint32_t width = input_cvt->GetWidth();
    const uint32_t height = input_cvt->GetHeight();

    // assure correct image size
    if(!overlay_cvt->Resize(width, height, imageConverter::ROSOutputFormat)){
        return false;
    }
    

    // generate the overlay
    char str[256];
    sprintf(str, "%05.2f%% %s", confidence * 100.0f, net->GetClassDesc(img_class) );

    font->OverlayText(input_cvt->ImageGPU(), width, height, str, 5,5, make_float4(255, 255,255,255), make_float4(0,0,0,100));

    /// populate the massage
    sensor_msgs::msg::Image msg;

    if(!overlay_cvt->Convert(msg, imageConverter::ROSOutputFormat, input_cvt->ImageGPU())){
        return false;
    }

    // populate timestamp in header field
    msg.header.stamp = __global_clock_->now();

    // publish the message
    overlay_pub_->publish(msg);
    
    ROS_DEBUG("publishing %ux%u overlay image", width, height);
    

}

ImageNet::~ImageNet(){
    delete net;
    delete input_cvt;
    delete overlay_cvt;
}