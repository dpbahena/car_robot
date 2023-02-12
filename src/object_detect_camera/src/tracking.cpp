#include "ros2_definitions.h"
#include "utils/pid.h""
#include "image_converter.h"
#include "geometry_msgs/msg/twist.hpp"
#include "car_interfaces/msg/pantilt.hpp"
#include <thread>




using std::placeholders::_1;
using namespace std::chrono_literals;

/*
*-------------------running the node only ---------------------------------
* 
* 
* ros2 run machine_learning_pkg test_subscriber --ros-args -p kp:=0.003 -p kd:=-0.001 -p ki:=-0.0035 
*  changing kp=-.001 follows you but slow to respond...so this is the best tunning
* ros2 run machine_learning_pkg test_subscriber --ros-args -p kp:=0.003 -p kd:=-0.0005 -p ki:=-0.0053
*/


class TrackingImages : public rclcpp::Node{

public:
    TrackingImages();
   


private:
    //imageConverter* input_cvt = NULL;
    rclcpp::TimerBase::SharedPtr parameter_timer_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detect_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detection_status_sub_;
    rclcpp::Subscription<car_interfaces::msg::Pantilt>::SharedPtr pantilt_sub_;

    
    int32_t image_width;
    int32_t image_height;
    int pan, tilt;
    
    bool is_object_detected;
    // parameters 
    float tolerance;
    float kp, kd, ki, tkp, tkd, tki, pkp, pkd, pki;
    float tau;
    float pLimMinInt, pLimMaxInt, tLimMinInt, tLimMaxInt;  // integral min and max values
    //int myrate;
    

    
    rclcpp::Subscription<vision_msgs::msg::VisionInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_pub_;
    rclcpp::Publisher<car_interfaces::msg::Pantilt>::SharedPtr pantilt_move_pub_;
   


    void init_subscribers();

    void info_callback(const vision_msgs::msg::VisionInfo::ConstSharedPtr msg);
    void detect_callback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void detection_status_callback(const std_msgs::msg::Bool::ConstSharedPtr msg);
    void parameter_callback();
    void pantilt_pose_callback(const car_interfaces::msg::Pantilt::ConstSharedPtr msg);

    void init_publishers();


};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    __node_name_ = "tracking_images";
    rclcpp::spin(std::make_shared<TrackingImages>());
    rclcpp::shutdown();

    return 0;
}

void TrackingImages::parameter_callback(){
    this->get_parameter("tolerance", tolerance);
    this->get_parameter("tau", tau);
    // car wheels 
    this->get_parameter("kp", kp);
    this->get_parameter("kd", kd);
    this->get_parameter("ki", ki);
    // pan pid
    this->get_parameter("pkp", pkp);
    this->get_parameter("pkd", pkd);
    this->get_parameter("pki", pki);
    this->get_parameter("pminint",pLimMinInt);
    this->get_parameter("pmaxint",pLimMaxInt);

    // tilt pid
    this->get_parameter("tkp", tkp);
    this->get_parameter("tkd", tkd);
    this->get_parameter("tki", tki);

    this->get_parameter("tminint",tLimMinInt);
    this->get_parameter("tmaxint",tLimMaxInt);
    
}

TrackingImages::TrackingImages():Node(__node_name_){
    init_subscribers();
    init_publishers();

    this->declare_parameter("tolerance", 0.5);  // amount of the tracking object offcenter
    // this->declare_parameter("kp", 0.0048387097);
    // this->declare_parameter("kd", 0.0);
    // this->declare_parameter("ki", -0.0035);
    this->declare_parameter("tau", 0.02);

    this->declare_parameter("kp", 0.048);
    this->declare_parameter("kd", 0.0);
    this->declare_parameter("ki", -0.0035);

    this->declare_parameter("pkd", -0.00001);
    this->declare_parameter("pki", -0.003);
    this->declare_parameter("pkp", 0.016);
    

    this->declare_parameter("pminint", -10.0);
    this->declare_parameter("pmaxint", 10.0);

    this->declare_parameter("tkd", 0.000);  
    this->declare_parameter("tki", -0.01);

    this->declare_parameter("tkp", 0.0175);

    this->declare_parameter("tminint", -2.0);
    this->declare_parameter("tmaxint", 0.5);
    

    // retrieve parameter every second if any
    parameter_timer_ = this->create_wall_timer(1000ms, std::bind(&TrackingImages::parameter_callback, this));
    

}

void TrackingImages::init_subscribers(){
    detect_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>("detections", 2, std::bind(&TrackingImages::detect_callback, this, _1));
    info_sub_ = this->create_subscription<vision_msgs::msg::VisionInfo>("vision_info", 4, std::bind(&TrackingImages::info_callback, this, _1) );
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("raw", 10, std::bind(&TrackingImages::image_callback, this, _1));
    detection_status_sub_ = this->create_subscription<std_msgs::msg::Bool>("detection_status",1,std::bind(&TrackingImages::detection_status_callback, this, _1));
    pantilt_sub_ = this->create_subscription<car_interfaces::msg::Pantilt>("pantilt_pose",30,std::bind(&TrackingImages::pantilt_pose_callback, this, _1)); //it was 1 instead of 30

}

void TrackingImages::init_publishers(){
    move_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("vel_cmds", 10);
    pantilt_move_pub_ = this->create_publisher<car_interfaces::msg::Pantilt>("pantilt_cmds", 15);  //it was 10 
}
void TrackingImages::detect_callback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg){
    
    int nd = msg->detections.size();
    int32_t PERSON = 1; 
    int32_t BOTTLE = 0;
    int32_t FACE = 0;
    //geometry_msgs::msg::Twist move_msg;
    geometry_msgs::msg::Twist move_msg;
    move_msg.linear.x = 0;
    move_msg.angular.z = 0;
    car_interfaces::msg::Pantilt move_pantilt;

   
    
    //ROS_INFO("the size of the detection list is %i", nd );
    
    // set up PID control variables for motor wheels
    PIDController pid;
    pid.Kp = kp;
    pid.Ki = ki;
    pid.Kd = kd;
    pid.limMin = -3.0;
    pid.limMax = 3.0;
    pid.limMinInt = -2.9;
    pid.limMaxInt = 2.9;
    pid.T = 0.0;
    float CarprevT = 0.0;
    // pan PID control
    PIDController panpid;
    panpid.Kp = pkp;
    panpid.Ki = pki;
    panpid.Kd = pkd;
    panpid.tau = tau;
    panpid.limMin = -20.0;    // degrees
    panpid.limMax = 20.0;
    panpid.limMinInt = pLimMinInt;
    panpid.limMaxInt = pLimMaxInt;
    panpid.T = 0.0;
    float prevT = 0.0;
    // Tilt PID control
    PIDController tiltpid;
    tiltpid.Kp = tkp;
    tiltpid.Ki = tki;
    tiltpid.Kd = tkd;
    tiltpid.limMin = -10.0;    // in degrees
    tiltpid.limMax = 10.0;
    tiltpid.limMinInt = tLimMinInt;
    tiltpid.limMaxInt = tLimMaxInt;
    tiltpid.T = 0.0;
    //float prevT = 0.0;



    //rclcpp::Rate rate(myrate);

    PIDController_Init(&panpid);
    PIDController_Init(&tiltpid);
    

    for (int n = 0; n < nd; n++ ){
                        
        // FOXY distro:
        /* int id = msg->detections.at(n).results.at(0).id[0]; */
        
        // HUMBLE distro:
        int id = msg->detections.at(n).id[0];

        // FOXY distro:
        /* float confidence = msg->detections.at(n).results.at(0).score; */
        
        // HUMBLE distro:
        float confidence = msg->detections.at(n).results.at(0).hypothesis.score;
        
        //ROS_INFO("id is %i and score is %f ", id, confidence);

        if (id == BOTTLE && confidence > .85){
            // get the center of the target 

            // FOR FOXY DISTRO:
            /* float x = msg->detections.at(0).bbox.center.x;
               float y = msg->detections.at(0).bbox.center.y;  */

            // For HUMBLE distro :
            float x = msg->detections.at(0).bbox.center.position.x;
            float y = msg->detections.at(0).bbox.center.position.y;

            std::cout << "horizontal center : " << x << " image center is: " << image_width/2 << std::endl;
            std::cout << "vertical center : " << y << " image center is: " << image_height/2 << std::endl;
          
            // set target position
            float target = 0;   // means camera is centered at the tracking object
            // PID constants
            
            float currT = this->get_clock()->now().seconds();
            float deltaT = (currT - prevT)/1000000000L;    // to seconds
            if (deltaT <= 0)
                deltaT = 1.65671;  // prevent deltaT becomes zero
            panpid.T = deltaT;
            tiltpid.T = deltaT;

            std::cout << "DELTAt = " << deltaT << std::endl;
            prevT = currT;
           

            float horizontal_measure = x - image_width/2;
            float vertical_measure = y - image_height/2;
            std::cout << "Horizontal error = " << horizontal_measure << std::endl;
            std::cout << "Vertical error = " << vertical_measure << std::endl;
            /* Compute control signal */
            float out1, out2;
            out1 = PIDController_Update(&panpid, target, horizontal_measure);
            out2 = PIDController_Update(&tiltpid, target, vertical_measure);

            
            std::cout << "Output #1 is " << out1 << std::endl;
            std::cout << "Output #2 is " << out2 << std::endl;


            
            // if (pid.out > -0.1 && pid.out < 0.1){
            if(panpid.out > -tolerance && panpid.out < tolerance){
                // move forward
                
                if (pan > 90)
                    move_msg.angular.z = -0.7;
                else
                    move_msg.angular.z = 0.7; 
                
                move_pub_->publish(move_msg);
                std::this_thread::sleep_for(300ms);  // to turn
                move_msg.angular.z = 0;
                if (pan == 90)
                    move_msg.linear.x = 0.4;

                
            }

            //move_msg.angular.z = -pid.out;
            move_pub_->publish(move_msg); 
            move_pantilt.x = out1 + pan;   // add or subtract degrees depending of the camera pan pose
            move_pantilt.y = -out2 + tilt;
            pantilt_move_pub_->publish(move_pantilt);
            //rate.sleep();
             
            
        }else{
        
            move_msg.linear.x = 0;
            move_msg.angular.z = 0;
            move_pub_->publish(move_msg);
        }

        
    }
}

void TrackingImages::info_callback(const vision_msgs::msg::VisionInfo::ConstSharedPtr msg){
    std::string location = msg->database_location;
    int32_t version = msg->database_version;
    std::string method = msg->method;
    ROS_INFO("location %s, version: %i, method: %s", location.c_str(), version, method.c_str());

}

void TrackingImages::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg){
    image_height = msg->height;
    image_width = msg->width;
}


float TestSystem_Update(float input){
    static float output = 0.0f; 
    static const float alpha = 0.02f;

    return output;
}

void TrackingImages::detection_status_callback(const std_msgs::msg::Bool::ConstSharedPtr msg){
    is_object_detected = msg->data;
    //std::cout << " I am checking if I detect something ..." << std::endl;
    geometry_msgs::msg::Twist move_msg;
    car_interfaces::msg::Pantilt pantilt_msg;
    return;
    if(!is_object_detected){
         // go to scan mode
        std::cout << "SCANNING THE AREA" << std::endl;
        move_msg.angular.z=.4;
        move_pub_->publish(move_msg);
        if (pan > 90){
            pantilt_msg.x = pan - 5;
            
        }
        else{
            pantilt_msg.x = pan + 5;
            
        }
        if (tilt > 90)
            pantilt_msg.y = tilt - 5;
        else 
            pantilt_msg.y = tilt + 5;

        pantilt_move_pub_->publish(pantilt_msg);        
    }
    //move_msg.angular.z = 0;  // reset after scanning
    
}

void TrackingImages::pantilt_pose_callback(const car_interfaces::msg::Pantilt::ConstSharedPtr msg){
    pan = msg->x;
    tilt = msg->y;
}