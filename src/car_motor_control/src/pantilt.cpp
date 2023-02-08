#include "pca9685SetUp.h"
#include "rclcpp/rclcpp.hpp"
#include "car_interfaces/msg/pantilt.hpp"


/*
*
*  ros2 topic pub  --once /pantilt_move car_interfaces/msg/Pantilt "{x: 20, y: 80}"
*
*/

using std::placeholders::_1;
using namespace std::chrono_literals;

class PanTilt : public rclcpp::Node{
public:
    PanTilt();
private:
    PCA9685SetUp servos_;
    /* Publish current pantilt position (pose) */
    rclcpp::Publisher<car_interfaces::msg::Pantilt>::SharedPtr pantilt_pub_;
    
    /* Get movement commands */
    rclcpp::Subscription<car_interfaces::msg::Pantilt>::SharedPtr pantilt_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    /* max and min travel in degrees : pan-tilt values */
    const int tilt_max = 135;
    const int tilt_min = 0;
    const int pan_max = 177;
    const int pan_min = 0;

    float pose_x{}, pose_y{};  // holds pantilt pose
    int degrees;
    void move_pantilt_callback(const car_interfaces::msg::Pantilt::ConstSharedPtr msg);
    void update_pantilt_pose();
    void init_pantilt();
    void pan(int x);
    void tilt(int y);
    void pose_callback();
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PanTilt>());
    rclcpp::shutdown();

    return 0;
}

PanTilt::PanTilt(): Node("pantilt_node"){
    /* init subscribers and publishers */
    pantilt_sub_ = this->create_subscription<car_interfaces::msg::Pantilt>("pantilt_cmds", 1, std::bind(&PanTilt::move_pantilt_callback, this, _1));
    pantilt_pub_ = this->create_publisher<car_interfaces::msg::Pantilt>("pantilt_pose", 1);
    timer_ = this->create_wall_timer(10ms, std::bind(&PanTilt::pose_callback, this));
    pose_x = 90;
    pose_y = 90;
    servos_.moveServo(0, pose_x);  // center pan
    servos_.moveServo(1, pose_y);  // center tilt

}

void PanTilt::move_pantilt_callback(const car_interfaces::msg::Pantilt::ConstSharedPtr msg){
    int x, y;
    if (msg->x > pan_max)
        x = pan_max;
    else if (msg->x < pan_min)
        x = pan_min;
    else
        x = msg->x;
    
    if (msg->y > tilt_max)
        y = tilt_max;
    else if (msg->y < tilt_min)
        y = tilt_min;
    else
        y = msg->y;

    // pan and tilt independently
    std::thread{std::bind(&PanTilt::pan, this, _1), x}.detach();
    std::thread{std::bind(&PanTilt::tilt, this, _1), y}.detach();
    
    //servos_.moveServo(0,x);  // pan
    //servos_.moveServo(1,y);  // tilt
}

void PanTilt::pan(int x){
    rclcpp::Rate rate(60);
    if (pose_x > x){  // go backwards
        for(int i = pose_x; i >= x; i--){
            servos_.moveServo(0,i);
            pose_x = i;
            rate.sleep();
        }
    }else{    // go forward
        for(int i = pose_x; i <= x; i++){
            servos_.moveServo(0,i);
            pose_x = i;
            rate.sleep();
        }   
    }
    //servos_.moveServo(0,x);   // uncomment for quick movement
}

void PanTilt::tilt(int y){
    rclcpp::Rate rate(60);
    if (pose_y > y){  // go backwards
        for(int i = pose_y; i >= y; i--){
            servos_.moveServo(1,i);
            pose_y = i;
            rate.sleep();
        }
    }else{
        for(int i = pose_y; i <= y; i++){
            servos_.moveServo(1,i);
            pose_y = i;
            rate.sleep();
        }
    }
    
    //servos_.moveServo(1,y);  // uncomment for quick movement
}

void PanTilt::pose_callback(){
    car_interfaces::msg::Pantilt pose;
    pose.x = pose_x;
    pose.y = pose_y;

    pantilt_pub_->publish(pose);
}

