#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "car_interfaces/msg/pantilt.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>


using std::placeholders::_1;
using namespace std::chrono_literals;


class JoyControl : public rclcpp::Node{
public:
    JoyControl(): Node("joycontrol_node"){
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&JoyControl::joy_callback, this, _1));
        pantilt_sub_ = this->create_subscription<car_interfaces::msg::Pantilt>("pantilt_pose",30,std::bind(&JoyControl::pantilt_pose_callback, this, _1));
        
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("vel_cmds", 1);
        timer_ = this->create_wall_timer(100ms, std::bind(&JoyControl::velocity_callback, this));

        pantilt_move_pub_ = this->create_publisher<car_interfaces::msg::Pantilt>("pantilt_cmds", 15);
    }

private:

rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
rclcpp::Subscription<car_interfaces::msg::Pantilt>::SharedPtr pantilt_sub_;

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
rclcpp::Publisher<car_interfaces::msg::Pantilt>::SharedPtr pantilt_move_pub_;
rclcpp::TimerBase::SharedPtr timer_, timer2_;

/* This data goes into the Twist command w_comp is the angular vel for z*/
float x_comp, y_comp, w_comp;

/* Pantilt camera control */
float pan_comp, tilt_comp;
int pan, tilt;   // pantilt camera pose
void pantilt_pose_callback(const car_interfaces::msg::Pantilt::ConstSharedPtr msg);


float turbo = 3.0;

void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg);
void velocity_callback();



};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyControl>());
    rclcpp::shutdown();

    return 0;
}


void JoyControl::joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg){
    /* Use dead-guy buttons (xbox) */
    if(msg->buttons.at(6) || msg->buttons.at(7)){
        x_comp = turbo * -msg->axes.at(2);  // reverse with a sighn x-axis left is negative, right is positive
        y_comp = turbo *  msg->axes.at(3);   // up and down 
        /* 0.5 makes rotation a little slower in comparison to x and y linear velocity*/ 
        w_comp = turbo * 0.5 * -msg->axes.at(0);  // reverse sign too for rotation. It will be used  for angular velocity for z
        /* turbo makes 3 degrees steps for pantilt movements */
        pan_comp = turbo * msg->axes.at(6);   // controls pan rotation (camera)
        tilt_comp = turbo *  msg->axes.at(7);  // controls tilt rotation (camera)

    }else{
        x_comp = 0;
        y_comp = 0;
        w_comp = 0;
        pan_comp = 0;   
        tilt_comp = 0;
    }


}

void JoyControl::velocity_callback(){
    auto velocities = geometry_msgs::msg::Twist();
    auto pantilt_move = car_interfaces::msg::Pantilt();


    velocities.linear.x = x_comp;
    velocities.linear.y = y_comp;
    velocities.angular.z = w_comp;

    pantilt_move.x = pan_comp + pan;  /*  add or subtract degrees depending of the camera pan pose */
    pantilt_move.y = tilt_comp + tilt;

    pantilt_move_pub_->publish(pantilt_move);
    velocity_pub_->publish(velocities);
    
}




void JoyControl::pantilt_pose_callback(const car_interfaces::msg::Pantilt::ConstSharedPtr msg){
    pan = msg->x;
    tilt = msg->y;
}