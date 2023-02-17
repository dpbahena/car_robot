#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>


using std::placeholders::_1;
using namespace std::chrono_literals;


class JoyControl : public rclcpp::Node{
public:
    JoyControl(): Node("joycontrol_node"){
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&JoyControl::joy_callback, this, _1));
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("vel_cmds", 1);
        timer_ = this->create_wall_timer(100ms, std::bind(&JoyControl::velocity_callback, this));
    }

private:

rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
rclcpp::TimerBase::SharedPtr timer_;

/* This data goes into the Twist command w_comp is the angular vel for z*/
float x_comp, y_comp, w_comp;

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
        x_comp = -msg->axes.at(2);  // reverse with a sighn x-axis left is negative, right is positive
        y_comp =  msg->axes.at(3);   // up and down  
        w_comp = -msg->axes.at(0);  // reverse sign too for rotation. It will be used  for angular velocity for z
    }else{
        x_comp = 0;
        y_comp = 0;
        w_comp = 0;
    }


}

void JoyControl::velocity_callback(){
    auto velocities = geometry_msgs::msg::Twist();
    velocities.linear.x = x_comp;
    velocities.linear.y = y_comp;
    velocities.angular.z = w_comp;

    velocity_pub_->publish(velocities);
}

