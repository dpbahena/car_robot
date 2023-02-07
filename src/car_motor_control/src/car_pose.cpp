#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "car_interfaces/msg/encoder.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class CarPose : public rclcpp::Node{
public:
    CarPose();

private:
    int m1,m2,m3;
    float v1,v2,v3;

    const int ticks = 75;
    const float WheelDiameter = 2.244;  // inches

    rclcpp::TimerBase::SharedPtr timer_;



    rclcpp::Subscription<car_interfaces::msg::Encoder>::SharedPtr encoder_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr relative_speeds_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;

    void encoder_callback(const car_interfaces::msg::Encoder::SharedPtr msg);
    void relative_speeds_callback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg);
    void distance_callback();

    float get_distance();
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto carpose = std::make_shared<CarPose>();
    rclcpp::spin(carpose);
    rclcpp::shutdown();

    return 0;
}

CarPose::CarPose(): Node("carpose_node"){
    // init subscribers
    encoder_sub_  = this->create_subscription<car_interfaces::msg::Encoder>("encoder", 10, std::bind(&CarPose::encoder_callback, this, _1));
    
    relative_speeds_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("relative_speeds", 10, std::bind(&CarPose::relative_speeds_callback, this, _1));
    encoder_sub_ = this->create_subscription<car_interfaces::msg::Encoder>("encoder",10, std::bind(&CarPose::encoder_callback, this, _1) );

    // init publishers
    distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("distance", 10);
    timer_= this->create_wall_timer(100ms, std::bind(&CarPose::distance_callback, this));
    
    
}

void CarPose::encoder_callback(const car_interfaces::msg::Encoder::SharedPtr msg){
    m1 = msg->m1_ticks;
    m2 = msg->m2_ticks;
    m3 = msg->m3_ticks;

    //RCLCPP_INFO(this->get_logger(),"testing testing testing testing");
    //RCLCPP_INFO(this->get_logger(),"m1: %i, m2: %i, m3: %i", m1, m2, m3);
}
void CarPose::relative_speeds_callback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg){
    v1 = msg->data[0];
    v2 = msg->data[1];
    v3 = msg->data[2];

    //RCLCPP_INFO(this->get_logger(),"v1: %f, v2: %f, v3: %f", v1, v2, v3);
}

void CarPose::distance_callback(){
   
    std_msgs::msg::Float32 distance;
    distance.data = get_distance();
    distance_pub_->publish(distance);  
    
}

float CarPose::get_distance(){
    auto dist_vector = m1*v1 + m2*v2 + m3*v3;
    auto vector_ticks = ticks * cos(M_PI/6);
    auto turns = dist_vector/vector_ticks;
    auto circumference = M_PI * WheelDiameter;
    auto distance = fabs( 0.90*(circumference * turns)/12);  // p0.90 it is an adjuster depending of the terrain. .90 makes the car actually travel the distance commanded
    //RCLCPP_INFO(this->get_logger(),"Distvec: %f, vectorThicks: %f, turns: %f, circum: %f, distance: %f", dist_vector, vector_ticks, turns, circumference, circumference* turns);
    return distance;   // send distance in feet

}