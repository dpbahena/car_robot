#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "../libraries/Eigen/Dense"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace Eigen;

enum eDirections {STOP = 0, FRONT, BACK, TURN_LEFT, TURN_RIGHT, SIDE_LEFT, SIDE_RIGHT};

class RelativeSpeeds : public rclcpp::Node{
public:
    RelativeSpeeds();

private:
    float x_comp, y_comp, w_comp;  // componets of the direction vector
    bool velocity_received = false;
    float velocity;   // holds either linear or angular velocity
    bool isLinear, isAngular;    //velocity received is either linear or angular 
    float m1, m2, m3;  // receives the relative speeds for each motor
    const int maxPwm = 440;
    const float maxLVelocity = 2.0; // feet per seconds
    const float maxAVelocity = 3.0; // radians per second

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr relative_speeds_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void init_subscribers();
    void init_publishers();

    void velocity_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
    void relative_speeds_callback();
    /* Set relative speeds for each motor */
    void set_relative_speeds(); 
    void set_direction_components(int direction);
    float mapVelocity(float v);
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RelativeSpeeds>());
    rclcpp::shutdown();

    return 0;
}

RelativeSpeeds::RelativeSpeeds() : Node("relativespeeds_node"){
    init_subscribers();
    init_publishers();  
}

void RelativeSpeeds::init_subscribers(){
    velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("vel_cmds", 1, std::bind(&RelativeSpeeds::velocity_callback, this, _1));
}

void RelativeSpeeds::init_publishers(){
    relative_speeds_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("relative_speeds", 1);
    timer_ = this->create_wall_timer(100ms, std::bind(&RelativeSpeeds::relative_speeds_callback, this));
}

void RelativeSpeeds::velocity_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg){
    
    int direction;
    if(msg->linear.x > 0){
        direction = FRONT;
        velocity = msg->linear.x;
        isLinear = true;
    }else if(msg->linear.x < 0){
        direction = BACK;
        velocity = msg->linear.x;
        isLinear = true;
    }else if(msg->linear.y > 0){
        direction = SIDE_LEFT;
        velocity = msg->linear.y;
        isLinear = true;
    }else if(msg->linear.y < 0){
        direction = SIDE_RIGHT;
        velocity = msg->linear.y;
        isLinear = true;
    }else if(msg->angular.z > 0){
        direction = TURN_RIGHT;
        velocity = msg->angular.z;
        isAngular = true;
    }else if(msg->angular.z < 0){
        direction = TURN_LEFT;
        velocity = msg->angular.z;
        isAngular = true;
    }else
        direction = STOP;
    
    set_direction_components(direction);

}

void RelativeSpeeds::set_direction_components(int direction){

    switch (direction){
        case FRONT:
            x_comp = 0;
            y_comp = 1;
            w_comp = 0;
            break;
        case BACK:
            x_comp = 0;
            y_comp = -1;
            w_comp = 0;
            break;
        case TURN_LEFT:
            x_comp = 0;
            y_comp = 0;
            w_comp = -1;
            break;
        case TURN_RIGHT:
            x_comp = 0;
            y_comp = 0;
            w_comp = 1;
            break;
        case SIDE_RIGHT:
            x_comp = -1;
            y_comp = 0;
            w_comp = 0;
            break;
        case SIDE_LEFT:
            x_comp = 1;
            y_comp = 0;
            w_comp = 0;
            break;
        default:
            x_comp = 0;
            y_comp = 0;   // stop the car
            w_comp = 0;
    }

    set_relative_speeds();
    velocity_received = true;

}

void RelativeSpeeds::set_relative_speeds(){

    MatrixXf m {
    {-0.33,0.58,0.33},
    {-0.33,-0.58,0.33},
    {0.67, 0, 0.33}
    };
    MatrixXf n {
        {x_comp},
        {y_comp},
        {w_comp}
    };

    MatrixXf motorSpeeds = m*n;
    
   

    // extract each indiviual motor speed from the matrix and publish it.
    m1 = motorSpeeds.coeff(0,0);
    m2 = motorSpeeds.coeff(1,0);
    m3 = motorSpeeds.coeff(2,0);

}

void RelativeSpeeds::relative_speeds_callback(){

    /* variable to be published */
    std_msgs::msg::Float32MultiArray motorspeeds; // holds the relative speeds of each motor in an array

    auto pwm = mapVelocity(abs(velocity));


    motorspeeds.data.push_back(m1);
    motorspeeds.data.push_back(m2);
    motorspeeds.data.push_back(m3);
    
    /* 4th element of array is the adjusted pwm */
    motorspeeds.data.push_back(pwm);      

    if(velocity_received)
        relative_speeds_pub_->publish(motorspeeds);
}

float RelativeSpeeds::mapVelocity(float v){
    if(isLinear){
        isLinear = false;
        if(v > 2.0)
            v = 2.0;   // max of 2.0  for linear velocity
        if(v < .23)
            v = .23;   // min of .22  for linear velocity
        return maxPwm * v / maxLVelocity;
    }else{
        isAngular = false;
        if(v > 4.4)
            v = 4.4;   // max of 4.4  for ANGULAR velocity
        if(v < 0.35)
            v = 0.0;   // min of .35  for ANGULAR velocity stop here!
        return maxPwm * v / maxAVelocity;
    }

}