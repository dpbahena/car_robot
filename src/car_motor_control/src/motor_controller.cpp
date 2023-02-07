#include "rclcpp/rclcpp.hpp"
#include "../drivers/AdafruitMotorHatLibrary/adafruitmotorhat.h"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

/* !!---- TESTING ------------------>
*   
*   ros2 run car_control motorcontroller  
    or use the launch file below: 
    ros2 launch car_control robot.launch.py 
*   ros2 topic pub --once relative_speeds std_msgs/msg/Float32MultiArray "{data: { 0.58,-0.58, 0.0, 255}}"   // to start motors ONCE at 255 value speed (max)
*   
*            
*    LINEAR:   MAX 2.0  results in 255 actual pwm in motors
     LINEAR:   MIN 0.23  results in 29 actual pwm in motors that is still usefull
     ANGULAR:  MAX 4.4  results in 212 actual pwm in ALL motors still SUSTAINABLE given that 2.0 max Amps
     ANGULAR:  MIN 0.6  results in 29  actual pwm in all motors that is still usefull
*/

using std::placeholders::_1;
using namespace std::chrono_literals;

class MotorController : public rclcpp::Node{
public:
    MotorController(AdafruitMotorHAT *hat);

private:
    int maxSpeed;   //max adjusted PWM  255 but motors shotdown due to Amp max over 2.0 amps drawn 
    int maxTime;  // max number of seconds the motors will shutdown regardless

    bool motorsOn = false;
    double start, end;  // measure time intervals

    std::shared_ptr <AdafruitDCMotor> motor1, motor2, motor3;
    rclcpp::TimerBase::SharedPtr timer_, timer2_;
    float m1_rs, m2_rs, m3_rs;   //holds relative speeds for each motor
    float lx,ly,az;   // linear velocity x, y, (z won't be used) and angular velocity z  (x and y angular velocity won't be used)

    //declare subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr rel_speeds_sub_;

    void velocity_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
    void relative_speeds_callback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg);
    void parameter_callback();

    void init_subscribers();

    void start_motors();
    void stop_motors();
    void timer_callback();

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    AdafruitMotorHAT hat;  // hat(96, 1600) is the default
    rclcpp::spin(std::make_shared<MotorController>(&hat));
    rclcpp::shutdown();
    return 0;
}

MotorController::MotorController(AdafruitMotorHAT *hat) : Node("motorcontroller_node"){
    init_subscribers();
    /* Declare parameters */

    this->declare_parameter("maxtime", 3);
    /* Get parameters using a callback function to update new value */
    timer2_ = this->create_wall_timer(1000ms, std::bind(&MotorController::parameter_callback, this));

    /* Initialize motors */
    motor1 = hat->getMotor(1);
    motor2 = hat->getMotor(2);
    motor3 = hat->getMotor(3);

    /* timer_ calls the function every 10th of a second and only if the motors are ON */
    timer_ = this->create_wall_timer(100ms, [&](){if(motorsOn) MotorController::timer_callback();});

}

void MotorController::init_subscribers(){
    velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("vel_cmds", 1, std::bind(&MotorController::velocity_callback, this, _1));
    rel_speeds_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("relative_speeds", 1, std::bind(&MotorController::relative_speeds_callback, this, _1));


}

void MotorController::parameter_callback(){
    this->get_parameter("maxtime", maxTime);
}

void MotorController::velocity_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg){
    lx = msg->linear.x;
    ly = msg->linear.y;
    az = msg->angular.z;

    if(msg->linear.x == 0 && msg->linear.y == 0 && msg->angular.z == 0){
        stop_motors();
    }
}

void MotorController::relative_speeds_callback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg){
    /* Get relative speeds for each motor */
     m1_rs = msg->data[0];
     m2_rs = msg->data[1];
     m3_rs = msg->data[2];
    /* Gets the adjusted pwm that may result to max of 255 actual pwm in each motor */
     maxSpeed = msg->data[3];  

     start_motors();
}

void MotorController::stop_motors(){
    end = this->get_clock()->now().seconds();
    motor1->run(AdafruitDCMotor::kRelease);
    motor2->run(AdafruitDCMotor::kRelease);
    motor3->run(AdafruitDCMotor::kRelease);
    RCLCPP_INFO(this->get_logger(), "************ time running : %f *****************", end - start);
}

void MotorController::start_motors(){
    int a1{}, b1{}, c1{}, a2{}, b2{}, c2{};
    /* Set speeds for each motor */
    if(m1_rs>= 0){
    	a1 = m1_rs*maxSpeed;
        motor1->setSpeed(a1);
    }else{
        a2 =  -m1_rs*maxSpeed;
    	motor1->setSpeed(a2);
    }
    if(m2_rs >= 0){
        b1 = m2_rs*maxSpeed;
    	motor2->setSpeed(b1);
    }else{
        b2 = -m2_rs*maxSpeed;
    	motor2->setSpeed(b2);
    }
    if(m3_rs>= 0){
        c1 = m3_rs*maxSpeed;
    	motor3->setSpeed(c1);
    }else{
        c2 = -m3_rs*maxSpeed;
    	motor3->setSpeed(c2);
    }

    RCLCPP_INFO(this->get_logger(),"%d, %d, %d, %d, %d, %d", a1, a2, b1, b2, c1, c2);

    /* start timer and motors */
    start = this->get_clock()->now().seconds();

    if(motor1 && motor2 && motor3){    
        if(m1_rs>0)  // forward ?
            motor1->run(AdafruitDCMotor::kForward);
        if(m1_rs<0)  // backward ?
            motor1->run(AdafruitDCMotor::kBackward); 
        if(m2_rs>0) // forward ?
            motor2->run(AdafruitDCMotor::kForward);
        if(m2_rs<0) // backward ?
            motor2->run(AdafruitDCMotor::kBackward);
        if(m3_rs>0) // forward ?
            motor3->run(AdafruitDCMotor::kForward);
        if(m3_rs<0) // backward ?
            motor3->run(AdafruitDCMotor::kBackward);

    }

    /* indicate that motors are on */
    motorsOn = true;

}

void MotorController::timer_callback(){
    auto check_time = this->get_clock()->now().seconds();
    auto timeElapsed = check_time - start;

    if(timeElapsed >= maxTime){
        stop_motors();
        motorsOn = false;  // change status. Motors are off
    }
}