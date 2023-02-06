#include "JetsonGPIO.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "car_interfaces/msg/encoder.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

/*    TESTING SERVICES   
*
*   ros2 service call /reset_encoder std_srvs/srv/SetBool "{data: true}"
*
*/

/* static variables */
static int pos1=0;
static int pos2=0;
static int pos3=0;

class MotorEncoder : public rclcpp::Node{
public:
    MotorEncoder();
    ~MotorEncoder();

private:
    rclcpp::Publisher<car_interfaces::msg::Encoder>::SharedPtr encoder_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr encoder_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
    /* Motor order counterclockwise. Motor #3 is in the front */
    /* Motor3 - pin definitions */
    const int hall_pin_1 = 13;  // board pin 33
    static const int hall_pin_2 = 11;  // Board pin 31

    /* Motor2 - pin definitions */
    const int hall_pin_3 = 18;  // BOARD pin 12
    static const int hall_pin_4 = 7;  // BOARD pin 26

    /* Motor1 - pin definitions */
    const int hall_pin_5 = 20;  // BOARD pin 38     
    static const int hall_pin_6 = 16;   // BOARD pin 36

    void encoder_callback();
    void GPIO_Settings();
    void init_publisher();
    static void getTicksM1(const std::string& channel);
    static void getTicksM2(const std::string& channel);
    static void getTicksM3(const std::string& channel);
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorEncoder>());
    rclcpp::shutdown();

    return 0;
}

MotorEncoder::MotorEncoder() : Node("motorencoder_node"){
    init_publisher();

    /* Service handler */
    auto reset_encoder = [this]( const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                            const std::shared_ptr<std_srvs::srv::SetBool::Response> response)->void{

        (void) request_header;
        if( request->data){
            RCLCPP_INFO(this->get_logger()," Incomming request to RESET encoders value");
            pos1 = 0;
            pos2 = 0;
            pos3 = 0;
            response->success = true;
            response->message = "Encoders have been reset!!";  /*  ------->   Sending back service response */
            
        }
    };

    /* Init services to reset the encoders count */
    encoder_srv_ = this->create_service<std_srvs::srv::SetBool>("reset_encoder", reset_encoder);
    GPIO_Settings();
    
}

void MotorEncoder::GPIO_Settings(){
    GPIO::setmode(GPIO::BCM);
    /* Set the Output pin with optional initial state of HIGH */
    GPIO::setup(hall_pin_1, GPIO::IN);
    GPIO::setup(hall_pin_2, GPIO::IN);

    GPIO::setup(hall_pin_3, GPIO::IN);
    GPIO::setup(hall_pin_4, GPIO::IN);

    GPIO::setup(hall_pin_5, GPIO::IN);
    GPIO::setup(hall_pin_6, GPIO::IN); 

    GPIO::add_event_detect(hall_pin_1, GPIO::Edge::RISING, getTicksM1);
    GPIO::add_event_detect(hall_pin_3, GPIO::Edge::RISING, getTicksM2);
    GPIO::add_event_detect(hall_pin_5, GPIO::Edge::RISING, getTicksM3);

}

void MotorEncoder::init_publisher(){
    encoder_pub_ = this->create_publisher<car_interfaces::msg::Encoder>("encoder", 500);
    timer_ = this->create_wall_timer(0ms, std::bind(&MotorEncoder::encoder_callback, this) );

}

void MotorEncoder::getTicksM1(const std::string& channel){
    (void)channel;
    int hall = GPIO::input(hall_pin_2);
    if (hall > 0)
        pos3++;
    else
        pos3--;
    
}

void MotorEncoder::getTicksM2(const std::string& channel){
    (void)channel;
    int hall = GPIO::input(hall_pin_4);
    if (hall > 0)
        pos2++;
    else
        pos2--;
    

}

void MotorEncoder::getTicksM3(const std::string& channel){
    (void)channel;
    int hall = GPIO::input(hall_pin_6);
    if (hall > 0)
        pos1++;
    else
        pos1--;

}


void MotorEncoder::encoder_callback(){

    car_interfaces::msg::Encoder msg;
    msg.m1_ticks = pos1;
    msg.m2_ticks = pos2;
    msg.m3_ticks = pos3;
    

    encoder_pub_->publish(msg);
}

MotorEncoder::~MotorEncoder(){
    GPIO::cleanup();
}