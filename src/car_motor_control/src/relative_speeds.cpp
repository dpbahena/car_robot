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
    float x_comp{}, y_comp{}, w_comp{};  // componets of the direction vector
    bool velocity_received;
    float average_velocity;  // from linear.x, linear.y and angular.z   
    //float velocity;
    bool isLinear = false;
    bool isAngular = false;    //velocity received is either linear or angular 
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
    //void set_direction_components(int direction);
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
    velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("vel_cmds", 10, std::bind(&RelativeSpeeds::velocity_callback, this, _1));
}

void RelativeSpeeds::init_publishers(){
    relative_speeds_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("relative_speeds", 1);
    timer_ = this->create_wall_timer(100ms, std::bind(&RelativeSpeeds::relative_speeds_callback, this));
}

void RelativeSpeeds::velocity_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg){
    
    if(!msg->linear.x && !msg->linear.y && !msg->angular.z){
        x_comp = 0;
        y_comp = 0;
        w_comp = 0;
        velocity_received = false;
        return;
    }

    auto angle = atan(msg->linear.x/msg->linear.y);
    float bearing{};
    
    average_velocity = (abs(msg->linear.x) + abs(msg->linear.y) + abs(msg->angular.z))/3;
    
    if (msg->angular.z != 0)
        isAngular = true;
  
        
    if (msg->linear.x > 0 && msg->linear.y == 0){  // moved up (North) only
        /* x_comp = 0;
        y_comp = 1;
        w_comp = 0; */
        bearing = M_PI/2;
        isLinear = true;
        //std::cout << "[" << msg->linear.x << ", " << msg->linear.y << "]" << " UP" << "  Bearing = " << bearing <<  std::endl;

    }
    else if (msg->linear.x < 0 && msg->linear.y == 0){  //moved down (South) only
        /* x_comp = 0;
        y_comp = -1;
        w_comp = 0; */
        bearing = 3*M_PI/2;
        isLinear = true;
        //std::cout << "[" << msg->linear.x << ", " << msg->linear.y << "]" << " DOWN" <<  "  Bearing = " << bearing << std::endl;
    }

    else if(msg->linear.y > 0  && msg->linear.x == 0){   // moved to the right (East) only
        /* x_comp = 1;
        y_comp = 0;
        w_comp = 0; */
        bearing = 0;
        isLinear = true;
        //std::cout << "[" << msg->linear.x << ", " << msg->linear.y << "]" << " RIGHT" <<  "  Bearing = " << bearing << std::endl;
    }

    else if(msg->linear.y < 0 && msg->linear.x == 0 ){   // moved to the left (West) only
        /* x_comp = -1;
        y_comp = 0;
        w_comp = 0; */
        bearing = M_PI;
        isLinear = true;
        //std::cout << "[" << msg->linear.x << ", " << msg->linear.y << "]" << " LEFT" <<  "  Bearing = " << bearing << std::endl;
    }
    else{   // get the bearing angle
        if(msg->linear.x > 0 && msg->linear.y > 0){   // Quadrant 1
            bearing = M_PI/2 - angle;
            std::cout << "[" << msg->linear.x << ", " << msg->linear.y << "]" << " Quadrant I" <<  std::endl;
        }
        if(msg->linear.x < 0 && msg->linear.y > 0){   // Quadrant II
            bearing = M_PI/2 - angle;
            std::cout << "[" << msg->linear.x << ", " << msg->linear.y << "]" << " Quadrant II" <<  std::endl;
        }
        if(msg->linear.x < 0 && msg->linear.y < 0){   // Quadrant III
            bearing = 3*M_PI/2 - angle;
            std::cout << "[" << msg->linear.x << ", " << msg->linear.y << "]" << " Quadrant III" <<  std::endl;
        }
        if(msg->linear.x > 0 && msg->linear.y < 0){   // Quadrant IV
            bearing = 3*M_PI/2 - angle;
            std::cout << "[" << msg->linear.x << ", " << msg->linear.y << "]" << " Quadrant IV" <<  std::endl;
        }
        isLinear = true;
    }

    // if(msg->linear.x || msg->linear.y){   // If this axes were moved then do the calculations. Just in case only rotation was activated
    //     x_comp = cos(bearing);
    //     y_comp = sin(bearing);
    // }

    x_comp = cos(bearing);
    y_comp = sin(bearing);
    w_comp = msg->angular.z;
    
    set_relative_speeds();
    velocity_received = true;

    
}

// void RelativeSpeeds::set_direction_components(int direction){

//     switch (direction){
//         case FRONT:
//             x_comp = 0;
//             y_comp = 1;
//             w_comp = 0;
//             break;
//         case BACK:
//             x_comp = 0;
//             y_comp = -1;
//             w_comp = 0;
//             break;
//         case TURN_LEFT:
//             x_comp = 0;
//             y_comp = 0;
//             w_comp = -1;
//             break;
//         case TURN_RIGHT:
//             x_comp = 0;
//             y_comp = 0;
//             w_comp = 1;
//             break;
//         case SIDE_RIGHT:
//             x_comp = -1;
//             y_comp = 0;
//             w_comp = 0;
//             break;
//         case SIDE_LEFT:
//             x_comp = 1;
//             y_comp = 0;
//             w_comp = 0;
//             break;
//         default:
//             x_comp = 0;
//             y_comp = 0;   // stop the car
//             w_comp = 0;
//     }

//     set_relative_speeds();
//     velocity_received = true;

// }

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
    
    //std::cout << "Components: {"<< x_comp <<", " << y_comp << ", " << w_comp << "}" << std::endl;

    // extract each indiviual motor speed from the matrix and publish it.
    m1 = motorSpeeds.coeff(0,0);
    m2 = motorSpeeds.coeff(1,0);
    m3 = motorSpeeds.coeff(2,0);
    //std::cout << "motorSpeeds :" << m1 << " and " << m2 << " and " << m3 << std::endl;
}

void RelativeSpeeds::relative_speeds_callback(){

    /* variable to be published */
    std_msgs::msg::Float32MultiArray motorspeeds; // holds the relative speeds of each motor in an array

    auto pwm = mapVelocity(average_velocity);
    //auto pwm = mapVelocity(velocity);

    motorspeeds.data.push_back(m1);
    motorspeeds.data.push_back(m2);
    motorspeeds.data.push_back(m3);
    
    /* 4th element of array is the adjusted pwm */
    motorspeeds.data.push_back(pwm);      

    if(velocity_received){
        relative_speeds_pub_->publish(motorspeeds);
        
    }
        
}

/* Set limits for actual velocity multiplier - higher values result in values over 255 
* - lower values result in motors not responding due to low voltage 
*/
float RelativeSpeeds::mapVelocity(float v){
    if(isLinear && !isAngular){    // linear only
        isLinear = false;
        if(v > 2.0)
            v = 2.0;   // max of 2.0  for linear velocity
        if(v < .23)
            v = .23;   // min of .22  for linear velocity
        return maxPwm * v / maxLVelocity;
    }else if(isAngular && !isLinear){  // angular only
        isAngular = false;
        if(v > 4.4)
            v = 4.4;   // max of 4.4  for ANGULAR velocity
        if(v < 0.35)
            v = 0.0;   // min of .35  for ANGULAR velocity stop here!
        return maxPwm * v / maxAVelocity;
    }
    else{ 
                     // combination of both linear and angular velocity given
        isLinear = false;
        isAngular = false;             
        if(v > 3.2)
            v = 3.2; //  max of 3.2 for combination of linear and angular
        if(v < 0.29)
            v = 0.29;  // min speed for combination of linear and angular
        
        return maxPwm * v /((maxLVelocity + maxAVelocity)/2);

    }

}


// void RelativeSpeeds::velocity_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg){
    
//     int direction;
//     if(msg->linear.x > 0){
//         direction = FRONT;
//         velocity = msg->linear.x;
//         isLinear = true;
//     }else if(msg->linear.x < 0){
//         direction = BACK;
//         velocity = msg->linear.x;
//         isLinear = true;
//     }else if(msg->linear.y > 0){
//         direction = SIDE_LEFT;
//         velocity = msg->linear.y;
//         isLinear = true;
//     }else if(msg->linear.y < 0){
//         direction = SIDE_RIGHT;
//         velocity = msg->linear.y;
//         isLinear = true;
//     }else if(msg->angular.z > 0){
//         direction = TURN_RIGHT;
//         velocity = msg->angular.z;
//         isAngular = true;
//     }else if(msg->angular.z < 0){
//         direction = TURN_LEFT;
//         velocity = msg->angular.z;
//         isAngular = true;
//     }else
//         direction = STOP;
    
//     set_direction_components(direction);

// }