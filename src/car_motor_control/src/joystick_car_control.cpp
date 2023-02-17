#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>


using std::placeholders::_1;
using namespace std::chrono_literals;


class JoyControl : public rclcpp::Node{
public:
    JoyControl(): Node("joycontrol_node"){
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("vel_cmds", 1, std::bind(&JoyControl::joy_callback, this, _1));
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

void get_x_y_w_components(float x_disp, float y_disp, float w_disp);

};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyControl>());
    rclcpp::shutdown();

    return 0;
}


void JoyControl::joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg){
    x_comp = -msg->axes.at(2);  // reverse with a sighn x-axis left is negative, right is positive
    y_comp =  msg->axes.at(3);   // up and down  
    w_disp = -msg->axes.at(0);  // reverse sign too for rotation. It will be used  for angular velocity for z
}

void JoyControl::get_x_y_w_components(float x_disp, float y_disp, float w_disp){
    auto angle = atan(x_disp/y_disp);
    float bearing{};
    
    if (w_disp > 0)
        w_comp = 1;
    if (w_disp < 0)
        w_comp = -1;
    if (w_disp == 0)
        w_comp = 0;

    if (y_disp > 0 && x_disp == 0){  // moved up (North) only
        /* x_comp = 0;
        y_comp = 1;
        w_comp = 0; */
        bearing = M_PI/2;
        //std::cout << "[" << x_disp << ", " << y_disp << "]" << " UP" << "  Bearing = " << bearing <<  std::endl;

    }
    else if (y_disp < 0 && x_disp == 0){  //moved down (South) only
        /* x_comp = 0;
        y_comp = -1;
        w_comp = 0; */
        bearing = 3*M_PI/2;
        //std::cout << "[" << x_disp << ", " << y_disp << "]" << " DOWN" <<  "  Bearing = " << bearing << std::endl;
    }

    else if(x_disp > 0  && y_disp == 0){   // moved to the right (East) only
        /* x_comp = 1;
        y_comp = 0;
        w_comp = 0; */
        bearing = 0;
        //std::cout << "[" << x_disp << ", " << y_disp << "]" << " RIGHT" <<  "  Bearing = " << bearing << std::endl;
    }

    else if(x_disp < 0 && y_disp == 0 ){   // moved to the left (West) only
        /* x_comp = -1;
        y_comp = 0;
        w_comp = 0; */
        bearing = M_PI;
        //std::cout << "[" << x_disp << ", " << y_disp << "]" << " LEFT" <<  "  Bearing = " << bearing << std::endl;
    }
    else{   // get the bearing angle
        if(x_disp > 0 && y_disp > 0){   // Quadrant 1
            bearing = M_PI/2 - angle;
            std::cout << "[" << x_disp << ", " << y_disp << "]" << " Quadrant I" <<  std::endl;
        }
        if(x_disp < 0 && y_disp > 0){   // Quadrant II
            bearing = M_PI/2 - angle;
            std::cout << "[" << x_disp << ", " << y_disp << "]" << " Quadrant II" <<  std::endl;
        }
        if(x_disp < 0 && y_disp < 0){   // Quadrant III
            bearing = 3*M_PI/2 - angle;
            std::cout << "[" << x_disp << ", " << y_disp << "]" << " Quadrant III" <<  std::endl;
        }
        if(x_disp > 0 && y_disp < 0){   // Quadrant IV
            bearing = 3*M_PI/2 - angle;
            std::cout << "[" << x_disp << ", " << y_disp << "]" << " Quadrant IV" <<  std::endl;
        }
    }

    if(x_disp || y_disp){   // If this axes were moved then do the calculations. Just in case only rotation was activated
        x_comp = cos(bearing);
        y_comp = sin(bearing);
    }
}