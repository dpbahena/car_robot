#ifndef __ROS2_DEFINITIONS_H_
#define __ROS2_DEFINITIONS_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/bool.hpp>



//#if ROS_DISTRO >= ROS_GALACTIC
    #include <vision_msgs/msg/classification.hpp>
//#else
//    #include <vision_msgs/msg/classification2_d.hpp>

//#endif

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/vision_info.hpp>
#include <geometry_msgs/msg/twist.hpp>


extern std::string __node_name_;
extern rclcpp::Clock::SharedPtr __global_clock_;

#define ROS_DEBUG(...)          RCUTILS_LOG_DEBUG_NAMED(__node_name_.c_str(), __VA_ARGS__)
#define ROS_ERROR(...)          RCUTILS_LOG_ERROR_NAMED(__node_name_.c_str(), __VA_ARGS__)
#define ROS_INFO(...)           RCUTILS_LOG_INFO_NAMED(__node_name_.c_str(), __VA_ARGS__)


#define ROS_CREATE_PUBLISHER_STATUS(msg, topic, queue, callback, ptr)	ptr = this->create_publisher<msg>(topic, queue); rclcpp::TimerBase::SharedPtr __timer_publisher_##ptr = this->create_wall_timer(std::chrono::milliseconds(500), \
														[&, this](){ static int __subscribers_##ptr=0; const size_t __subscription_count=ptr->get_subscription_count(); if(__subscribers_##ptr != __subscription_count) { if(__subscription_count > __subscribers_##ptr) callback(); __subscribers_##ptr=__subscription_count; }}) 
//template<class MessageType>
//using Publisher = std::shared_ptr<ros::Publisher<MessageType>>;

#define ROS_CREATE_NODE(name)   \
        rclcpp::init(argc, argv);   \
        auto node = rclcpp::Node::make_shared(name, "/" name); \
        __node_name_ = name;  \
        __global_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        

#endif