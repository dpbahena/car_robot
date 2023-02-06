#include <functional>
#include <thread>
#include <inttypes.h>
#include <memory>
#include "car_interfaces/action/carmove.hpp"
#include "car_interfaces/msg/encoder.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"


/* ---------------------  TESTING ACTION --------------------------------->
*
*   ros2 action send_goal /move_the_car car_interfaces/action/Carmove "{distance: 4}"    // old command
*
*   ros2 action send_goal /move_the_car car_interfaces/action/Carmove "{distance: 4}" -f   // old command
*
*   ros2 action send_goal /move_the_car car_interfaces/action/Carmove "{distance: 2 , twist:{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0}}}" -f
*/

using namespace std::chrono_literals;
using Carmove = car_interfaces::action::Carmove;
using GoalHandleCarmove = rclcpp_action::ServerGoalHandle<Carmove>;

class CarActionServer : public rclcpp::Node{
public:
    explicit CarActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("caractionserver_node", options){
        using namespace std::placeholders;

        this->car_action_server_ = rclcpp_action::create_server<Carmove>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "move_the_car",
            std::bind(&CarActionServer::handle_goal,this, _1, _2),
            std::bind(&CarActionServer::handle_cancel, this, _1),
            std::bind(&CarActionServer::handle_accepted, this, _1)
        );

        /* Init Twist publisher */
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("vel_cmds", 1);

        /* Init subscriber to Pose topic */
        pose_sub_ = this->create_subscription<std_msgs::msg::Float32>("udpated_distance", 1, std::bind(&CarActionServer::distance_callback, this, _1));

        /* Init client-server services */
        encoder_client_ = this->create_client<std_srvs::srv::SetBool>("reset_encoder");
        
    }

    void queue_async_request(){
        // Service call from client
        while( !encoder_client_->wait_for_service(1s)){
            if(!rclcpp::ok()){
                RCLCPP_INFO(this->get_logger(),"client interrupted while waiting for service to appear.");
                return;
            }
            RCLCPP_INFO(this->get_logger(),"waiting for service to appear ...");
        }
        auto request_reset = std::make_shared<std_srvs::srv::SetBool::Request>();
        request_reset->data = true;  // true means : reset the encoder count
        rclcpp::Rate rate(0.7);
        auto response_received_callback = [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
            {
                auto result = future.get();
                result->message = "Encoders have been reset!";
                result->success = true;
                RCLCPP_INFO(this->get_logger()," %s", result->message.c_str());
            };
        rate.sleep();
        auto future_result = encoder_client_->async_send_request(request_reset, response_received_callback);
    }

private:
    float posed_distance{};
    /* action server - issues an action command to be executed */
    rclcpp_action::Server<Carmove>::SharedPtr car_action_server_;

    /* publish the velocity vector */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;

    /* subcribes to the pose topic to check for the updated distance */
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pose_sub_;

    /* client --- make server's request to reset the encoders value to 0 */
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr encoder_client_;

    void distance_callback(const std_msgs::msg::Float32::ConstSharedPtr msg){
        /* Get updated distance from  topic 'updated_distance' */
        posed_distance = msg->data;  
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Carmove::Goal> goal){
        RCLCPP_INFO(this->get_logger(),"Received goal request: Distance %f", goal->distance);
        (void)uuid;
        /* reject distance that are over 10 feet */
        if(goal->distance > 10){
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCarmove> goal_handle){
        RCLCPP_INFO(this->get_logger(),"Received request to cancel the goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleCarmove> goal_handle){
        using namespace std::placeholders;
        /* This returns quckly to avoid blocking the executor: ->>> so spin up  a new thread */
        std::thread{std::bind(&CarActionServer::execute, this, _1), goal_handle}.detach();

    } 

    void execute(const std::shared_ptr<GoalHandleCarmove> goal_handle){
        RCLCPP_INFO(this->get_logger(),"Executing goal");
        rclcpp::Rate loop_rate(50);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Carmove::Feedback>();
        auto &latest = feedback->updated_distance;
        latest = 0.0;
        auto result = std::make_shared<Carmove::Result>();
        auto velocity = geometry_msgs::msg::Twist();
        
        // init velocity vector

        velocity.linear.x = goal->twist.linear.x;
        velocity.linear.y = goal->twist.linear.y;
        velocity.angular.z = goal->twist.angular.z;
        
        while(latest <=goal->distance && rclcpp::ok()){
            // Check if there is a cancel request
            if( goal_handle->is_canceling() ){
                result->distance = latest;   // update final distance to the latest distance the car moved
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(),"Goal canceled!");
                return;
            }
            // update feedback
            latest = posed_distance;  // latest = distance_update (from a pose subscriber)
            
            velocity_pub_->publish(velocity);
            goal_handle->publish_feedback(feedback);  // publish the range which is defined as the so far distance range
            loop_rate.sleep();
        }

    }


};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarActionServer>());
    rclcpp::shutdown();

    return 0;
}