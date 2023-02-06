#include <inttypes.h>
#include <memory>

#include <functional>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "car_interfaces/action/carmove.hpp"


/*  --------------------- COMMAND LINE --------------------
*
*            ros2 run car_control actionclient 2 1 0 0
*
*/



class CarActionClient : public rclcpp::Node{
public:
    using Carmove = car_interfaces::action::Carmove;
    using GoalHandleCarmove = rclcpp_action::ClientGoalHandle<Carmove>;

    explicit CarActionClient(float d, float lx, float ly, float az, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
    bool is_goal_done();

private:
    rclcpp_action::Client<Carmove>::SharedPtr car_action_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    float d, lx,ly,az;
    

    bool goal_done_;
    void send_goal();
    void goal_response_callback(std::shared_future<GoalHandleCarmove::SharedPtr> future);
    void feedback_callback(GoalHandleCarmove::SharedPtr, const std::shared_ptr<const Carmove::Feedback> feedback);
    void result_callback(const GoalHandleCarmove::WrappedResult &result);


    
};

int main(int argc, char** argv){
    
    rclcpp::init(argc, argv);
    float distance;
    float linear_x, linear_y, angular_z; 
    if (argc == 5){
        distance = atof(argv[1]);
        linear_x = atof(argv[2]);
        linear_y = atof(argv[3]);
        angular_z = atof(argv[4]);
    }else{
        std::cout << "Correct usage is ./actionclient <distance> <linearVelocity-x> <linearVelocity-y> <angularVelocity-z> /n" <<
                     "distance(1-10 ft) linear velocity min: .23 ft/sec max:2.0 ft/s /n" << 
                     "angular velocity min .23 radians/sec max: 3 radians/sec" << std::endl;
        return 0;
    }
    auto action_client = std::make_shared<CarActionClient>(distance, linear_x, linear_y, angular_z);
    while(!action_client->is_goal_done()){
        rclcpp::spin_some(action_client);
    }
    rclcpp::shutdown();

    return 0;

}

CarActionClient::CarActionClient(float d, float lx, float ly, float az, const rclcpp::NodeOptions &options  ) : Node("car_action_client", options), goal_done_(false){
    this->car_action_client_ = rclcpp_action::create_client<Carmove>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "move_the_car");
    this->d = d;
    this->lx = lx;
    this->ly = ly;
    this->az = az;

    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&CarActionClient::send_goal, this));

}

bool CarActionClient::is_goal_done(){
    return this->goal_done_;
}

void CarActionClient::send_goal(){
    using namespace std::placeholders;
    timer_->cancel();
    goal_done_ = false;
    if(!car_action_client_){
        RCLCPP_ERROR(get_logger(),"Action client not initialize");
    }
    if(!car_action_client_->wait_for_action_server(std::chrono::seconds(10))){
        RCLCPP_INFO(get_logger(),"Action server not available after waiting");
        this->goal_done_ = true;
        return;
    }
    
    auto goal_msg = Carmove::Goal();
    goal_msg.distance = d;
    goal_msg.twist.linear.x = lx;
    goal_msg.twist.linear.y = ly;
    goal_msg.twist.angular.z = az;

    RCLCPP_INFO(get_logger(),"Sending goal ...");

    auto send_goal_options = rclcpp_action::Client<Carmove>::SendGoalOptions();
    // send_goal_options.goal_response_callback = std::bind(&CarActionClient::goal_response_callback, this, _1 );
    send_goal_options.goal_response_callback = std::bind(&CarActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&CarActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&CarActionClient::result_callback, this, _1);
    auto goal_handle_future = this->car_action_client_->async_send_goal(goal_msg, send_goal_options);

    
    
}

//void CarActionClient::goal_response_callback(std::shared_future<GoalHandleCarmove::SharedPtr> future){
void CarActionClient::goal_response_callback(const GoalHandleCarmove::ConstSharedPtr & goal_handle){
    //auto goal_handle = future.get();
    if(!goal_handle){
        RCLCPP_ERROR(get_logger(),"Goal was rejected by server");
    }else{
        RCLCPP_INFO(get_logger(),"Goal accepted by server, waiting for result");
    }
}

void CarActionClient::feedback_callback(GoalHandleCarmove::SharedPtr, const std::shared_ptr<const Carmove::Feedback> feedback){
    RCLCPP_INFO(get_logger(),"Distance reached so far is %f", feedback->updated_distance);
}

void CarActionClient::result_callback(const GoalHandleCarmove::WrappedResult &result){
    goal_done_ = true;
    switch (result.code){

        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(),"Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(get_logger(),"Goal was canceled");
            break;
    
        default:
            RCLCPP_ERROR(get_logger(),"Unknown result code");
            return;
    }
    RCLCPP_INFO(get_logger(),"Results received");
    
    RCLCPP_INFO(get_logger(),"The car has moved to a new location: %f", result.result->distance);

}