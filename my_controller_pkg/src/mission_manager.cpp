#include <rclcpp/rclcpp.hpp>
#include <eufs_msgs/srv/set_can_state.hpp>
#include <eufs_msgs/msg/can_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>

using namespace std::chrono_literals;

class MissionManager : public rclcpp::Node {
public:
    MissionManager() : Node("mission_manager") {
        this->declare_parameter("mission", "skidpad");

        client_reset_ = this->create_client<std_srvs::srv::Trigger>("/ros_can/reset");
        client_mission_ = this->create_client<eufs_msgs::srv::SetCanState>("/ros_can/set_mission");
        
        state_pub_ = this->create_publisher<eufs_msgs::msg::CanState>("/ros_can/state", 10);
        
        sub_state_ = this->create_subscription<eufs_msgs::msg::CanState>(
            "/ros_can/state", 10, std::bind(&MissionManager::state_cb, this, std::placeholders::_1));
            
        timer_ = this->create_wall_timer(1000ms, std::bind(&MissionManager::check_status, this));
        
        RCLCPP_INFO(this->get_logger(), "Universal Mission Manager Initialized.");
    }

private:
    void state_cb(const eufs_msgs::msg::CanState::SharedPtr msg) { 
        current_state_ = msg->as_state; 
    }

    void check_status() {
        if (!client_reset_->wait_for_service(1s)) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for EUFS services...");
            return;
        }

        if (current_state_ == 0) { 
            RCLCPP_INFO_ONCE(this->get_logger(), "Car is OFF. Requesting Reset...");
            auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
            client_reset_->async_send_request(req);
        } 
        else if (current_state_ == 1) { 
            std::string mission_str = this->get_parameter("mission").as_string();
            int ami_state = 4; // Default to Trackdrive

            if (mission_str == "acceleration") ami_state = 1;
            else if (mission_str == "skidpad") ami_state = 3;
            else if (mission_str == "autocross") ami_state = 2;
            else if (mission_str == "trackdrive") ami_state = 4;

            RCLCPP_INFO_ONCE(this->get_logger(), "Car is READY. Sending Service Call for %s...", mission_str.c_str());
            
            auto req = std::make_shared<eufs_msgs::srv::SetCanState::Request>();
            req->ami_state = ami_state; 
            req->as_state = 4;  // AS_DRIVING
            client_mission_->async_send_request(req);
        }
        else if (current_state_ == 2) {
            RCLCPP_INFO_ONCE(this->get_logger(), "SUCCESS: Car is in DRIVE mode.");
        }
    }

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_reset_;
    rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr client_mission_;
    rclcpp::Publisher<eufs_msgs::msg::CanState>::SharedPtr state_pub_;
    rclcpp::Subscription<eufs_msgs::msg::CanState>::SharedPtr sub_state_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint16_t current_state_ = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionManager>());
    rclcpp::shutdown();
    return 0;
}