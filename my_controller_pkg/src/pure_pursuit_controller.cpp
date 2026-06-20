#include <memory>
#include <cmath>
#include <algorithm>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/msg/marker.hpp>

using std::placeholders::_1;

class HybridControllerNode : public rclcpp::Node {
public:
    HybridControllerNode() : Node("pure_pursuit_node") {
        // Lateral Control (Steering) Parameters
        this->declare_parameter("L_base", 1.53);      
        this->declare_parameter("L_min", 2.0);        
        this->declare_parameter("k_pure", 0.5);          

        // Longitudinal Control (Acceleration/Braking) Parameters
        this->declare_parameter("max_speed_limit", 3.0); // Absolute max speed (m/s)
        this->declare_parameter("max_accel", 1.0);        // Max positive acceleration (m/s^2)
        this->declare_parameter("max_decel", 4.0);        // Max braking capability (m/s^2)

        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/slam/odom", 10, std::bind(&HybridControllerNode::odomCallback, this, _1));
        
        speed_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth/odom", 10, std::bind(&HybridControllerNode::speedCallback, this, _1));    
        
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/target_path", 10, std::bind(&HybridControllerNode::pathCallback, this, _1));

        // NEW: Separate Speed Signal Subscriber
        speed_profile_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/target_speeds", 10, std::bind(&HybridControllerNode::speedProfileCallback, this, _1));
            
        // Publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10);
        vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lookahead_marker", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&HybridControllerNode::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "🏁 Hybrid Controller Initialized with Separate Speed Signal!");
    }

private:
    double x_ = 0, y_ = 0, psi_ = 0, vx_ = 0;
    nav_msgs::msg::Path path_;
    std::vector<double> speed_profile_; // Stores data from /target_speeds
    bool has_odom_ = false, has_path_ = false;
    double last_steering_ = 0.0;
    size_t last_closest_idx_ = 0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr speed_sub_; 
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr speed_profile_sub_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        x_ = msg->pose.pose.position.x; 
        y_ = msg->pose.pose.position.y; 
        
        tf2::Quaternion q(
            msg->pose.pose.orientation.x, 
            msg->pose.pose.orientation.y, 
            msg->pose.pose.orientation.z, 
            msg->pose.pose.orientation.w
        );
        double r, p, yaw; 
        tf2::Matrix3x3(q).getRPY(r, p, yaw); 
        psi_ = yaw; 
        has_odom_ = true;
    }

    void speedCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        vx_ = msg->twist.twist.linear.x; 
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) return;
        path_ = *msg; 
        has_path_ = true; 
        last_closest_idx_ = 0;
    }

    void speedProfileCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        speed_profile_ = msg->data;
    }

    void controlLoop() {
        if (!has_odom_ || !has_path_) return;
        size_t N = path_.poses.size(); 
        if (N < 2) return;

        // 1. Find the closest point to the car
        double min_d = 1e9;
        for (size_t i = 0; i < N; ++i) {
            double d = std::hypot(path_.poses[i].pose.position.x - x_, path_.poses[i].pose.position.y - y_);
            if (d < min_d) { min_d = d; last_closest_idx_ = i; }
        }

        double L_base = get_parameter("L_base").as_double();
        double Ld = get_parameter("L_min").as_double() + get_parameter("k_pure").as_double() * std::abs(vx_);
        
        // 2. Find the Lookahead Point
        size_t idx_ld = last_closest_idx_;
        for (size_t i = last_closest_idx_; i < N; ++i) {
            double dx = path_.poses[i].pose.position.x - x_;
            double dy = path_.poses[i].pose.position.y - y_;
            if (std::hypot(dx, dy) >= Ld && (dx * std::cos(psi_) + dy * std::sin(psi_)) > 0.0) { 
                idx_ld = i; break; 
            }
            if (i == N - 1) idx_ld = N - 1;
        }
        
        double tx = path_.poses[idx_ld].pose.position.x;
        double ty = path_.poses[idx_ld].pose.position.y;
        
        // --- LATERAL CONTROL (PURE PURSUIT) ---
        double dx = tx - x_; 
        double dy = ty - y_;
        double ly = -dx * std::sin(psi_) + dy * std::cos(psi_);
        
        double Ld_sq = dx * dx + dy * dy;
        if (Ld_sq < 0.001) Ld_sq = 0.001; 
        
        double delta = std::atan2(2.0 * L_base * ly, Ld_sq);
        
        // Visualize lookahead target in RViz
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map"; marker.header.stamp = this->now();
        marker.ns = "lookahead"; marker.id = 0; marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = tx; marker.pose.position.y = ty; marker.pose.position.z = 0.5;
        marker.scale.x = 0.4; marker.scale.y = 0.4; marker.scale.z = 0.4;
        marker.color.a = 1.0; marker.color.g = 1.0; 
        vis_pub_->publish(marker);

        // Populate Drive Message
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->now();
        
        double raw_steering = std::clamp(delta, -0.5, 0.5);
        double smoothed_steering = (0.60 * raw_steering) + (0.40 * last_steering_);
        last_steering_ = smoothed_steering;
        drive_msg.drive.steering_angle = smoothed_steering;
        
        // --- LONGITUDINAL CONTROL (PREDICTIVE BRAKING) ---
        double max_speed_limit = get_parameter("max_speed_limit").as_double();
        double deceleration_limit = get_parameter("max_decel").as_double(); 
        double target_velocity = max_speed_limit; 
        
        int velocity_scan_limit = std::min(static_cast<int>(last_closest_idx_) + 80, static_cast<int>(N) - 1);
        
        for (int i = last_closest_idx_; i <= velocity_scan_limit; ++i) {
            // FIXED: Read from separate speed profile vector instead of Z coordinate
            double node_speed = max_speed_limit;
            if (i < (int)speed_profile_.size()) {
                node_speed = speed_profile_[i];
            }
            
            double dist_to_node = std::hypot(path_.poses[i].pose.position.x - x_, 
                                             path_.poses[i].pose.position.y - y_);
            
            // v_i = sqrt(v_f^2 + 2ad)
            double required_current_speed = std::sqrt(std::max(0.0, (node_speed * node_speed) + (2.0 * deceleration_limit * dist_to_node)));
            
            if (required_current_speed < target_velocity) {
                target_velocity = required_current_speed;
            }
        }

        // Final safety bounds (lower bound reduced to 1.5 for sharper hairpins)
        target_velocity = std::clamp(target_velocity, 1.5, max_speed_limit);

        drive_msg.drive.speed = target_velocity;
        
        if (target_velocity < vx_) {
            drive_msg.drive.acceleration = -deceleration_limit; 
        } else {
            drive_msg.drive.acceleration = get_parameter("max_accel").as_double(); 
        }
        
        drive_pub_->publish(drive_msg);
    }
};

int main(int argc, char **argv) { 
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<HybridControllerNode>()); 
    rclcpp::shutdown(); 
    return 0; 
}