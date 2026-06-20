#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class HybridControllerNode : public rclcpp::Node {
public:
    HybridControllerNode() : Node("hybrid_controller_node") {
        // Vehicle Physical Geometry & Tuning Limits
        L_base_    = declare_parameter("L_base", 1.53);
        L_min_     = declare_parameter("L_min", 3.0);
        k_pure_    = declare_parameter("k_pure", 0.75);
        k_stanley_ = declare_parameter("k_stanley", 0.3);
        k_soft_    = declare_parameter("k_soft", 3.5);
        delta_max_ = declare_parameter("delta_max", 0.35);

        // Friction Limits & Acceleration Profiling
        mu_               = declare_parameter("mu", 0.45);
        g_                = declare_parameter("g", 9.81);
        v_max_            = declare_parameter("v_max", 1.0);
        v_min_            = declare_parameter("v_min", 0.5); 
        a_dec_max_        = declare_parameter("a_dec_max", 4.0);
        a_acc_max_        = declare_parameter("a_acc_max", 0.1); 
        lookahead_points_ = declare_parameter("lookahead_points", 250);
        Kp_v_             = declare_parameter("Kp_v", 1.0); 

        cmd_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10);

        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/target_path", 1,
            std::bind(&HybridControllerNode::pathCallback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/slam/odom", 10,
            std::bind(&HybridControllerNode::odomCallback, this, std::placeholders::_1));

        speed_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&HybridControllerNode::speedCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Cleaned Hybrid Controller Node Online.");
    }

private:
    double L_base_, L_min_, k_pure_, k_stanley_, k_soft_, delta_max_;
    double mu_, g_, v_max_, v_min_, a_dec_max_, a_acc_max_, Kp_v_;
    int    lookahead_points_;

    std::mutex mtx_;
    std::vector<double> path_x_, path_y_, path_kappa_;
    bool has_path_ = false;

    double last_delta_ = 0.0;
    int    last_idx_   = 0;       
    rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
    double clean_vx_ = 0.0;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr speed_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_pub_;

    void speedCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        clean_vx_ = msg->twist.twist.linear.x;
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        int N = msg->poses.size();
        if (N < 3) return;

        path_x_.clear(); path_y_.clear(); path_kappa_.clear();
        path_kappa_.resize(N, 0.0);

        for (int i = 0; i < N; ++i) {
            path_x_.push_back(msg->poses[i].pose.position.x);
            path_y_.push_back(msg->poses[i].pose.position.y);
        }

        // Compute local trajectory curvature profiles
        for (int i = 1; i < N - 1; ++i) {
            double ax = path_x_[i-1], ay = path_y_[i-1];
            double bx = path_x_[i],   by = path_y_[i];
            double cx = path_x_[i+1], cy = path_y_[i+1];

            double ab = std::hypot(bx - ax, by - ay);
            double bc = std::hypot(cx - bx, cy - by);
            double ca = std::hypot(ax - cx, ay - cy);

            double cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
            double denom = std::max(ab * bc * ca, 1e-9);
            path_kappa_[i] = 2.0 * cross / denom;
        }
        path_kappa_[0]   = path_kappa_[1];
        path_kappa_[N-1] = path_kappa_[N-2];

        last_idx_ = 0;
        has_path_ = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        
        if (!has_path_ || path_x_.empty()) return;

        rclcpp::Time current_time = msg->header.stamp;
        if (last_time_.nanoseconds() == 0) { last_time_ = current_time; return; }
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        
        if (dt <= 0.0 || dt > 0.5) return;

        double est_x  = msg->pose.pose.position.x;
        double est_y  = msg->pose.pose.position.y;
        double est_vx = clean_vx_;

        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, est_psi;
        m.getRPY(roll, pitch, est_psi);

        int N = path_x_.size();
        if (last_idx_ >= N) last_idx_ = 0;

        // 1. Sliding Window Nearest Neighbor Search
        const int SEARCH_WINDOW = 60;
        int search_start = std::max(0, last_idx_ - 5);       
        int search_end   = std::min(N - 1, last_idx_ + SEARCH_WINDOW);

        double min_dist_sq_rear = 1e9;
        int idx_rear = last_idx_;
        for (int i = search_start; i <= search_end; ++i) {
            double dist_sq = std::pow(path_x_[i] - est_x, 2) + std::pow(path_y_[i] - est_y, 2);
            if (dist_sq < min_dist_sq_rear) { min_dist_sq_rear = dist_sq; idx_rear = i; }
        }
        last_idx_ = idx_rear;  

        // 2. Pure Pursuit Core Logic (Handles high-speed lookahead guidance)
        double L_ld = std::max(L_min_, k_pure_ * std::max(0.0, est_vx));
        double arc = 0.0;
        int idx_ld = idx_rear;
        while (idx_ld < N - 1) {
            double ds = std::hypot(path_x_[idx_ld + 1] - path_x_[idx_ld],
                                   path_y_[idx_ld + 1] - path_y_[idx_ld]);
            if (arc + ds >= L_ld) break;
            arc += ds;
            idx_ld++;
        }

        double alpha = std::atan2(path_y_[idx_ld] - est_y, path_x_[idx_ld] - est_x) - est_psi;
        alpha = std::atan2(std::sin(alpha), std::cos(alpha));
        double delta_pp = std::atan2(2.0 * L_base_ * std::sin(alpha), L_ld);

        // 3. Stanley Core Logic (Eliminates localized track centerline errors)
        double xf = est_x + L_base_ * std::cos(est_psi);
        double yf = est_y + L_base_ * std::sin(est_psi);
        
        int idx_front = idx_rear;
        double min_dist_sq_front = 1e9;
        int front_search_end = std::min(N - 1, idx_rear + SEARCH_WINDOW);
        for (int i = idx_rear; i <= front_search_end; ++i) {
            double dist_sq = std::pow(path_x_[i] - xf, 2) + std::pow(path_y_[i] - yf, 2);
            if (dist_sq < min_dist_sq_front) { min_dist_sq_front = dist_sq; idx_front = i; }
        }

        double sum_x = 0.0, sum_y = 0.0;
        int pts_to_average = std::min(4, N - 1 - idx_front);
        for(int i = 0; i < pts_to_average; ++i) {
            double seg_heading = std::atan2(path_y_[idx_front + i + 1] - path_y_[idx_front + i], 
                                            path_x_[idx_front + i + 1] - path_x_[idx_front + i]);
            sum_x += std::cos(seg_heading);
            sum_y += std::sin(seg_heading);
        }
        
        double path_heading = (pts_to_average > 0) ? std::atan2(sum_y, sum_x) : est_psi;
        double psi_e = std::atan2(std::sin(path_heading - est_psi), std::cos(path_heading - est_psi));
        double ef = -(path_x_[idx_front] - xf) * std::sin(path_heading) + (path_y_[idx_front] - yf) * std::cos(path_heading);
        ef = std::clamp(ef, -0.6, 0.6);
        
        double delta_stanley = psi_e + std::atan2(k_stanley_ * ef, k_soft_ + std::max(0.0, est_vx));

        // 4. Clean Control Fusion & Dynamic Steering Rate Limit
        double raw_delta = (0.5 * delta_stanley) + (0.5 * delta_pp);
        raw_delta = std::clamp(raw_delta, -delta_max_, delta_max_);

        // Low-pass filter to respect physical steering actuator limits
        double alpha_lp = std::clamp(dt / (0.05 + dt), 0.0, 1.0);
        double delta = alpha_lp * raw_delta + (1.0 - alpha_lp) * last_delta_;
        last_delta_  = delta;

        // 5. Clean Velocity Profiler (Forward Curvature Lookahead & Backward Deceleration Pass)
        int end_idx = std::min(idx_front + lookahead_points_, N - 1);
        int seg_len = end_idx - idx_front + 1;
        std::vector<double> v_limits(seg_len, v_max_);

        // Forward Pass: Compute max safe cornering speed based on lateral friction limits ($v = \sqrt{\mu g / \kappa}$)
        for (int i = idx_front; i <= end_idx; ++i) {
            double k = std::abs(path_kappa_[i]);
            v_limits[i - idx_front] = (k > 0.01) ? std::min(v_max_, std::sqrt((mu_ * g_) / k)) : v_max_;
        }

        // Backward Pass: Propagate deceleration requirements backward through time
        for (int i = end_idx - 1; i >= idx_front; --i) {
            double ds = std::max(std::hypot(path_x_[i+1] - path_x_[i], path_y_[i+1] - path_y_[i]), 0.01);
            double v_brake = std::sqrt(std::pow(v_limits[i - idx_front + 1], 2) + 2.0 * a_dec_max_ * ds);
            v_limits[i - idx_front] = std::min(v_limits[i - idx_front], v_brake);
        }

        double v_target = std::max(v_limits[0], v_min_);
        double v_err = v_target - est_vx;
        double acc = std::clamp(Kp_v_ * v_err, -a_dec_max_, a_acc_max_);

        if (std::isnan(delta) || std::isnan(v_target) || std::isinf(delta)) return;

        // 6. Output Command Formulation
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp    = current_time;
        drive_msg.header.frame_id = "base_footprint";
        drive_msg.drive.steering_angle = delta;
        drive_msg.drive.speed          = v_target;
        drive_msg.drive.acceleration   = acc;
        cmd_pub_->publish(drive_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HybridControllerNode>());
    rclcpp::shutdown();
    return 0;
}