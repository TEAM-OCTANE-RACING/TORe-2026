#include <memory>
#include <chrono>
#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <eigen3/Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

class StateEstimatorEKF : public rclcpp::Node {
public:
    StateEstimatorEKF() : Node("state_estimator") {
        // --- EKF STATE & MATRICES ---
        // State: [vx, vy, yaw_rate]
        x_hat_ = Eigen::Vector3d::Zero();
        
        // Covariance Matrix (Uncertainty)
        P_ = Eigen::Matrix3d::Identity() * 0.1;
        
        // Process Noise (Trust in the IMU prediction)
        Q_ = Eigen::Matrix3d::Identity() * 0.05;
        
        // Measurement Noise (Trust in the wheel speeds)
        R_meas_ = 0.1; 
        
        y_raw_ = Eigen::Vector3d::Zero();
        gx_ = 0.0; gy_ = 0.0; gyaw_ = 0.0;
        latest_vx_meas_ = 0.0;
        has_meas_ = false;
        
        this->declare_parameter("wheel_radius", 0.2286);
        this->declare_parameter("gear_ratio", 1.0);

        last_time_ = this->now();

        reset_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/reset_simulation", 10, [this](std_msgs::msg::Empty::SharedPtr) {
                gx_ = 0.0; gy_ = 0.0; gyaw_ = 0.0;
                x_hat_.setZero();
                P_ = Eigen::Matrix3d::Identity() * 0.1;
                RCLCPP_INFO(this->get_logger(), "EKF: State and Covariance Reset.");
            });

        // IMU Subscription (Prediction Input)
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                // FIX: Low-Pass Filter (LPF)
                // We keep 80% of the old value and only blend in 20% of the new raw data.
                // This completely destroys the high-frequency Gazebo vibration!
                y_raw_(0) = (0.8 * y_raw_(0)) + (0.2 * msg->linear_acceleration.x);
                y_raw_(1) = (0.8 * y_raw_(1)) + (0.2 * msg->linear_acceleration.y);
                y_raw_(2) = (0.8 * y_raw_(2)) + (0.2 * msg->angular_velocity.z); 
            });

        // Joint States Subscription (Measurement Update)
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(), std::bind(&StateEstimatorEKF::jointCallback, this, _1));

        state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Run EKF math at 100Hz (10ms)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&StateEstimatorEKF::runEKF, this));
        
        RCLCPP_INFO(this->get_logger(), "True EKF State Estimator Online.");
    }

private:
    Eigen::Vector3d x_hat_, y_raw_;
    Eigen::Matrix3d P_, Q_;
    double R_meas_;
    double gx_, gy_, gyaw_;
    double latest_vx_meas_;
    bool has_meas_;
    rclcpp::Time last_time_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        double r_wheel = this->get_parameter("wheel_radius").as_double();
        double gear_ratio = this->get_parameter("gear_ratio").as_double(); 
        double v_rl = 0.0, v_rr = 0.0;
        int wheels_found = 0; // FIX: Track how many wheels we actually found
        
        for (size_t i = 0; i < msg->name.size(); i++) {
            if (msg->name[i] == "wheel_rl_joint" || msg->name[i] == "left_rear_axle") {
                v_rl = (msg->velocity[i] / gear_ratio) * r_wheel; 
                wheels_found++;
            } else if (msg->name[i] == "wheel_rr_joint" || msg->name[i] == "right_rear_axle") {
                v_rr = (msg->velocity[i] / gear_ratio) * r_wheel; 
                wheels_found++;
            }
        }
        
        if (wheels_found > 0) {
            latest_vx_meas_ = (v_rl + v_rr) / wheels_found; // FIX: Safe average
            has_meas_ = true;
        }
    }

   void runEKF() {
        double dt = (this->now() - last_time_).seconds();
        if (dt <= 0 || dt > 0.1) { last_time_ = this->now(); return; }
        last_time_ = this->now();

        double ax = y_raw_(0);
        double ay = y_raw_(1);
        double r  = y_raw_(2);

        // ==========================================
        // STEP 1: EKF PREDICTION (Time Update)
        // ==========================================
        
        // Jacobian Matrix (F) for the dynamic model
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0, 1) = r * dt;
        F(1, 0) = -r * dt;

        // Predict State
        x_hat_(0) += (ax + r * x_hat_(1)) * dt; 
        x_hat_(1) += (ay - r * x_hat_(0)) * dt;
        x_hat_(2) = r; // Direct feedthrough for yaw rate

        // Predict Covariance
        P_ = F * P_ * F.transpose() + Q_;

        // ==========================================
        // STEP 2: EKF UPDATE (Measurement Correction)
        // ==========================================
        
        // Only update if we received new wheel speed data
        if (has_meas_) {
            // FIX: Apply the Non-Holonomic Constraint (NHC)
            // H Matrix now maps state [vx, vy, r] to measurements [vx, vy]
            Eigen::Matrix<double, 2, 3> H;
            H << 1.0, 0.0, 0.0,   // We measure forward speed (vx)
                 0.0, 1.0, 0.0;   // We "measure" lateral speed (vy)
            
            // The Innovation (Error between IMU guess and Reality)
            // z(0) is true wheel speed, z(1) is 0.0 (car doesn't slide sideways)
            Eigen::Vector2d z(latest_vx_meas_, 0.0);
            Eigen::Vector2d y = z - H * x_hat_; 
            
            // Measurement Noise Matrix (R)
            Eigen::Matrix2d R_mat;
            R_mat << R_meas_, 0.0,
                     0.0,     0.01; // 0.1 gives the car a tiny bit of room to naturally slip
            
            // Innovation Covariance (S)
            Eigen::Matrix2d S = H * P_ * H.transpose() + R_mat;
            
            // Kalman Gain (K)
            Eigen::Matrix<double, 3, 2> K = P_ * H.transpose() * S.inverse();
            
            // Update State with Kalman Gain
            x_hat_ = x_hat_ + K * y;
            
            // Update Covariance
            P_ = (Eigen::Matrix3d::Identity() - K * H) * P_;
            
            has_meas_ = false; // Reset for next reading
        }

        // ==========================================
        // STEP 3: GLOBAL INTEGRATION & PUBLISH
        // ==========================================
        
        gyaw_ += x_hat_(2) * dt;
        gyaw_ = std::atan2(std::sin(gyaw_), std::cos(gyaw_));

        gx_ += (x_hat_(0) * std::cos(gyaw_) - x_hat_(1) * std::sin(gyaw_)) * dt;
        gy_ += (x_hat_(0) * std::sin(gyaw_) + x_hat_(1) * std::cos(gyaw_)) * dt;

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        
        odom.pose.pose.position.x = gx_;
        odom.pose.pose.position.y = gy_;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, gyaw_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = x_hat_(0);
        odom.twist.twist.linear.y = x_hat_(1);
        odom.twist.twist.angular.z = x_hat_(2);

        state_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_footprint"; // EUFS usually anchors the 3D mesh to base_footprint
        
        t.transform.translation.x = gx_;
        t.transform.translation.y = gy_;
        t.transform.translation.z = 0.0;
        
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateEstimatorEKF>());
    rclcpp::shutdown();
    return 0;
}