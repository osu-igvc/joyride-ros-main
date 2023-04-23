
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

namespace joyride_control
{

class VelocityPreprocessor : public rclcpp::Node
{
public:
    explicit VelocityPreprocessor(const rclcpp::NodeOptions & options);

    void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &stat);


private:

    // Setup
    void initializeParameters();
    void initializeROS();

    // Main
    void publishAckermann();
    void updateAckermann();
    double firstOrderLowpass(double new_input, double old_output, double alpha);

    // ROS Callbacks
    void newCMDVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void newOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // ROS Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // ROS Publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_pub_;
    rclcpp::TimerBase::SharedPtr pub_ackermann_timer_;

    // Variables
    double current_speed_;
    double current_steering_angle_;

    double target_speed_;
    double target_steering_angle_;

    double commanded_speed_;
    double commanded_steering_angle_;
    double commanded_steering_velocity_;
    double previous_steering_velocity_;



    // Parameters
    double pub_ackermann_rate_;
    double max_steering_angle_rad_;
    double max_steering_velocity_radps_;
    double max_steering_acceleration_radps2_;
    double max_speed_mps_;
    double max_acceleration_mps2_;
    double wheelbase_meters_;
    double steering_alpha_;
    double speed_alpha_;
    std::string odom_topic_;
    std::string cmd_vel_topic_;
    std::string ackermann_topic_;

    double ZERO_THRESHOLD = 0.05;
    double STEERING_CHANGE_THRESHOLD_RAD = 0.05;

}; // class VelocityPreprocessor

} // namespace joyride_control