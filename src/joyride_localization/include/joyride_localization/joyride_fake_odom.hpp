

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"


#include "rclcpp/time.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"


namespace joyride_odometry
{
    
class JoyrideFakeOdom : public rclcpp::Node
{

public:
    explicit JoyrideFakeOdom(const rclcpp::NodeOptions & options);
    ~JoyrideFakeOdom();

private:

    void initializeParameters();
    void initializeROS();

    // ROS Callbacks
    void newCmdAckCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg);
    void publishOdomCallback();


    void computeOdom(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg);


    // ROS Subscribers
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr cmd_ack_sub_;

    // ROS Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr pub_odom_timer_;
    

    // Variables
    double position_x_;
    double position_y_;
    double position_z_;
    double yaw_;

    double velocity_x_;
    double velocity_y_;
    double velocity_z_;
    double yaw_rate_;


    // Parameters
    double linear_lag_;
    double angular_lag_;
    double wheelbase_;

    double pub_odom_rate_;
    std::string odom_topic_;
    std::string cmd_ack_topic_;
    std::string odom_frame_;
    std::string base_frame_;




};
}