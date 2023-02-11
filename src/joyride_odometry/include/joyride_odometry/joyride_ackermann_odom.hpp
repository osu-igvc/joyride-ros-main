#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class OdometryNode : public rclcpp::Node{
  public:
    OdometryNode();

  private:
    float x, y, v, theta, phi;
    float L = 1.75;
    float z = 0.2794;
    double time[2];

    std::string angle_fb_topic;
    std::string wheelspeed_fb_topic;
    std::string output_topic;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocitySub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steeringSub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr poseTimer_;

    void velocityCB(const std::shared_ptr<std_msgs::msg::Float32> msg){this->v = msg->data;}

    void steeringCB(const std::shared_ptr<std_msgs::msg::Float32> msg){this->phi = msg->data;}

    void poseTimerCB();
};