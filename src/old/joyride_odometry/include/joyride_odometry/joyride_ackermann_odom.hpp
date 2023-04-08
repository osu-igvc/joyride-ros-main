#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "joyride_interfaces/msg/eps_position_velocity_feedback.hpp"

using namespace std::chrono_literals;

class OdometryNode : public rclcpp::Node{
  public:
    OdometryNode();

  private:
    float x, y, v, theta, phi;
    const float L = 1.75; // Wheelbase in meters
    const float z = 0.2794;
    const float STEER_ANGLE_TO_WHEEL_ANGLE = 1.0/25.59;
    const float STEER_BIAS = 0.0;
    bool sim_time = false;
    bool publish_odom_tf_;
    rclcpp::Time latest_time;
    double time[2];

    std::string angle_fb_topic;
    std::string wheelspeed_fb_topic;
    std::string output_topic;
    std::string child_frame_id;
    std::string parent_frame_id;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocitySub_;
    rclcpp::Subscription<joyride_interfaces::msg::EPSPositionVelocityFeedback>::SharedPtr steeringSub_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr timeSub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
    
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::TransformBroadcaster odom_broadcaster_;

    rclcpp::TimerBase::SharedPtr poseTimer_;

    void velocityCB(const std::shared_ptr<std_msgs::msg::Float32> msg){this->v = msg->data;}

    void steeringCB(const std::shared_ptr<joyride_interfaces::msg::EPSPositionVelocityFeedback> msg){this->phi = msg->position_radians;}

    void poseTimerCB();

    void updateSimTimeCB(const std::shared_ptr<rosgraph_msgs::msg::Clock> clockMsg);
};