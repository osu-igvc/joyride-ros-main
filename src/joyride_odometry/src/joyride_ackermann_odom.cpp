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

#include "joyride_odometry/joyride_ackermann_odom.hpp"

using namespace std::chrono_literals;

// Parameters
// - Steering wheel angle
// - Speed feedback
// - Publish topic

OdometryNode::OdometryNode(): Node("joyride_odometry_publisher"){

    this->declare_parameter("steering_angle_fb_topic", "/feedback/steer_angle");
    this->declare_parameter("wheelspeed_fb_topic", "/feedback/wheelspeed");
    this->declare_parameter("output_topic", "/odom/ackermann");

    this->angle_fb_topic = this->get_parameter("steering_angle_fb_topic").get_parameter_value().get<std::string>();
    this->wheelspeed_fb_topic = this->get_parameter("wheelspeed_fb_topic").get_parameter_value().get<std::string>();
    this->output_topic = this->get_parameter("output_topic").get_parameter_value().get<std::string>();


    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    poseTimer_ = this->create_wall_timer(10ms, std::bind(&OdometryNode::poseTimerCB, this));

    velocitySub_ = this->create_subscription<std_msgs::msg::Float32>(this->wheelspeed_fb_topic, 10,
                    std::bind(&OdometryNode::velocityCB, this, std::placeholders::_1));

    steeringSub_ = this->create_subscription<std_msgs::msg::Float32>(this->angle_fb_topic, 10,
                    std::bind(&OdometryNode::steeringCB, this, std::placeholders::_1));

    odomPub_ = this->create_publisher<nav_msgs::msg::Odometry>(this->output_topic,10);
}

void OdometryNode::poseTimerCB(){
    rclcpp::Time T; 
    tf2::Quaternion q;
    nav_msgs::msg::Odometry o;
    geometry_msgs::msg::TransformStamped t;

    T = this->get_clock()->now();
    this->time[1] = T.seconds();

    float omega = v*tan(this->phi)/this->L;

    this->x     += this->v*cos(this->theta)*(this->time[1]-this->time[0]);
    this->y     += this->v*sin(this->theta)*(this->time[1]-this->time[0]);
    this->theta += omega*(this->time[1]-this->time[0]);

    q.setRPY(0, 0, this->theta);

    this->time[0] = this->time[1];

    t.header.stamp = T;
    t.header.frame_id = "/odom";
    t.child_frame_id = "/base_link";

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    t.transform.translation.x = this->x;
    t.transform.translation.y = this->y;
    t.transform.translation.z = this->z;

    o.header.stamp = T;
    o.header.frame_id = "/odom";
    o.child_frame_id = "/base_link";

    o.pose.pose.position.x = this->x;
    o.pose.pose.position.y = this->y;
    o.pose.pose.position.z = this->z;
    o.pose.pose.orientation.x = q.x();
    o.pose.pose.orientation.y = q.y();
    o.pose.pose.orientation.z = q.z();
    o.pose.pose.orientation.w = q.w();

    o.twist.twist.linear.x = this->v;
    o.twist.twist.linear.y = 0;
    o.twist.twist.linear.z = 0;
    o.twist.twist.angular.x = 0;
    o.twist.twist.angular.y = 0;
    o.twist.twist.angular.z = omega;

    tf_broadcaster_->sendTransform(t);
    odomPub_->publish(o);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryNode>());
  rclcpp::shutdown();
  return 0;
}