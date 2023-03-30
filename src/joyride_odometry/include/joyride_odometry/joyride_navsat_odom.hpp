
#ifndef JOYRIDE_ODOMETRY__NAVSAT_ODOM_HPP_
#define JOYRIDE_ODOMETRY__NAVSAT_ODOM_HPP_

#include <memory>
#include <string>

#include "GeographicLib/LocalCartesian.hpp"
#include "Eigen/Dense"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace joyride_odometry
{
class NavSatOdom : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   */
  explicit NavSatOdom(const rclcpp::NodeOptions&);

  /**
   * @brief Destructor
   */
  ~NavSatOdom();

void cmdVelCallback();

private:
  void gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void fakeOdomTimerCallback();
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  geometry_msgs::msg::TransformStamped buildOdomTF(double x, double y, double z,
                                                   geometry_msgs::msg::Quaternion orientation, std::string frame_id,
                                                   std::string child_frame_id);

  nav_msgs::msg::Odometry buildOdomMsg(double x, double y, double z, geometry_msgs::msg::Quaternion orientation,
                                       std::string frame_id, std::string child_frame_id);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navSatSub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;

  // TImers
  rclcpp::TimerBase::SharedPtr fakeOdomTimer_;


  // Parameters
  std::string base_link_frame_id_;
  std::string world_frame_id_;
  std::string navsat_sub_topic_;
  std::string imu_sub_topic_;
  std::string odom_pub_topic_;

  bool broadcast_cartesian_tf_;
  bool broadcast_odom_;
  bool use_fake_odom_;

  double fake_pos_x;
  double fake_pos_y;
  double fake_yaw_;
  double last_cmd_linx_;
  double last_cmd_angz_;

  // State
  sensor_msgs::msg::NavSatFix initialLLA_fix_ = sensor_msgs::msg::NavSatFix();
  double previousYawAngleRadians_;

  // TF2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2_ros::TransformBroadcaster odom_broadcaster_;

  // LTM
  GeographicLib::LocalCartesian local_cartesian_plane_;
};

}  // namespace joyride_odometry

#endif  // JOYRIDE_ODOMETRY__NAVSAT_ODOM_HPP_