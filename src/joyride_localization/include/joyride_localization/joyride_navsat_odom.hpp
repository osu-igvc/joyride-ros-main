
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
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "tf2_ros/buffer.h"
#include "vectornav_msgs/msg/common_group.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "joyride_interfaces/srv/get_odom_origin_ll.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace joyride_odometry
{
class NavSatOdom : public rclcpp::Node
{
public:

  // Setup
  explicit NavSatOdom(const rclcpp::NodeOptions&);
  ~NavSatOdom();

  void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &stat);

private:

  void initializeParameters();
  void initializeROS();

  // Callbacks
  void gpsCommonCallback(const vectornav_msgs::msg::CommonGroup::SharedPtr msg);
  void updateTransform(const vectornav_msgs::msg::CommonGroup::SharedPtr msg);

  nav_msgs::msg::Odometry
  buildOdometryMessage(const geometry_msgs::msg::Point::SharedPtr position,
                            const geometry_msgs::msg::Quaternion::SharedPtr orientation,
                            const geometry_msgs::msg::Twist::SharedPtr velocity,
                            const std::string frame_id,
                            const std::string child_frame_id);

  geometry_msgs::msg::TransformStamped
  buildOdometryTransform(const geometry_msgs::msg::Point::SharedPtr position,
                              const geometry_msgs::msg::Quaternion::SharedPtr orientation,
                              const std::string frame_id,
                              const std::string child_frame_id);

  void publishOdometryCallback();

  // Helpers
  double llDistance(double lat1, double lon1, double lat2, double lon2);
  
  void getInitialLLCallback(const std::shared_ptr<joyride_interfaces::srv::GetOdomOriginLL::Request> request, std::shared_ptr<joyride_interfaces::srv::GetOdomOriginLL::Response> response);
  void resetOdomCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);


  // Last Measured State
  vectornav_msgs::msg::CommonGroup::SharedPtr initialLLA_fix_;
  geometry_msgs::msg::Point::SharedPtr last_position_;
  geometry_msgs::msg::Quaternion::SharedPtr last_orientation_;
  geometry_msgs::msg::Twist::SharedPtr last_velocity_;
  bool valid_fix_obtained_;

  // ROS
  rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr gpsCommonSub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
  rclcpp::TimerBase::SharedPtr publishOdometryTimer_;
  rclcpp::Service<joyride_interfaces::srv::GetOdomOriginLL>::SharedPtr getOdomOriginLLService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetOdomService_;

  // TF2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2_ros::TransformBroadcaster odom_broadcaster_;

  // LTM
  GeographicLib::LocalCartesian local_cartesian_plane_;
  double expected_lat_;
  double expected_lon_;
  double initial_ll_radius_;

  // Parameters
  double tf_frequency_;
  double tf_period_;

  std::string base_link_frame_id_;
  std::string world_frame_id_;
  std::string odom_pub_topic_;
  std::string gps_common_topic_;
  
  bool broadcast_cartesian_tf_;
  bool broadcast_odom_;
  bool use_fake_odom_;

};

}  // namespace joyride_odometry

#endif  // JOYRIDE_ODOMETRY__NAVSAT_ODOM_HPP_