
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
#include "tf2_ros/buffer.h"
#include "vectornav_msgs/msg/common_group.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace joyride_odometry
{
class NavSatOdom : public rclcpp::Node
{
public:

  // Setup
  explicit NavSatOdom(const rclcpp::NodeOptions&);
  ~NavSatOdom();


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

  // void gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  // void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  // void odomCallback();
  // void updateTF(const vectornav_msgs::msg::CommonGroup::SharedPtr msg);
  // void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  // void gpsCommonCallback(const vectornav_msgs::msg::CommonGroup::SharedPtr msg);






  // // Helpers
  
  // double llDistance(double lat1, double lon1, double lat2, double lon2);


  // // Variables
  // geometry_msgs::msg::TransformStamped buildOdomTF(double x, double y, double z,
  //                                                  geometry_msgs::msg::Quaternion orientation, std::string frame_id,
  //                                                  std::string child_frame_id);

  // nav_msgs::msg::Odometry buildOdomMsg(double x, double y, double z, double v_x, double v_y, double v_z, double angular_z, geometry_msgs::msg::Quaternion orientation,
  //                                      std::string frame_id, std::string child_frame_id);

  // // Subscribers
  // rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navSatSub_;
  // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
  // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
  // rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr gpsCommonSub_;

  // // Publishers
  // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;

  // // Timers
  // rclcpp::TimerBase::SharedPtr odomTimer_;


  // // Parameters
  // std::string base_link_frame_id_;
  // std::string world_frame_id_;
  // std::string navsat_sub_topic_;
  // std::string imu_sub_topic_;
  // std::string odom_pub_topic_;

  // bool broadcast_cartesian_tf_;
  // bool broadcast_odom_;
  // bool use_fake_odom_;

  // double fake_pos_x;
  // double fake_pos_y;
  // double fake_yaw_;
  // double last_cmd_linx_;
  // double last_cmd_angz_;
  // double tf_frequency_;
  // double tf_period_;

  // double expected_lat_;
  // double expected_lon_;
  // double initial_ll_radius_;

  // State

};

}  // namespace joyride_odometry

#endif  // JOYRIDE_ODOMETRY__NAVSAT_ODOM_HPP_