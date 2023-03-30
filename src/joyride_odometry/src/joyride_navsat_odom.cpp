


// 1. Subscribe to navsatfix message
// 2. Allow for manual datum specification or initial position


// SUBSCRIBED TOPICS
// - navsatfix

// PUBLISHED TOPICS
// - /odom

// PUBLISHED TRANFORMS
// - odom->base_link

// SERVICES
// - Set Datum

// PARAMETERS
// - use_manual_datum
// - publish odom
// - publish transform
// - odom frame id
// - base frame id
// - publish_static_map_transform - whether or not to publish an odom = map transform


// Credit - Largely based off of robot_localization's navsat_transform_node

#include "joyride_odometry/joyride_navsat_odom.hpp"
#include "rclcpp/time.hpp"

using namespace std::chrono_literals;


namespace joyride_odometry
{
NavSatOdom::NavSatOdom(const rclcpp::NodeOptions & options) : Node("navsat_odom_node", options),
    base_link_frame_id_("base_link"),
    world_frame_id_("odom"),
    broadcast_cartesian_tf_(true),
    broadcast_odom_(true),
    navsat_sub_topic_("/vectornav/gnss"),
    imu_sub_topic_("/vectornav/imu"),
    odom_pub_topic_("/odom"),
    odom_broadcaster_(*this)
{
    this->declare_parameter("use_fake_odom", false);
    this->use_fake_odom_ = this->get_parameter("use_fake_odom").get_parameter_value().get<bool>();
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // Setup odom publisher
    odomPub_ = this->create_publisher<nav_msgs::msg::Odometry>(this->odom_pub_topic_, 10);

    if(use_fake_odom_) {
        fakeOdomTimer_ = this->create_wall_timer(10ms, std::bind(&NavSatOdom::fakeOdomTimerCallback, this));
        // cmdVelSub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
        //     std::bind(&NavSatOdom::cmdVelCallback, this, std::placeholders::_1));
    }
    else {


        // Subscribe to navsatfix topic
        navSatSub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(this->navsat_sub_topic_, 10,
            std::bind(&NavSatOdom::gpsFixCallback, this, std::placeholders::_1));

        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(this->imu_sub_topic_, 10,
            std::bind(&NavSatOdom::imuCallback, this, std::placeholders::_1));
    }
}

NavSatOdom::~NavSatOdom()
{

}

void NavSatOdom::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->last_cmd_angz_ = msg->angular.z;
    this->last_cmd_linx_ = msg->linear.x;
}

void NavSatOdom::fakeOdomTimerCallback()
{
    tf2::Quaternion yawQuat;
    yawQuat.setRPY(0, 0, 0);
    geometry_msgs::msg::Quaternion orientation = tf2::toMsg(yawQuat);

    odom_broadcaster_.sendTransform(buildOdomTF(0, 0, 0, orientation, "odom", "base_link"));
    odomPub_->publish(buildOdomMsg(0, 0, 0, orientation, "odom", "base_link"));
}

void NavSatOdom::gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    // If first fix, set datum, compute transforms and odom then publish
    // Otherwise, compute tfs, odom and publish

    // Check if first message we've received. If so, setup LTP.
    if (initialLLA_fix_.header.frame_id.empty())
    {
        initialLLA_fix_ = *msg;
        local_cartesian_plane_ = GeographicLib::LocalCartesian(initialLLA_fix_.latitude, initialLLA_fix_.longitude, 0.0);
    }

    // Get yaw
    tf2::Quaternion yawQuat;
    yawQuat.setRPY(0, 0, previousYawAngleRadians_);
    geometry_msgs::msg::Quaternion orientation = tf2::toMsg(yawQuat);
    // Get XYZ
    double x, y, z;
    local_cartesian_plane_.Forward(msg->latitude, msg->longitude, 0, x, y, z);

    
    geometry_msgs::msg::TransformStamped odomTF = buildOdomTF(x, y, 0, orientation, world_frame_id_, base_link_frame_id_);

    odom_broadcaster_.sendTransform(odomTF);

    nav_msgs::msg::Odometry odomMsg = buildOdomMsg(x, y, 0, orientation, world_frame_id_, base_link_frame_id_);
    odomPub_->publish(odomMsg);
    
}

void NavSatOdom::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{

    // IMU Messages defined as having orientation with respect to due east.
    // - Therefore, we will not perform any transform.

    tf2::Quaternion orientation;
    tf2::fromMsg(msg->orientation, orientation);

    tf2::Matrix3x3 m(orientation);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    previousYawAngleRadians_ = yaw;
}

geometry_msgs::msg::TransformStamped NavSatOdom::buildOdomTF(double x, double y, double z, geometry_msgs::msg::Quaternion orientation, std::string frame_id, std::string child_frame_id)
{
    geometry_msgs::msg::TransformStamped newOdom_tf;

    newOdom_tf.header.stamp = this->get_clock()->now();
    newOdom_tf.header.frame_id = frame_id;
    newOdom_tf.child_frame_id = child_frame_id;

    newOdom_tf.transform.translation.x = x;
    newOdom_tf.transform.translation.y = y;
    newOdom_tf.transform.translation.z = z;

    newOdom_tf.transform.rotation = orientation;

    return newOdom_tf;
}


nav_msgs::msg::Odometry NavSatOdom::buildOdomMsg(double x, double y, double z, geometry_msgs::msg::Quaternion orientation, std::string frame_id, std::string child_frame_id)
{
    nav_msgs::msg::Odometry newOdom_msg;

    newOdom_msg.header.stamp = this->get_clock()->now();
    newOdom_msg.header.frame_id = frame_id;
    newOdom_msg.child_frame_id = child_frame_id;

    newOdom_msg.pose.pose.position.x = x;
    newOdom_msg.pose.pose.position.y = y;
    newOdom_msg.pose.pose.position.z = z;
    newOdom_msg.pose.pose.orientation = orientation;

    return newOdom_msg;
}

}
