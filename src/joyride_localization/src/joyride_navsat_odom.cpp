


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

#include "joyride_localization/joyride_navsat_odom.hpp"
#include <cmath>
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
    this->declare_parameter("tf_frequency", 50.0);
    this->declare_parameter("expected_lat", 36.1156);
    this->declare_parameter("expected_lon", -97.0584);
    this->declare_parameter("initial_ll_radius", 1.0);

    this->expected_lat_ = this->get_parameter("expected_lat").get_parameter_value().get<double>();
    this->expected_lon_ = this->get_parameter("expected_lon").get_parameter_value().get<double>();
    this->initial_ll_radius_ = this->get_parameter("initial_ll_radius").get_parameter_value().get<double>();


    this->use_fake_odom_ = this->get_parameter("use_fake_odom").get_parameter_value().get<bool>();
    this->tf_frequency_ = this->get_parameter("tf_frequency").get_parameter_value().get<double>();
    this->tf_period_ = 1.0 / this->tf_frequency_;
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // Setup odom publisher
    odomPub_ = this->create_publisher<nav_msgs::msg::Odometry>(this->odom_pub_topic_, 10);

    odomTimer_ = this->create_wall_timer(10ms, std::bind(&NavSatOdom::odomCallback, this));

    if(!this->use_fake_odom_) {
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

void NavSatOdom::odomCallback()
{
    if(this->use_fake_odom_) {
        tf2::Quaternion yawQuat;
        yawQuat.setRPY(0, 0, 0);
        geometry_msgs::msg::Quaternion orientation = tf2::toMsg(yawQuat);

        odom_broadcaster_.sendTransform(buildOdomTF(0, 0, 0, orientation, "odom", "base_link"));
        odomPub_->publish(buildOdomMsg(0, 0, 0, orientation, "odom", "base_link"));
    }
    else if(this->valid_fix_obtained_) {
        // Publish new odom and TF
        geometry_msgs::msg::TransformStamped odomTF = buildOdomTF(this->last_x_, this->last_y_, 0, this->last_yaw_, this->world_frame_id_, this->base_link_frame_id_);
        nav_msgs::msg::Odometry odomMsg = buildOdomMsg(this->last_x_, this->last_y_, 0, this->last_yaw_, this->world_frame_id_, this->base_link_frame_id_);

        odom_broadcaster_.sendTransform(odomTF);
        odomPub_->publish(odomMsg);
    }
}

double NavSatOdom::llDistance(double lat1, double lon1, double lat2, double lon2)
{
    double deg2rad = 3.14159/180.0;
    lat1 = lat1*deg2rad;
    lat2 = lat2*deg2rad;
    lon1 = lon1*deg2rad;
    lon2 = lon2*deg2rad;
    double R = 6371; // Earth's radius
    double dLat = lat2-lat1;
    double dLon = lon2-lon1;
    double temp = sin(dLat/2)*sin(dLat/2) + cos(lat1) * cos(lat2) * sin(dLon/2) * sin(dLon/2);
    temp = 2 * atan2(sqrt(temp), sqrt(1-temp));
    return R * temp * 1609.34; //1609.34 miles to meters conversion
}

void NavSatOdom::gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    // If first fix, set datum, compute transforms and odom then publish
    // Otherwise, compute tfs, odom and publish

    // Check if first message we've received. If so, setup LTP.
    if (!this->valid_fix_obtained_)
    {
        
        // Ensure first point is within an acceptable distance of where we think we should be.
        // - This is to reject outliers, like LL = 00.
        double distance = llDistance(this->expected_lat_, this->expected_lon_, msg->latitude, msg->longitude);

        if(distance <= this->initial_ll_radius_ && msg->status.status > sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
        {
            // Accept initial pose.
            this->initialLLA_fix_ = *msg;
            local_cartesian_plane_ = GeographicLib::LocalCartesian(initialLLA_fix_.latitude, initialLLA_fix_.longitude, 0.0);
            this->valid_fix_obtained_ = true;
            updateTF(msg);
        }
    }
    else {
        updateTF(msg);
    }

}

void NavSatOdom::updateTF(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    // Get yaw
    tf2::Quaternion yawQuat;
    yawQuat.setRPY(0, 0, this->previousYawAngleRadians_);

    // Save values for next callback
    this->last_yaw_ = tf2::toMsg(yawQuat);
    local_cartesian_plane_.Forward(msg->latitude, msg->longitude, 0, this->last_x_, this->last_y_, this->last_z_);
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
