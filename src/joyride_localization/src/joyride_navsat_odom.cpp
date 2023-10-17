


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

#include "joyride_localization/joyride_navsat_odom.hpp"
#include <cmath>
#include "rclcpp/time.hpp"
#include "joyride_interfaces/srv/get_odom_origin_ll.hpp"


using namespace std::chrono_literals;


namespace joyride_odometry
{
NavSatOdom::NavSatOdom(const rclcpp::NodeOptions & options) : Node("navsat_odom_node", options),
    odom_broadcaster_(*this)
{

    // Initialize parameters
    this->initializeParameters();

    // Initialize ROS
    this->initializeROS();
}

NavSatOdom::~NavSatOdom()
{
    //RCLCPP_INFO(this->get_logger(), "Shutting down NavSatOdom node");

    // Reset pointers
    this->last_position_.reset();
    this->last_velocity_.reset();
    this->last_orientation_.reset();
    this->initialLLA_fix_.reset();


}


// --------------- Setup --------------------- //
void NavSatOdom::initializeParameters()
{
    //RCLCPP_INFO(this->get_logger(), "Init params");
    this->declare_parameter("use_fake_odom", false);
    this->declare_parameter("tf_frequency", 50.0);
    this->declare_parameter("expected_lat", 36.115);
    this->declare_parameter("expected_lon", -97.0584);
    this->declare_parameter("initial_ll_radius", 10000.0);
    this->declare_parameter("broadcast_odom", true);
    this->declare_parameter("gps_common_topic", "/vectornav/raw/common");
    this->declare_parameter("odom_pub_topic", "/odom");
    this->declare_parameter("broadcast_cartesian_tf", true);
    this->declare_parameter("world_frame_id", "odom");
    this->declare_parameter("base_frame_id", "base_link");

    this->expected_lat_ = this->get_parameter("expected_lat").get_parameter_value().get<double>();
    this->expected_lon_ = this->get_parameter("expected_lon").get_parameter_value().get<double>();
    this->initial_ll_radius_ = this->get_parameter("initial_ll_radius").get_parameter_value().get<double>();
    this->base_link_frame_id_ = this->get_parameter("base_frame_id").get_parameter_value().get<std::string>();
    this->world_frame_id_ = this->get_parameter("world_frame_id").get_parameter_value().get<std::string>();
    this->broadcast_cartesian_tf_ = this->get_parameter("broadcast_cartesian_tf").get_parameter_value().get<bool>();
    this->broadcast_odom_ = this->get_parameter("broadcast_odom").get_parameter_value().get<bool>();
    this->gps_common_topic_ = this->get_parameter("gps_common_topic").get_parameter_value().get<std::string>();
    this->odom_pub_topic_ = this->get_parameter("odom_pub_topic").get_parameter_value().get<std::string>();
    this->use_fake_odom_ = this->get_parameter("use_fake_odom").get_parameter_value().get<bool>();
    this->tf_frequency_ = this->get_parameter("tf_frequency").get_parameter_value().get<double>();
    this->tf_period_ = 1.0 / this->tf_frequency_;
    //RCLCPP_INFO(this->get_logger(), "End init params");

}

void NavSatOdom::getInitialLLCallback(const std::shared_ptr<joyride_interfaces::srv::GetOdomOriginLL::Request> request, std::shared_ptr<joyride_interfaces::srv::GetOdomOriginLL::Response> response)
{
    response->is_valid = this->valid_fix_obtained_;
    
    if(this->initialLLA_fix_ != nullptr)
    {
        response->latitude = this->initialLLA_fix_->position.x;
        response->longitude = this->initialLLA_fix_->position.y;
    }
    else
    {
        response->latitude = 0.0;
        response->longitude = 0.0;
    }
}

void NavSatOdom::initializeROS()
{
    //RCLCPP_INFO(this->get_logger(), "Init ROS");

    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // Setup odom publisher
    this->odomPub_ = this->create_publisher<nav_msgs::msg::Odometry>(this->odom_pub_topic_, 10);
    this->publishOdometryTimer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000 * tf_period_)), std::bind(&NavSatOdom::publishOdometryCallback, this));

    this->getOdomOriginLLService_ = this->create_service<joyride_interfaces::srv::GetOdomOriginLL>("getInitialLL", std::bind(&NavSatOdom::getInitialLLCallback, this, std::placeholders::_1, std::placeholders::_2));
    this->resetOdomService_ = this->create_service<std_srvs::srv::Trigger>("resetOdom", std::bind(&NavSatOdom::resetOdomCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    // Initialize pointers
    this->last_position_ = std::make_shared<geometry_msgs::msg::Point>();
    this->last_velocity_ = std::make_shared<geometry_msgs::msg::Twist>();
    this->last_orientation_ = std::make_shared<geometry_msgs::msg::Quaternion>();
    





    if(!this->use_fake_odom_) {
        //RCLCPP_INFO(this->get_logger(), "No fake ROS");

        // Setup GPS Common subscriber
        this->gpsCommonSub_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>(this->gps_common_topic_, 10,
            std::bind(&NavSatOdom::gpsCommonCallback, this, std::placeholders::_1));

    }

}

void NavSatOdom::resetOdomCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    // Reset valid fix. This will cause next message to act as if new fix was obtained.
    this->valid_fix_obtained_ = false;
    RCLCPP_INFO(this->get_logger(), "Resetting Odometry LTP");

}

void NavSatOdom::gpsCommonCallback(const vectornav_msgs::msg::CommonGroup::SharedPtr msg)
{
    // If first fix, set datum, compute transforms and odom then publish
    // Otherwise, compute tfs, odom and publish

    // Check if first message we've received. If so, setup LTP.
    if (!this->valid_fix_obtained_)
    {
        
        // Ensure first point is within an acceptable distance of where we think we should be.
        // - This is to reject outliers, like LL = 00.
        double distance = llDistance(this->expected_lat_, this->expected_lon_, msg->position.x, msg->position.y);

        if(distance <= this->initial_ll_radius_)
        {
            RCLCPP_INFO(this->get_logger(), "Valid GPS Fix Obtained");

            // Accept initial pose.
            this->initialLLA_fix_ = msg;
            this->local_cartesian_plane_ = GeographicLib::LocalCartesian(initialLLA_fix_->position.x, initialLLA_fix_->position.y, 0.0);
            this->valid_fix_obtained_ = true;
            this->updateTransform(msg);
        }
    }
    else {
        this->updateTransform(msg);
    }

}

// Update measurements 
void NavSatOdom::updateTransform(const vectornav_msgs::msg::CommonGroup::SharedPtr msg)
{

    // Heads up!
    /*
        This code makes several assumptions about the structure of the incoming msg.
        It will only work with the VectorNav VN300.
        VN300 data comes in the form:
        YawPitchRoll: orientation in the NED frame
        quaternion: orientation in NED.
        position: position in lat, lon, alt
        velocity: velocity in NED (this is important! ROS wants Odom velocity in body frame)
        angular rates: in body frame (which is transformed to be equivalent to vehicle frame)
    */

    // Update position
    double lat = msg->position.x;
    double lon = msg->position.y;

    double newX, newY, newZ;
    this->local_cartesian_plane_.Forward(lat, lon, 0.0, newX, newY, newZ);

    this->last_position_->x = newX;
    this->last_position_->y = newY;
    this->last_position_->z = newZ;

    // Update yaw (in odom frame)
    double yaw = (90.0 - msg->yawpitchroll.x) * M_PI / 180.0; // Respect to East (rather than North), Convert to radians
    
    tf2::Quaternion yawQuat;
    yawQuat.setRPY(0, 0, yaw);


    // Update linear velocity (body frame)
    double velNED_n = msg->velocity.x;
    double velNED_e = msg->velocity.y;
    tf2::Vector3 velNED = tf2::Vector3(velNED_n, velNED_e, 0.0);
    tf2::Vector3 yawNED = tf2::Vector3(cos(msg->yawpitchroll.x * M_PI/180.0), sin(msg->yawpitchroll.x * M_PI/180.0), 0.0);
    double angle_diff = velNED.angle(yawNED);
    double velBody_x = velNED.length();
    bool dir = abs(angle_diff) > M_PI/2.0; // If angle between vectors greater than pi, we are travelling backwards

    
    // Update angular velocity (body frame)
    tf2::Quaternion quat = tf2::Quaternion(msg->quaternion.x, msg->quaternion.y, msg->quaternion.z, msg->quaternion.w);
    tf2::Vector3 angRateNED = tf2::Vector3(msg->angularrate.x, msg->angularrate.y, msg->angularrate.z);
    tf2::Vector3 angRateBody = tf2::quatRotate(quat, angRateNED);
    double angRateBody_z = -angRateBody.z(); // Invert yaw rate to match ROS convention

    this->last_velocity_->linear.x = dir ? -velBody_x : velBody_x;
    this->last_velocity_->angular.z = angRateBody_z;
    this->last_orientation_->x = yawQuat.x(); // Probably not the simplest way to do this.
    this->last_orientation_->y = yawQuat.y();
    this->last_orientation_->z = yawQuat.z();
    this->last_orientation_->w = yawQuat.w();
}

void NavSatOdom::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    if (this->valid_fix_obtained_)
    {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "GPS Fix Obtained");
        stat.add("Home latitude", this->initialLLA_fix_->position.x);
        stat.add("Home longitude", this->initialLLA_fix_->position.y);
        stat.add("Current latitude", this->last_position_->x);
        stat.add("Current longitude", this->last_position_->y);
    }
    else
    {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No GPS Fix Obtained");
    }
}

nav_msgs::msg::Odometry NavSatOdom::buildOdometryMessage(const geometry_msgs::msg::Point::SharedPtr position, const geometry_msgs::msg::Quaternion::SharedPtr orientation, const geometry_msgs::msg::Twist::SharedPtr velocity, const std::string frame_id, const std::string child_frame_id)
{
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = frame_id;
    odom.child_frame_id = child_frame_id;
    odom.pose.pose.position = *position;
    odom.pose.pose.orientation = *orientation;
    odom.twist.twist = *velocity;

    return odom;
}

geometry_msgs::msg::TransformStamped NavSatOdom::buildOdometryTransform(const geometry_msgs::msg::Point::SharedPtr position, const geometry_msgs::msg::Quaternion::SharedPtr orientation, const std::string frame_id, const std::string child_frame_id)
{
    geometry_msgs::msg::TransformStamped odomTF;
    odomTF.header.stamp = this->get_clock()->now();
    odomTF.header.frame_id = frame_id;
    odomTF.child_frame_id = child_frame_id;
    odomTF.transform.translation.x = position->x;
    odomTF.transform.translation.y = position->y;
    odomTF.transform.translation.z = position->z;
    odomTF.transform.rotation = *orientation;

    return odomTF;
}

// publishOdomTimerCallback
void NavSatOdom::publishOdometryCallback()
{

    if(this->valid_fix_obtained_) {
        //RCLCPP_INFO(this->get_logger(), "Attempting publish");

        // Publish new odom and TF
        geometry_msgs::msg::TransformStamped odomTF = buildOdometryTransform(this->last_position_, this->last_orientation_, this->world_frame_id_, this->base_link_frame_id_);
        nav_msgs::msg::Odometry odomMsg = buildOdometryMessage(this->last_position_, this->last_orientation_, this->last_velocity_, this->world_frame_id_, this->base_link_frame_id_);

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

}