#include "joyride_localization/joyride_fake_odom.hpp"

//needs to be added for the transform call tf2_ros::TransformBroadcaster odom_broadcaster_; to the header file
//maybe also needs the rest of the TF2 stuff such as the buffer and listener in both the .cpp and .hpp files
namespace joyride_odometry
{
JoyrideFakeOdom::JoyrideFakeOdom(const rclcpp::NodeOptions &options) : Node("joyride_fake_odom", options),
    odom_broadcaster_(*this) //Appears this is used to call the transform
{
    initializeParameters();
    initializeROS();
}

JoyrideFakeOdom::~JoyrideFakeOdom()
{
}

void JoyrideFakeOdom::initializeParameters()
{

    this->declare_parameter("linear_lag", 0.1);
    this->declare_parameter("angular_lag", 0.1);
    this->declare_parameter("pub_odom_rate", 100.0);
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("cmd_ack_topic", "cmd_ack");
    this->declare_parameter("odom_frame", "odom");
    // this->declare_parameter("local_cartesian", "local_enu");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("wheelbase", 1.75);
    

    this->linear_lag_ = this->get_parameter("linear_lag").as_double();
    this->angular_lag_ = this->get_parameter("angular_lag").as_double();
    this->pub_odom_rate_ = this->get_parameter("pub_odom_rate").as_double();
    this->odom_topic_ = this->get_parameter("odom_topic").as_string();
    this->cmd_ack_topic_ = this->get_parameter("cmd_ack_topic").as_string();
    this->odom_frame_ = this->get_parameter("odom_frame").as_string();
    this->base_frame_ = this->get_parameter("base_frame").as_string();
    this->wheelbase_ = this->get_parameter("wheelbase").as_double();
}



void JoyrideFakeOdom::initializeROS()
{
    cmd_ack_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(cmd_ack_topic_, 10, std::bind(&joyride_odometry::JoyrideFakeOdom::newCmdAckCallback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    pub_odom_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / pub_odom_rate_)), std::bind(&joyride_odometry::JoyrideFakeOdom::publishOdomCallback, this));

    //TF2 init
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
   
    position_x_ = 0.0;
    position_y_ = 0.0;
    position_z_ = 0.0;
    yaw_ = 0.0;

    velocity_x_ = 0.0;
    velocity_y_ = 0.0;
    velocity_z_ = 0.0;
    yaw_rate_ = 0.0;

    //Spoofing GPS in Gui
    // rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("NavSatOdom");
    RCLCPP_INFO(this->get_logger(), "Indoor GPS Fix Obtained");
}

void JoyrideFakeOdom::newCmdAckCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg)
{
    computeOdom(msg);
}

void JoyrideFakeOdom::publishOdomCallback()
{
    nav_msgs::msg::Odometry odom_msg;

    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = position_z_;

    //NOTE: Might want to initiallize in the same structure as odom to improve readability
    // 
    odom_msg.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_));

    odom_msg.twist.twist.linear.x = velocity_x_;
    odom_msg.twist.twist.linear.y = velocity_y_;
    odom_msg.twist.twist.linear.z = velocity_z_;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = yaw_rate_;

    odom_pub_->publish(odom_msg);

    
    geometry_msgs::msg::TransformStamped odomTF;
    odomTF.header.stamp = this->get_clock()->now(); 
    odomTF.header.frame_id = odom_frame_;
    odomTF.child_frame_id = base_frame_;
    odomTF.transform.translation.x = position_x_;
    odomTF.transform.translation.y = position_y_;
    odomTF.transform.translation.z = position_z_;
    odomTF.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_));

    odom_broadcaster_.sendTransform(odomTF);

}

void JoyrideFakeOdom::computeOdom(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg)
{
    double dt = 1.0 / pub_odom_rate_;

    double linear_velocity = msg->speed;
    double angular_velocity = tan(msg->steering_angle) * msg->speed / wheelbase_;

    double pos_diff = linear_velocity * dt;
    double yaw_diff = angular_velocity * dt;

    position_x_ += pos_diff * cos(yaw_);
    position_y_ += pos_diff * sin(yaw_);
    yaw_ += yaw_diff;

    velocity_x_ = linear_velocity;
    velocity_y_ = 0.0;
    velocity_z_ = 0.0;
    yaw_rate_ = angular_velocity;
    
}

} // namespace joyride_odometry