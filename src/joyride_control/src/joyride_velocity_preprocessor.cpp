#include "joyride_control/joyride_velocity_preprocessor.hpp"
#include "rclcpp/time.hpp"

using namespace std::chrono_literals;
using namespace joyride_control;

VelocityPreprocessor::VelocityPreprocessor(const rclcpp::NodeOptions &options) : Node("velocity_preprocessor", options)
{
    this->initializeParameters();
    this->initializeROS();
}

void VelocityPreprocessor::initializeParameters()
{
    this->declare_parameter("pub_ackermann_rate", 10.0);
    this->declare_parameter("max_steering_angle_degrees", 30.0);
    this->declare_parameter("max_speed_mps", 1.0);
    this->declare_parameter("max_acceleration_mps2", 0.5);
    this->declare_parameter("wheelbase_meters", 0.33);
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter("ackermann_topic", "/ackermann");

    this->pub_ackermann_rate_ = this->get_parameter("pub_ackermann_rate").as_double();
    this->max_steering_angle_degrees_ = this->get_parameter("max_steering_angle_degrees").as_double();
    this->max_speed_mps_ = this->get_parameter("max_speed_mps").as_double();
    this->max_acceleration_mps2_ = this->get_parameter("max_acceleration_mps2").as_double();
    this->wheelbase_meters_ = this->get_parameter("wheelbase_meters").as_double();
    this->odom_topic_ = this->get_parameter("odom_topic").as_string();
    this->cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    this->ackermann_topic_ = this->get_parameter("ackermann_topic").as_string();
}

void VelocityPreprocessor::initializeROS()
{
    this->ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(this->ackermann_topic_, 10);
    this->cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(this->cmd_vel_topic_, 10, std::bind(&VelocityPreprocessor::newCMDVelCallback, this, std::placeholders::_1));
    this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(this->odom_topic_, 10, std::bind(&VelocityPreprocessor::newOdomCallback, this, std::placeholders::_1));
    this->pub_ackermann_timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000 / this->pub_ackermann_rate_)), std::bind(&VelocityPreprocessor::publishAckermann, this));
}

// Update command and publish
void VelocityPreprocessor::publishAckermann()
{
    this->updateAckermann();

    ackermann_msgs::msg::AckermannDrive ackermann_msg;
    ackermann_msg.steering_angle = this->commanded_steering_angle_;
    ackermann_msg.steering_angle_velocity = this->commanded_steering_velocity_;
    ackermann_msg.speed = this->commanded_speed_;
    this->ackermann_pub_->publish(ackermann_msg);
}

// Using measurements and targets, compute command values that obey accel, vel limits
void VelocityPreprocessor::updateAckermann()
{
    // Update steering angle
    this->commanded_steering_angle_ = this->current_steering_angle_;
    this->commanded_steering_velocity_ = this->previous_steering_velocity_;

    if (this->target_steering_angle_ > this->current_steering_angle_)
    {
        this->commanded_steering_angle_ += this->max_steering_velocity_radps_ / this->pub_ackermann_rate_;
        if (this->commanded_steering_angle_ > this->target_steering_angle_)
        {
            this->commanded_steering_angle_ = this->target_steering_angle_;
        }

        // Update steering velocity
        this->commanded_steering_velocity_ += this->max_steering_acceleration_radps2_ / this->pub_ackermann_rate_;
        if (this->commanded_steering_velocity_ > this->max_steering_velocity_radps_)
        {
            this->commanded_steering_velocity_ = this->max_steering_velocity_radps_;
        }
    }
    else if (this->target_steering_angle_ < this->current_steering_angle_)
    {
        this->commanded_steering_angle_ -= this->max_steering_velocity_radps_ / this->pub_ackermann_rate_;
        if (this->commanded_steering_angle_ < this->target_steering_angle_)
        {
            this->commanded_steering_angle_ = this->target_steering_angle_;
        }

        this->commanded_steering_velocity_ -= this->max_steering_acceleration_radps2_ / this->pub_ackermann_rate_;
        if (this->commanded_steering_velocity_ < -this->max_steering_velocity_radps_)
        {
            this->commanded_steering_velocity_ = -this->max_steering_velocity_radps_;
        }
    }
    this->previous_steering_velocity_ = this->commanded_steering_velocity_;

    // Update speed
    this->commanded_speed_ = this->current_speed_;
    if (this->target_speed_ > this->current_speed_)
    {
        this->commanded_speed_ += this->max_acceleration_mps2_ / this->pub_ackermann_rate_;
        if (this->commanded_speed_ > this->target_speed_)
        {
            this->commanded_speed_ = this->target_speed_;
        }
    }
    else if (this->target_speed_ < this->current_speed_)
    {
        this->commanded_speed_ -= this->max_acceleration_mps2_ / this->pub_ackermann_rate_;
        if (this->commanded_speed_ < this->target_speed_)
        {
            this->commanded_speed_ = this->target_speed_;
        }
    }
}

// Our targets from the motion controller or joystick. Prefiltered by velocity smoother.
void VelocityPreprocessor::newCMDVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (abs(msg->linear.x) < ZERO_THRESHOLD || abs(msg->angular.z) < ZERO_THRESHOLD)
    {
        this->target_speed_ = 0.0;
        this->target_steering_angle_ = this->current_steering_angle_; // Keep current steering angle
        return;
    }

    // Limit speed
    if(abs(msg->linear.x) > this->max_speed_mps_)
    {
        msg->linear.x = msg->linear.x > 0 ? this->max_speed_mps_ : -this->max_speed_mps_;
    }

    this->target_speed_ = msg->linear.x;

    double new_steering_angle = atan2(this->wheelbase_meters_ * msg->angular.z, msg->linear.x);

    // Only update steering angle if it is sufficiently different, to reduce chatter
    if(abs(new_steering_angle - this->current_steering_angle_) < STEERING_CHANGE_THRESHOLD_RAD)
    {
        this->target_steering_angle_ = this->current_steering_angle_;
    }
    else {
        this->target_steering_angle_ = new_steering_angle;
    }
}

// Measures speed and steering angle from GPS
void VelocityPreprocessor::newOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    this->current_speed_ = msg->twist.twist.linear.x;
    this->current_steering_angle_ = atan2(msg->twist.twist.angular.z * this->wheelbase_meters_, msg->twist.twist.linear.x);
}
