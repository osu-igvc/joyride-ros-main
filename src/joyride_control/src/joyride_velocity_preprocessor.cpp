#include "joyride_control/joyride_velocity_preprocessor.hpp"
#include "rclcpp/time.hpp"

using namespace std::chrono_literals;
using namespace joyride_control;
using namespace std;

VelocityPreprocessor::VelocityPreprocessor(const rclcpp::NodeOptions &options) : Node("velocity_preprocessor", options)
{
    this->initializeParameters();
    this->initializeROS();
}

void VelocityPreprocessor::initializeParameters()
{
    this->declare_parameter("pub_ackermann_rate", 10.0);
    this->declare_parameter("max_steering_angle_degrees", 30.0);
    this->declare_parameter("max_steering_angele_rads", 3.1415/6);
    this->declare_parameter("max_steering_velocity", 0.5);
    this->declare_parameter("min_steering_velocity", 2);
    this->declare_parameter("max_speed_mps_fow", 1.0);
    this->declare_parameter("max_speed_mps_rev", 0.5);
    this->declare_parameter("max_acceleration_mps2", 0.5);
    this->declare_parameter("max_decceleration_mps2", 0.5);
    this->declare_parameter("wheelbase_meters", 0.33);
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter("ackermann_topic", "/ackermann");

    this->pub_ackermann_rate_ = this->get_parameter("pub_ackermann_rate").as_double();
    this->max_steering_angle_degrees_ = this->get_parameter("max_steering_angle_degrees").as_double();
    this->max_steering_angle_rads_ = this->get_parameter("max_steering_angle_rads").as_double();
    this->max_steering_velocity_radps_ = this->get_parameter("max_steering_velocity").as_double();
    this->min_steering_velocity_radps_ = this->get_parameter("min_steering_velocity").as_double();
    this->max_speed_mps_fow_ = this->get_parameter("max_speed_mps_fow").as_double();
    this->max_speed_mps_rev_ = this->get_parameter("max_speed_mps_rev").as_double();
    this->max_acceleration_mps2_ = this->get_parameter("max_acceleration_mps2").as_double();
    this->max_decceleration_mps2_ = this->get_parameter("max_decceleration_mps2").as_double();
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
    this->commanded_speed_ = max(min(this->target_speed_, this->current_speed_ + this->max_acceleration_mps2_),this->current_speed_ - this->max_decceleration_mps2_);
    this->commanded_speed_ = max(min(this->commanded_speed_, this->max_speed_mps_fow_), this->max_speed_mps_rev_);

    this->commanded_steering_angle_ = this->target_steering_angle_;
    this->commanded_steering_velocity_ = this->COEFFS[0] * exp(-this->COEFFS[1]*pow(this->steering_angle_error_ - this->COEFFS[2],2));

    /*
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
    } */
}

// Our targets from the motion controller or joystick. Prefiltered by velocity smoother.
void VelocityPreprocessor::newCMDVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (abs(msg->linear.x) < LINEAR_X_ZERO_THRESHOLD || abs(msg->angular.z) < ANGULAR_Z_ZERO_THRESHOLD)
    {
        this->target_speed_ = 0.0;
        this->target_steering_angle_ = this->current_steering_angle_; // Keep current steering angle
        return;
    }

    // Bound Speed
    msg->linear.x = min(max(msg->linear.x, this->max_speed_mps_rev_), this->max_speed_mps_fow_);
    this->target_speed_ = msg->linear.x;

    // Bounds and Deadbands on steering
    double new_steering_angle = atan2(this->wheelbase_meters_ * msg->angular.z, msg->linear.x);
    double steering_angle_change = new_steering_angle - this->current_steering_angle_;
    this->target_steering_angle_ = abs(steering_angle_change) > STEERING_CHANGE_THRESHOLD_RAD ? new_steering_angle : this->current_steering_angle_;
    this->target_steering_angle_ = min(max(this->target_steering_angle_, -this->max_steering_angle_rads_), this->max_steering_angle_rads_);
    this->steering_angle_error_ = this->target_steering_angle_ - this->commanded_steering_angle_;

    // Coefficiencts for steering speed mapping
    double phi_error_mid = steering_angle_change/2;
    this->COEFFS[0] = min(this->max_steering_velocity_radps_, phi_error_mid/(2*this->max_steering_angle_rads_) + this->min_steering_velocity_radps_);
    this->COEFFS[1] = -log(this->min_steering_velocity_radps_/this->max_steering_velocity_radps_)/(phi_error_mid*phi_error_mid);
    this->COEFFS[2] = phi_error_mid;
}

// Measures speed and steering angle from GPS
void VelocityPreprocessor::newOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    this->current_speed_ = msg->twist.twist.linear.x;
    this->current_steering_angle_ = atan2(msg->twist.twist.angular.z * this->wheelbase_meters_, msg->twist.twist.linear.x);
}
