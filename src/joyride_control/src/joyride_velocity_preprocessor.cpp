#include "joyride_control/joyride_velocity_preprocessor.hpp"
#include "rclcpp/time.hpp"

using namespace std::chrono_literals;


namespace joyride_control
{

VelocityPreprocessor::VelocityPreprocessor(const rclcpp::NodeOptions &options) : Node("velocity_preprocessor", options)
{
    this->initializeParameters();
    this->initializeROS();
}

void VelocityPreprocessor::initializeParameters()
{
    this->declare_parameter("pub_ackermann_rate", 30.0);
    this->declare_parameter("max_steering_angle_rad", 1.5);
    this->declare_parameter("max_speed_mps", 1.0);
    this->declare_parameter("max_acceleration_mps2", 0.5);
    this->declare_parameter("wheelbase_meters", 1.7526);
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter("ackermann_topic", "/cmd_ack");
    this->declare_parameter("max_steering_velocity_radps", 0.7);
    this->declare_parameter("max_steering_acceleration_radps2", 0.2);
    this->declare_parameter("steering_alpha", 0.2);
    this->declare_parameter("speed_alpha", 0.5);

    this->pub_ackermann_rate_ = this->get_parameter("pub_ackermann_rate").as_double();
    this->max_steering_angle_rad_ = this->get_parameter("max_steering_angle_rad").as_double();
    this->max_speed_mps_ = this->get_parameter("max_speed_mps").as_double();
    this->max_acceleration_mps2_ = this->get_parameter("max_acceleration_mps2").as_double();
    this->wheelbase_meters_ = this->get_parameter("wheelbase_meters").as_double();
    this->odom_topic_ = this->get_parameter("odom_topic").as_string();
    this->cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    this->ackermann_topic_ = this->get_parameter("ackermann_topic").as_string();
    this->max_steering_velocity_radps_ = this->get_parameter("max_steering_velocity_radps").as_double();
    this->max_steering_acceleration_radps2_ = this->get_parameter("max_steering_acceleration_radps2").as_double();
    this->steering_alpha_ = this->get_parameter("steering_alpha").as_double();
    this->speed_alpha_ = this->get_parameter("speed_alpha").as_double();
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

    double new_speed = this->current_speed_;
    double new_steering_angle = this->current_steering_angle_;
    double new_steering_velocity = this->previous_steering_velocity_;

    RCLCPP_INFO(this->get_logger(), "Target steering angle: %f", this->target_steering_angle_);

    if (this->target_steering_angle_ > this->current_steering_angle_)
    {
        new_steering_angle += this->max_steering_velocity_radps_ / this->pub_ackermann_rate_;
        //RCLCPP_INFO(this->get_logger(), "Commanded steering angle: %f", this->commanded_steering_angle_);
        if (new_steering_angle > this->target_steering_angle_)
        {
            new_steering_angle = this->target_steering_angle_;
        }

        // Potential change here is to scale the steering velocity based off of error, then filter it to smooth it out

        // // Update steering velocity
        // new_steering_velocity += this->max_steering_acceleration_radps2_ / this->pub_ackermann_rate_;
        // if (new_steering_velocity > this->max_steering_velocity_radps_)
        // {
        //     new_steering_velocity = this->max_steering_velocity_radps_;
        // }
    }

    else if (this->target_steering_angle_ < this->current_steering_angle_)
    {
        new_steering_angle -= this->max_steering_velocity_radps_ / this->pub_ackermann_rate_;
        if (new_steering_angle < this->target_steering_angle_)
        {
            new_steering_angle = this->target_steering_angle_;
        }

        // new_steering_velocity -= this->max_steering_acceleration_radps2_ / this->pub_ackermann_rate_;
        // if (new_steering_velocity < -this->max_steering_velocity_radps_)
        // {
        //     new_steering_velocity = -this->max_steering_velocity_radps_;
        // }
    }
    

    // Update speed
    if (this->target_speed_ > this->current_speed_)
    {
        new_speed += this->max_acceleration_mps2_ / this->pub_ackermann_rate_;
        if (new_speed> this->target_speed_)
        {
            new_speed = this->target_speed_;
        }
    }
    else if (this->target_speed_ < this->current_speed_)
    {
        new_speed -= this->max_acceleration_mps2_ / this->pub_ackermann_rate_;
        if (new_speed < this->target_speed_)
        {
            new_speed = this->target_speed_;
        }
    }

    new_steering_velocity = abs((new_steering_angle - this->current_steering_angle_) * 3.0);
    new_steering_velocity = firstOrderLowpass(new_steering_velocity, this->previous_steering_velocity_, this->steering_alpha_);
    new_steering_velocity = std::min(std::max(new_steering_velocity, -this->max_steering_velocity_radps_), this->max_steering_velocity_radps_);

    new_speed = std::min(std::max(new_speed, -this->max_speed_mps_), this->max_speed_mps_);
    new_steering_angle = std::min(std::max(new_steering_angle, -this->max_steering_angle_rad_), this->max_steering_angle_rad_);
    if(abs(new_steering_angle - this->current_steering_angle_) < 0.005)
    {
        new_steering_angle = this->current_steering_angle_;
    }

    this->commanded_speed_ = new_speed;
    this->commanded_steering_angle_ = new_steering_angle;
    this->commanded_steering_velocity_ = new_steering_velocity;
    this->previous_steering_velocity_ = this->commanded_steering_velocity_;
    // RCL log commands
    //RCLCPP_INFO(this->get_logger(), "Commanded speed: %f, Commanded steering angle: %f", this->commanded_speed_, this->commanded_steering_angle_);

}

double VelocityPreprocessor::firstOrderLowpass(double new_input, double old_output, double alpha)
{
    new_input += alpha * (new_input - old_output);
    return new_input;
}

// Our targets from the motion controller or joystick. Prefiltered by velocity smoother.
void VelocityPreprocessor::newCMDVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{

    double linear_x = msg->linear.x;
    double steering_angle = 0;

    if (abs(linear_x) < ZERO_THRESHOLD)
    {
        linear_x = 0;
        steering_angle = 0; //this->current_steering_angle_;
    }
    else if (abs(msg->angular.z) < ZERO_THRESHOLD)
    {
        linear_x = msg->linear.x;
        steering_angle = 0;
    }
    else
    {
        // Limit speed
        if(abs(linear_x) > this->max_speed_mps_)
        {
            linear_x = linear_x > 0 ? this->max_speed_mps_ : -this->max_speed_mps_;
        }

        double new_steering_angle = atan2(this->wheelbase_meters_ * msg->angular.z, msg->linear.x);
        //RCLCPP_INFO(this->get_logger(), "New steering angle: %f", new_steering_angle);
        // Only update steering angle if it is sufficiently different, to reduce chatter
        if(abs(new_steering_angle - this->current_steering_angle_) < STEERING_CHANGE_THRESHOLD_RAD)
        {
            steering_angle = this->current_steering_angle_;
        }
        else {
            steering_angle = new_steering_angle;
        }
    }

    // Smooth targets
    linear_x = firstOrderLowpass(linear_x, this->current_speed_, this->speed_alpha_);
    steering_angle = firstOrderLowpass(steering_angle, this->current_steering_angle_, this->steering_alpha_);

    this->target_speed_ = linear_x;
    this->target_steering_angle_ = steering_angle;
}

// Measures speed and steering angle from GPS
void VelocityPreprocessor::newOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    this->current_speed_ = msg->twist.twist.linear.x;

    // Only update steering angle if we are moving
    if (abs(msg->twist.twist.linear.x) > ZERO_THRESHOLD)
    {
        this->current_steering_angle_ = atan2(msg->twist.twist.angular.z * this->wheelbase_meters_, msg->twist.twist.linear.x);
        return;
    }
}


void VelocityPreprocessor::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
    
}
}