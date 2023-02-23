#include "rclcpp/rclcpp.hpp"
#include "joyride_control_lib/pid.h"

class SteeringPID : public rclcpp::Node {
  public:
    SteeringPID();

  private:
    rclcpp::Time T; 
    double time;

    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_; 
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    void pid_callback(const std_msgs::msg::String::SharedPtr msg);
    
    PID* controller;
};