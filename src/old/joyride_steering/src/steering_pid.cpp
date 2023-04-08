/*
steering_pid contains a node that behaves like a PID controller for EPS steering.
Input: Subscribes to topic ____ of msg type ____ -> contains a relative heading vector error.
Controller: Uses the joyride_control_lib' pid to compute output.
Output: Publishes to topic ____ of msg type ____ -> contains an EPS steering velocity command.

The PID needs to be tuned.
Antiwindup can also be set but is optional.
Source on how to tune: https://www.arxterra.com/lecture-6-pid-controllers/
Class PID has standard proportional, derivative, integral, and antiwindup. This simple controller may yield suboptimal results.
This PID assumes constant cycle frequency. If varying cycle frequency, new implementation needed.

More advanced controllers can be used:
- Jitter/Noise: first order filter (second or third order can yield s-shape/minimum jerk)
- Overshoot issues: Proportional on Measurement Controller (very similar to PID)
  - Source: http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/
*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "joyride_steering/steering_pid.hpp"

#define KP 39.0
#define KI 22.5
#define KD 17.5
#define CYCLE_FREQUENCY 50 // In Hz

SteeringPID::SteeringPID(): Node("steering_pid") {
    {
      // FIXME topic and msg
      this->subscription_ = this->create_subscription<std_msgs::msg::String>(
      "desired_heading_vector_topic", 1, std::bind(&SteeringPID::pid_callback, this, std::placeholders::_1));

      // FIXME topic and msg
      this->publisher_ = this->create_publisher<std_msgs::msg::String>("steering_vel_cmd_topic", 1);

      this->controller = new PID(CYCLE_FREQUENCY);
      this->controller->setTuning(KP, KI, KD);
      //Optional: this->controller->setAntiWindup(double minOutput, double maxOutput, double iTermMin, double iTermMax)
    }
}

void SteeringPID::pid_callback(const std_msgs::msg::String::SharedPtr msg)
{
  this->T = this->get_clock()->now();
  this->time = T.seconds();

// FIXME output message
  auto message = std_msgs::msg::String();
  
  double error_from_topic = 1.0;

  double output = this->controller->loop(error_from_topic);

  message.data = "Time:   " + std::to_string(this->time) + "\n" +
                 "Goal:   " + std::to_string(error_from_topic) + "\n" +
                 "Output: " + std::to_string(output) + "\n"; 

  publisher_->publish(message);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteeringPID>());
  rclcpp::shutdown();
  return 0;
}

