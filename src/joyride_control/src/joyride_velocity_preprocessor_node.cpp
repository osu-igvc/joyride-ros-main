#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "joyride_control/joyride_velocity_preprocessor.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    const rclcpp::NodeOptions options;

    auto vel_preprocessor = std::make_shared<joyride_control::VelocityPreprocessor>(options);

    rclcpp::spin(vel_preprocessor->get_node_base_interface());

    rclcpp::shutdown();
    
    return 0;
}