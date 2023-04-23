#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "joyride_control/joyride_velocity_preprocessor.hpp"
#include <diagnostic_updater/diagnostic_updater.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    const rclcpp::NodeOptions options;

    auto vel_preprocessor = std::make_shared<joyride_control::VelocityPreprocessor>(options);

    diagnostic_updater::Updater updater(vel_preprocessor);

    updater.setHardwareID("vel_preprocessor");
    updater.add("vel_preprocessor", vel_preprocessor.get(), &joyride_control::VelocityPreprocessor::diagnosticCallback);

    rclcpp::spin(vel_preprocessor->get_node_base_interface());

    rclcpp::shutdown();
    
    return 0;
}