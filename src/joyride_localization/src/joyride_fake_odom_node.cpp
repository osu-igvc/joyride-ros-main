#include "rclcpp/rclcpp.hpp"
#include "joyride_localization/joyride_fake_odom.hpp"
#include <memory>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    const rclcpp::NodeOptions options;
    auto joyride_fake_odom = std::make_shared<joyride_odometry::JoyrideFakeOdom>(options);

    rclcpp::spin(joyride_fake_odom->get_node_base_interface());




    rclcpp::shutdown();
    return 0;
}