
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "joyride_localization/joyride_navsat_odom.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    const rclcpp::NodeOptions options;

    auto navsat_odom_node = std::make_shared<joyride_odometry::NavSatOdom>(options);

    rclcpp::spin(navsat_odom_node->get_node_base_interface());

    rclcpp::shutdown();
    
    return 0;
}