#include <rclcpp/rclcpp.hpp>
#include "map_manager/occupancyMap.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = mapManager::occMap::create();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
