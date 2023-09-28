#include <rclcpp/rclcpp.hpp>
#include "robione_ros2_driver/robione.h"
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robione::RobioneInterface>();
    rclcpp::spin(node);
  return 0;
}
