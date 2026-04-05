#include <memory>

#include "pure_gnss_map_odom_fusion/map_odom_fusion_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pure_gnss_map_odom_fusion::MapOdomFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
