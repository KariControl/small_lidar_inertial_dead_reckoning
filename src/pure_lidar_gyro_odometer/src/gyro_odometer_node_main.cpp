#include <rclcpp/rclcpp.hpp>

#include "pure_lidar_gyro_odometer/gyro_odometer_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pure_gyro_odometer::GyroOdometerNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
