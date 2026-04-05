#include <rclcpp/rclcpp.hpp>
#include "pure_imu_undistortion/imu_undistorter.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pure_imu_undistortion::ImuUndistorter>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
