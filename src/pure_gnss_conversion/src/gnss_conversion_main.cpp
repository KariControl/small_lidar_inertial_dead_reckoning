#include "pure_gnss_conversion/gnss_conversion.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssConversion>());
  rclcpp::shutdown();
  return 0;
}
