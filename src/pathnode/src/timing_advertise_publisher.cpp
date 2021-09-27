#include "pathnode/timing_advertise_publisher.hpp"

rclcpp::Time pathnode::get_timestamp(rclcpp::Time t, ...)
{
  std::cout << "get rclcpp::Time t" << std::endl;
  return t;
}


