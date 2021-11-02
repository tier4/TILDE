#include "pathnode/timing_advertise_publisher.hpp"

using namespace pathnode;

rclcpp::Time pathnode::get_timestamp(rclcpp::Time t, ...)
{
  std::cout << "get rclcpp::Time t" << std::endl;
  return t;
}

void TimingAdvertisePublisherBase::set_input_info(
    const std::string &sub_topic,
    const std::shared_ptr<InputInfo> p)
{
  input_infos_[sub_topic] = p;
}
