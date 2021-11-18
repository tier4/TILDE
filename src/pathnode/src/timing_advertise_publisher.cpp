#include "pathnode/timing_advertise_publisher.hpp"

using namespace pathnode;

rclcpp::Time pathnode::get_timestamp(rclcpp::Time t, ...)
{
  std::cout << "get rclcpp::Time t" << std::endl;
  return t;
}

void TimingAdvertisePublisherBase::set_input_info(
    const std::string &sub_topic,
    const std::shared_ptr<const InputInfo> p)
{
  input_infos_[sub_topic] = p;
}

void TimingAdvertisePublisherBase::set_explicit_input_info(
    const std::string &sub_topic,
    const rclcpp::Time &stamp)
{
  // TODO set me
  // explicit_input_infos_[sub_topic].sub_time;
  explicit_input_infos_[sub_topic].has_header_stamp = true;
  explicit_input_infos_[sub_topic].header_stamp = stamp;
}

void TimingAdvertisePublisherBase::set_input_info(path_info_msg::msg::PubInfo &info_msg)
{
  if(explicit_input_infos_.size() == 0) {
    info_msg.input_infos.resize(input_infos_.size());

    size_t i = 0;
    for(const auto &[topic, input_info]: input_infos_) {
      info_msg.input_infos[i].topic_name = topic;
      info_msg.input_infos[i].sub_time = input_info->sub_time;
      info_msg.input_infos[i].has_header_stamp = input_info->has_header_stamp;
      info_msg.input_infos[i].header_stamp = input_info->header_stamp;
      i++;
    }
  } else {
    info_msg.input_infos.resize(explicit_input_infos_.size());

    size_t i = 0;
    for(const auto &[topic, input_info]: explicit_input_infos_) {
      info_msg.input_infos[i].topic_name = topic;
      info_msg.input_infos[i].sub_time = input_info.sub_time;
      info_msg.input_infos[i].has_header_stamp = input_info.has_header_stamp;
      info_msg.input_infos[i].header_stamp = input_info.header_stamp;
      i++;
    }
    explicit_input_infos_.clear();
  }
}
