#include "pathnode/path_node.hpp"

using pathnode::PathNodeSubscriptionOptions;
using pathnode::PathNodeInfo;
using pathnode::PathNode;

PathNodeSubscriptionOptions::PathNodeSubscriptionOptions():
    path_deadline_duration_(0, 0)
{
}

PathNodeInfo::PathNodeInfo():
    CLOCK_TYPE(RCL_SYSTEM_TIME),
    path_start_time_(0, 0, CLOCK_TYPE),
    path_deadline_duration_(0, 0)
{
}

PathNode::PathNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options):
    Node(node_name, options),
    CLOCK_TYPE(RCL_SYSTEM_TIME)
{
}

PathNode::PathNode(
    const std::string & node_name,
    const std::string & namespace_,
    const rclcpp::NodeOptions & options):
    Node(node_name, namespace_, options),
    CLOCK_TYPE(RCL_SYSTEM_TIME)
{
}

PathNode::~PathNode()
{
}

void PathNode::setup_path(
    const std::string &sub_topic,
    const PathNodeSubscriptionOptions &path_node_options)
{
  const auto& path_name = path_node_options.path_name_;

  // create path_info pub/sub
  if(path_node_info_map_.find(path_name) == path_node_info_map_.end()) {
    auto info = std::make_shared<PathNodeInfo>();
    rclcpp::QoS qos(3); // TODO: may need the number of nodes in the path

    info->path_name_ = path_name;
    info->subscription_topic_name_ = sub_topic;
    info->publish_topic_names_ = path_node_options.publish_topic_names_;
    info->is_first_ = path_node_options.is_first_;
    if(info->is_first_) {
      info->path_deadline_duration_ = path_node_options.path_deadline_duration_;
    }
    info->pub_ = this->create_publisher<path_info_msg::msg::PathInfo>(path_name + "_info", qos);

    auto path_info_callback =
        [this](path_info_msg::msg::PathInfo::UniquePtr msg) -> void
        {
          auto info_it = path_node_info_map_.find(msg->path_name);
          if(info_it == path_node_info_map_.end()) {
            std::cout << "path_info not found: " << msg->path_name << std::endl;
            return;
          }
          auto info = info_it->second;

          std::cout << "path_info_callback in: " << msg->topic_name << " "
                    << "info sub topic name: " << info->subscription_topic_name_
                    << std::endl;
          if(msg->topic_name != info->subscription_topic_name_) return;

          // TODO: validate msg->valid_ns
          info->path_start_time_ = rclcpp::Time(msg->path_start, info->CLOCK_TYPE);
          info->path_deadline_duration_ = msg->path_deadline_duration;
          std::cout << "get path_info: "
                    << " topic: "  << msg->topic_name
                    << " path_start: " << info->path_start_time_.nanoseconds() << std::endl;
        };
    info->sub_ = this->create_subscription<path_info_msg::msg::PathInfo>(
        path_name + "_info", qos, path_info_callback);

    std::cout << "create_path_info: " << path_name << std::endl;
    path_node_info_map_[path_name] = info;
  }
}

void PathNode::on_pathed_subscription(const std::string &path_name)
{
  auto info_it = this->path_node_info_map_.find(path_name);
  if(info_it == this->path_node_info_map_.end()) {
    std::cout << "pathinfo not found: " << path_name << std::endl;
    return;
  }

  auto info = info_it->second;

  auto m = std::make_unique<path_info_msg::msg::PathInfo>();
  m->path_name = path_name;

  // if is_first, set start_time, deadline_duration by myself.
  // Otherwise, path_info callback does it.
  if(info->is_first_) {
    info->path_start_time_ = now();
  }

  m->path_start = info->path_start_time_;
  m->path_deadline_duration = info->path_deadline_duration_;

  for(const auto &pub_topic : info->publish_topic_names_) {
    m->topic_name = pub_topic;
    std::cout << "send path_info: " << pub_topic << std::endl;
    info->pub_->publish(*m);
  }
}
