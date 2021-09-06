#include "pathnode/path_node.hpp"

using pathnode::PathNodeSubscriptionOptions;
using pathnode::PathNodeInfo;
using pathnode::PathNode;

PathNodeSubscriptionOptions::PathNodeSubscriptionOptions():
    valid_min_(0, 0), valid_max_(0, 0)
{
}

PathNodeInfo::PathNodeInfo():
    CLOCK_TYPE(RCL_SYSTEM_TIME),
    valid_min_(0, 0), valid_max_(0, 0)
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
    const PathNodeSubscriptionOptions &path_node_options)
{
  const auto& path_name = path_node_options.path_name_;

  // create path_info pub/sub
  if(path_node_info_map_.find(path_name) == path_node_info_map_.end()) {
    auto info = std::make_shared<PathNodeInfo>();
    rclcpp::QoS qos(1);

    info->path_name_ = path_name;
    info->is_first_ = path_node_options.is_first_;
    info->valid_min_ = path_node_options.valid_min_;
    info->valid_max_ = path_node_options.valid_max_;

    if(info->is_first_) {
      info->pub_ = this->create_publisher<path_info_msg::msg::PathInfo>(path_name + "_info", qos);
    } else {
      auto path_info_callback =
          [this](path_info_msg::msg::PathInfo::UniquePtr msg) -> void
          {
            auto info_it = path_node_info_map_.find(msg->path_name);
            if(info_it == path_node_info_map_.end()) {
              std::cout << "path_info not found: " << msg->path_name << std::endl;
              return;
            }
            auto info = info_it->second;
            info->path_tickets_.insert(rclcpp::Time(msg->path_start, info->CLOCK_TYPE));
          };
      info->sub_ = this->create_subscription<path_info_msg::msg::PathInfo>(
          path_name + "_info", qos, path_info_callback);
    }

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

  auto &info = info_it->second;

  if(!info->is_first_) {
    return;
  }

  auto m = std::make_unique<path_info_msg::msg::PathInfo>();
  m->path_name = path_name;
  m->path_start = now();
  info->pub_->publish(m);
}

rclcpp::Time PathNode::pop_path_start_time(const std::string& path)
{
  rclcpp::Time ret(0, 0, CLOCK_TYPE);
  auto path_node_it = path_node_info_map_.find(path);
  if(path_node_it == path_node_info_map_.end()) {
    std::cout << "cannot find path_node_info: " << std::endl;;
    return ret;
  }
  auto &info = path_node_it->second;

  // TODO: verify path_info.valid_ns
  auto &tickets = info->path_tickets_;

  auto nw = now();
  for(auto it=tickets.begin(); it!=tickets.end(); it++) {
    // if ticket is too old, remove it
    if(*it + info->valid_max_ < nw) {
      it = tickets.erase(it);
      continue;
    }
    if(*it - info->valid_min_ < nw && nw < *it + info->valid_max_) {
      ret = *it;
      it = tickets.erase(it);
      break;
    }
  }

  return ret;
}

