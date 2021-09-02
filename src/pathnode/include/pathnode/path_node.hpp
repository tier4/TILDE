#ifndef PATH_NODE_HPP_
#define PATH_NODE_HPP_

#include "rclcpp/node.hpp"
#include "rclcpp/visibility_control.hpp"

#include "path_info_msg/msg/path_info.hpp"

namespace pathnode
{

class PathNodeSubscriptionOptions
{
public:
  std::string path_name_;
  bool is_first_;
  rclcpp::Duration path_deadline_duration_;

  PathNodeSubscriptionOptions(
      const std::string &path_name,
      bool is_first,
      const rclcpp::Duration &path_deadline_duration);
};

class PathNodeInfo
{
public:
  std::string path_name_;
  std::string topic_name_;
  rclcpp::Time path_start_time_;
  rclcpp::Duration path_deadline_duration_;
  rcl_clock_type_t CLOCK_TYPE;

  rclcpp::Publisher<path_info_msg::msg::PathInfo>::UniquePtr pub_;
  rclcpp::Subscription<path_info_msg::msg::PathInfo>::UniquePtr sub_;

  PathNodeInfo();
};

class PathNode : public rclcpp::Node
{
public:
  RCLCPP_PUBLIC
  explicit PathNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RCLCPP_PUBLIC
  explicit PathNode(
    const std::string & node_name,
    const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RCLCPP_PUBLIC
  virtual ~PathNode();

  template<
    typename MessageT,
    typename CallbackT,
    typename AllocatorT = std::allocator<void>,
    typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>,
    typename MessageMemoryStrategyT = typename SubscriptionT::MessageMemoryStrategyType
  >
  std::shared_ptr<SubscriptionT>
  create_path_node_subscription(
    const PathNodeSubscriptionOptions &path_node_options,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    CallbackT && callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options =
    rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>(),
    typename MessageMemoryStrategyT::SharedPtr msg_mem_strat = (
      MessageMemoryStrategyT::create_default()
    ))
  {
    const auto& path_name = path_node_options.path_name_;

    if(path_node_info_map_.find(path_name) == path_node_info_map_.end()) {
      PathNodeInfo info;
      info.path_name_ = path_name;
      info.topic_name_ = topic_name;
      info.pub_ = this->create_publisher<path_info_msg::msg::PathInfo>(path_name + "_info", qos);
      auto path_info_callback =
          [this, &info](path_info_msg::msg::PathInfo::UniquePtr msg) -> void
          {
            if(msg->topic_name != info.sub_->get_topic_name()) return;

            // TODO: validate msg->valid_ns
            info.path_start_time_ = rclcpp::Time(msg->path_start, info.CLOCK_TYPE);
            info.path_deadline_duration_ = msg->path_deadline_duration;
            std::cout << "get path_info: "
                      << " topic: "  << msg->topic_name
                      << " path_start: " << info.path_start_time_.nanoseconds() << std::endl;
          };
      info.sub_ = this->create_subscription<path_info_msg::msg::PathInfo>(
          path_name + "_info", qos, path_info_callback);

      path_node_info_map_[path_name] = info;
    }

    auto main_topic_callback =
        [this, callback](MessageT msg) -> void
        {
          std::cout << "prehook" << std::endl;
          callback(msg);
        };

    return create_subscription(topic_name,
                               qos,
                               callback,
                               options,
                               msg_mem_strat);
  }

private:
  std::map<std::string, PathNodeInfo> path_node_info_map_;
};

} // namespace pathnode

#endif // PATH_NODE_HPP_
