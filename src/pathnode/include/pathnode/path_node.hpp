#ifndef PATH_NODE_HPP_
#define PATH_NODE_HPP_

#include "rclcpp/node.hpp"
#include "rclcpp/visibility_control.hpp"

#include "path_info_msg/msg/path_info.hpp"

namespace pathnode
{

struct PathNodeSubscriptionOptions
{
  std::string path_name_;
  bool is_first_;
  rclcpp::Duration path_deadline_duration_;
  std::vector<std::string> publish_topic_names_;

  PathNodeSubscriptionOptions();
};

// TODO: hide me
struct PathNodeInfo
{
  rcl_clock_type_t CLOCK_TYPE;
  std::string path_name_;
  // main subscription topic
  std::string subscription_topic_name_;
  // topics published in the main subscription callback
  std::vector<std::string> publish_topic_names_;
  rclcpp::Time path_start_time_;
  rclcpp::Duration path_deadline_duration_;

  rclcpp::Publisher<path_info_msg::msg::PathInfo>::SharedPtr pub_;
  rclcpp::Subscription<path_info_msg::msg::PathInfo>::SharedPtr sub_;

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

  /**
   * for now, we can use `const &` only for CallbackT arguments
   * TODO: support other types
   */
  template<
    typename MessageT,
    typename CallbackT,
    typename AllocatorT = std::allocator<void>,
    typename CallbackMessageT =
    typename rclcpp::subscription_traits::has_message_type<CallbackT>::type,
    typename CallbackArgT =
    typename rclcpp::function_traits::function_traits<CallbackT>::template argument_type<0>,
    typename SubscriptionT = rclcpp::Subscription<CallbackMessageT, AllocatorT>,
    typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<
      CallbackMessageT,
      AllocatorT
    >
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
      info.subscription_topic_name_ = topic_name;
      info.publish_topic_names_ = path_node_options.publish_topic_names_;
      info.pub_ = this->create_publisher<path_info_msg::msg::PathInfo>(path_name + "_info", qos);
      auto path_info_callback =
          [this, &info](path_info_msg::msg::PathInfo::UniquePtr msg) -> void
          {
            if(msg->topic_name != info.subscription_topic_name_) return;

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
        //[this, &path_node_options, &callback](MessageT msg) -> void
        [this, &path_node_options, &callback](CallbackArgT msg) -> void
        {
          std::cout << "main_topic_callback" << std::endl;
          // TODO: make path_node_options const (need C++14?)

          const auto& path_name = path_node_options.path_name_;
          const auto is_first = path_node_options.is_first_;
          auto info_it = this->path_node_info_map_.find(path_name);
          if(info_it == this->path_node_info_map_.end()) {
            return callback(msg);
          }

          auto info = info_it->second;

          auto m = std::make_unique<path_info_msg::msg::PathInfo>();
          if(is_first) {
            info.path_start_time_ = now();
            info.path_deadline_duration_ = path_node_options.path_deadline_duration_;

            m->path_start = info.path_start_time_;
            m->path_deadline_duration = info.path_deadline_duration_;
          } else {
            // TODO validate path_info
            m->path_start = info.path_start_time_;
            m->path_deadline_duration = info.path_deadline_duration_;
          }

          for(const auto &pub_topic : info.publish_topic_names_) {
            m->topic_name = pub_topic;
            info.pub_->publish(*m);
          }

          // finally, call original function
          callback(msg);
        };

    return create_subscription<MessageT>(
        topic_name,
        qos,
        main_topic_callback,
        options,
        msg_mem_strat);
  }

protected:

  bool exceeds_deadline(const std::string& path) const
  {
    auto path_node_it = path_node_info_map_.find(path);
    if(path_node_it == path_node_info_map_.end()) {
      // TODO: return bool? raise exception?
      return true;
    }
    const auto &info = path_node_it->second;

    std::cout << std::fixed
              << "now: " << now_ns()
              << " path_start: " << info.path_start_time_.nanoseconds()
              << " deadline: " << std::to_string(info.path_deadline_duration_.nanoseconds()) << std::endl;

    // TODO: verify path_info.valid_ns
    if(info.path_start_time_ + info.path_deadline_duration_ <= now()) {
      std::cout << "path overrun!" << std::endl;
      return true;
    }

    return false;
  }

private:
  std::map<std::string, PathNodeInfo> path_node_info_map_;
  rcl_clock_type_t CLOCK_TYPE;

  rclcpp::Time now() const
  {
    rclcpp::Clock ros_clock(CLOCK_TYPE);
    return ros_clock.now();
  }

  double now_ns() const
  {
    return now().nanoseconds();
  }
};

} // namespace pathnode

#endif // PATH_NODE_HPP_
