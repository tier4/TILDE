#ifndef PATH_NODE_HPP_
#define PATH_NODE_HPP_

#include <set>

#include "rclcpp/node.hpp"
#include "rclcpp/visibility_control.hpp"

#include "path_info_msg/msg/path_info.hpp"

namespace pathnode
{

struct PathNodeSubscriptionOptions
{
  std::string path_name_;
  bool is_first_;
  rclcpp::Duration valid_min_;
  rclcpp::Duration valid_max_;

  PathNodeSubscriptionOptions();
};

// TODO: hide me
struct PathNodeInfo
{
  using PathTicket = rclcpp::Time;
  using PathTickets = std::set<PathTicket>;

  rcl_clock_type_t CLOCK_TYPE;
  std::string path_name_;

  bool is_first_;
  PathTickets path_tickets_;
  rclcpp::Duration valid_min_;
  rclcpp::Duration valid_max_;

  rclcpp::Publisher<path_info_msg::msg::PathInfo>::SharedPtr pub_;
  rclcpp::Subscription<path_info_msg::msg::PathInfo>::SharedPtr sub_;

  PathNodeInfo();
};

class PathNode : public rclcpp::Node
{
public:
  enum deadline_t {
    OK,
    OVERRUN,
    NO_INFO
  };

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

  void setup_path(
      const PathNodeSubscriptionOptions &path_node_options);


  /// send path info at the beginning of main-topic subscription callback
  void on_pathed_subscription(const std::string &path_name);

  /// DON'T USE ME. create custom subscription
  /**
   * for now, we can use `const &` only for CallbackT arguments
   * TODO: support other types
   * TODO: in this implementation, original `callback(msg)` fails
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

    // setup path_info_map
    if(path_node_info_map_.find(path_name) == path_node_info_map_.end()) {
      PathNodeInfo info;
      info.path_name_ = path_name;
      info.is_first_ = path_node_options.is_first_;
      info.pub_ = this->create_publisher<path_info_msg::msg::PathInfo>(path_name + "_info", qos);
      auto path_info_callback =
          [this, &info](path_info_msg::msg::PathInfo::UniquePtr msg) -> void
          {
            auto st = rclcpp::Time(msg->path_start, info.CLOCK_TYPE);
            info.path_tickets_.insert(st);
            std::cout << "get path_info: "
                      << " path_start: " << st.nanoseconds() << std::endl;
          };
      info.sub_ = this->create_subscription<path_info_msg::msg::PathInfo>(
          path_name + "_info", qos, path_info_callback);

      std::cout << "create_path_info: " << path_name << std::endl;
      path_node_info_map_[path_name] = info;
    }

    // add pre-process to main callback
    auto main_topic_callback =
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

          // finally, call original function
          callback(msg);
          std::cout << "end" << std::endl;
        };

    return create_subscription<MessageT>(
        topic_name,
        qos,
        main_topic_callback,
        options,
        msg_mem_strat);
  }

protected:

  deadline_t exceeds_deadline(const std::string& path) const
  {
    auto path_node_it = path_node_info_map_.find(path);
    if(path_node_it == path_node_info_map_.end()) {
      std::cout << "cannot find path_node_info: " << std::endl;;
      return NO_INFO;
    }
    auto &info = path_node_it->second;

    // TODO: verify path_info.valid_ns
    auto &tickets = info->path_tickets_;
    deadline_t ret = OVERRUN;

    auto nw = now();
    for(auto it=tickets.begin(); it!=tickets.end(); it++) {
      // if ticket is too old, remove it
      if(*it + info->valid_max_ < nw) {
        it = tickets.erase(it);
        continue;
      }
      if(*it - info->valid_min_ < nw && nw < *it + info->valid_max_) {
        ret = OK;
        it = tickets.erase(it);
        break;
      }
    }

    return ret;
  }

private:
  std::map<std::string, std::shared_ptr<PathNodeInfo>> path_node_info_map_;
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
