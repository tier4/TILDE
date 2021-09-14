#ifndef PATH_NODE_HPP_
#define PATH_NODE_HPP_

#include <set>
#include <mutex>

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

    setup_path(path_node_options);

    // add pre-process to main callback
    // if callback is reference(i.e. &callback), SEGV in callback(std::forward<...>)
    // I guess we should hold variables such as pub_ in original function (really?)
    auto main_topic_callback =
        [this, path_name, callback](CallbackArgT msg) -> void
        {
          on_pathed_subscription(path_name);

          // finally, call original function
          callback(std::forward<CallbackArgT>(msg));
        };

    return create_subscription<MessageT>(
        topic_name,
        qos,
        main_topic_callback,
        options,
        msg_mem_strat);
  }

protected:
  bool pop_path_start_time(const std::string& path, rclcpp::Time &out);
  rclcpp::Duration get_path_valid_min(const std::string &path);
  rclcpp::Duration get_path_valid_max(const std::string &path);
  rclcpp::Time now() const;
  rcl_clock_type_t CLOCK_TYPE;

private:
  std::mutex path_node_info_map_mutex_;
  std::map<std::string, std::shared_ptr<PathNodeInfo>> path_node_info_map_;
};

} // namespace pathnode

#endif // PATH_NODE_HPP_
